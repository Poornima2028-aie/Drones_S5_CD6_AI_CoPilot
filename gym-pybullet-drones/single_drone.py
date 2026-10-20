#!/usr/bin/env python3
"""
Manual Drone + Automatic Static AND Dynamic Obstacle Avoidance
==============================================================

CONTROLS:
    W = Forward      S = Backward
    A = Left         D = Right
    E = Up           Q = Down
    ESC = Exit

WHAT CHANGED vs. the previous version:
    1. DYNAMIC OBSTACLES: three moving spheres (patrolling, orbiting,
       bobbing up/down). Avoidance predicts where each obstacle will be
       (closest point of approach) instead of only looking at its
       current position.
    2. SMOOTH CONTROL: the commanded velocity is acceleration-limited
       (slew-rate limited), so the drone can never be asked to jump
       from 0 to full speed. This was the main cause of the "going
       off" behaviour.
    3. NO SETPOINT WIND-UP: the target position is kept on a short
       "leash" (LEASH m) around the real drone position, so the PID
       never chases a far-away target.
    4. FIXED AUTO MODE: the old AUTO mode fed the drone's own velocity
       back into the setpoint (positive feedback). Now AUTO computes a
       clean repulsion velocity and goes through the same limiter.
    5. AUTO no longer gets stuck hovering forever near an obstacle.
    6. Speeds are lower, yaw is held fixed, cameras render every
       CAM_EVERY steps so the simulation keeps real-time pace.
"""

import time
import threading

import cv2
import numpy as np
import pybullet as p

from pynput import keyboard

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.utils.utils import sync


# ============================================================
# CONFIGURATION  (tune these if you want it gentler / snappier)
# ============================================================

PYB_FREQ = 240
CTRL_FREQ = 48

START_POS = np.array([0.0, 0.0, 1.0])

# Arena limits
Z_MIN = 0.35
Z_MAX = 2.5
XY_LIMIT = 4.5

# ---- Motion limits (lower = calmer, more stable) ----
MAX_SPEED = 0.50          # manual top speed, m/s
AVOID_SPEED = 0.60        # normal automatic avoidance speed, m/s
EMERGENCY_SPEED = 0.90    # speed when an obstacle is very close, m/s
MAX_ACCEL = 1.0           # m/s^2, normal acceleration limit
EMERGENCY_ACCEL = 2.2     # m/s^2, allowed during emergency avoidance
LEASH = 0.30              # max distance of setpoint from real position, m

# ---- Avoidance distances (surface-to-surface gaps), metres ----
DRONE_RADIUS = 0.10
AVOID_DISTANCE = 1.5      # start avoiding inside this gap
EMERGENCY_DISTANCE = 0.65 # override the pilot inside this gap
PREDICT_RANGE = 2.5       # only consider obstacles closer than this
PREDICT_HORIZON = 2.0     # seconds to look ahead for moving obstacles

# How long the path must stay clear before control returns to pilot
RELEASE_HOLD = 0.8

# AUTO can never hold control longer than this (unless in emergency)
AUTO_MAX_TIME = 3.0

# After control returns to the pilot, AUTO will not re-trigger from
# a mere "threat" for this long (emergency can still take over)
MANUAL_GRACE = 1.5

# Camera
CAM_W = 480
CAM_H = 360
CAM_EVERY = 2             # render cameras every N control steps

# Static obstacles
PILLAR = (2.0, 0.0, 0.25, 2.5)          # x, y, radius, height
BOX_POS = np.array([3.0, 1.5, 1.0])
BOX_HALF = 0.30


# ============================================================
# GLOBAL KEYBOARD
# ============================================================

class KeyInput:
    def __init__(self):
        self.lock = threading.Lock()
        self.pressed = set()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.daemon = True
        self.listener.start()

    @staticmethod
    def key_name(key):
        if isinstance(key, keyboard.KeyCode):
            return key.char.lower() if key.char else ""
        if key == keyboard.Key.esc:
            return "esc"
        return str(key).replace("Key.", "").lower()

    def on_press(self, key):
        with self.lock:
            self.pressed.add(self.key_name(key))

    def on_release(self, key):
        with self.lock:
            self.pressed.discard(self.key_name(key))

    def down(self, name):
        with self.lock:
            return name in self.pressed

    def close(self):
        self.listener.stop()


# ============================================================
# OBSTACLES  (static + dynamic)
# ============================================================
# Every obstacle is a dict:
#   id      : pybullet body id
#   kind    : "cylinder" (infinite in height -> horizontal distance)
#             or "sphere" (full 3D distance)
#   radius  : collision radius used for avoidance
#   pos/vel : current position / velocity (updated each step if moving)
#   motion  : None for static, or function(t) -> (pos, vel)

def create_pillar(client):
    x, y, radius, height = PILLAR

    col = p.createCollisionShape(
        p.GEOM_CYLINDER, radius=radius, height=height,
        physicsClientId=client
    )
    vis = p.createVisualShape(
        p.GEOM_CYLINDER, radius=radius, length=height,
        rgbaColor=[0.55, 0.58, 0.65, 1.0], physicsClientId=client
    )
    body = p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[x, y, height / 2], physicsClientId=client
    )

    return {
        "id": body, "kind": "cylinder", "radius": radius,
        "pos": np.array([x, y, height / 2]), "vel": np.zeros(3),
        "motion": None,
    }


def create_box(client):
    col = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[BOX_HALF] * 3, physicsClientId=client
    )
    vis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[BOX_HALF] * 3,
        rgbaColor=[1.0, 0.48, 0.08, 1.0], physicsClientId=client
    )
    body = p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=BOX_POS.tolist(), physicsClientId=client
    )

    # A sphere of radius ~1.8 x half-size safely wraps the cube.
    return {
        "id": body, "kind": "sphere", "radius": BOX_HALF * 1.8,
        "pos": BOX_POS.copy(), "vel": np.zeros(3), "motion": None,
    }


# ---- Motion patterns for the dynamic obstacles ----

def motion_patrol(t):
    """Sweeps back and forth along Y at x = 1.0 (max ~0.7 m/s)."""
    w, amp = 0.35, 2.0
    pos = np.array([1.0, amp * np.sin(w * t), 1.0])
    vel = np.array([0.0, amp * w * np.cos(w * t), 0.0])
    return pos, vel


def motion_orbit(t):
    """Circles around (-1.5, 1.0) at height 1.2 (~0.48 m/s)."""
    w, r = 0.40, 1.2
    cx, cy = -1.5, 1.0
    pos = np.array([cx + r * np.cos(w * t),
                    cy + r * np.sin(w * t), 1.2])
    vel = np.array([-r * w * np.sin(w * t),
                    r * w * np.cos(w * t), 0.0])
    return pos, vel


def motion_bob(t):
    """Moves up and down at (2.5, -1.5) between ~0.6 m and ~1.8 m."""
    w, amp = 0.80, 0.6
    pos = np.array([2.5, -1.5, 1.2 + amp * np.sin(w * t)])
    vel = np.array([0.0, 0.0, amp * w * np.cos(w * t)])
    return pos, vel


def create_dynamic_sphere(client, radius, color, motion):
    col = p.createCollisionShape(
        p.GEOM_SPHERE, radius=radius, physicsClientId=client
    )
    vis = p.createVisualShape(
        p.GEOM_SPHERE, radius=radius, rgbaColor=color,
        physicsClientId=client
    )

    pos, vel = motion(0.0)

    body = p.createMultiBody(
        baseMass=0, baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=pos.tolist(), physicsClientId=client
    )

    return {
        "id": body, "kind": "sphere", "radius": radius,
        "pos": pos, "vel": vel, "motion": motion,
    }


def update_dynamic_obstacles(client, obstacles, t):
    """Move every dynamic obstacle to its position at time t."""
    for ob in obstacles:
        if ob["motion"] is None:
            continue

        pos, vel = ob["motion"](t)
        ob["pos"] = pos
        ob["vel"] = vel

        p.resetBasePositionAndOrientation(
            ob["id"], pos.tolist(), [0, 0, 0, 1],
            physicsClientId=client
        )


# ============================================================
# THREAT ASSESSMENT  (works for static AND moving obstacles)
# ============================================================

def assess_obstacles(pos, vel, obstacles):
    """
    For each obstacle, predict the point of closest approach within
    PREDICT_HORIZON seconds and push the drone away from it.

    Returns:
        escape_vec : summed avoidance direction (norm in [0, ~N])
        nearest    : smallest current surface gap
        threat     : True if any obstacle requires avoidance
    """
    total = np.zeros(3)
    nearest = float("inf")
    threat = False

    for ob in obstacles:
        # Cylinders are infinitely tall: ignore the Z axis.
        mask = (
            np.array([1.0, 1.0, 0.0])
            if ob["kind"] == "cylinder"
            else np.ones(3)
        )

        rel = (ob["pos"] - pos) * mask
        rel_vel = (ob["vel"] - vel) * mask

        dist = float(np.linalg.norm(rel))
        gap = dist - ob["radius"] - DRONE_RADIUS
        nearest = min(nearest, gap)

        if gap > PREDICT_RANGE:
            continue

        # ---- Closest point of approach (CPA) ----
        rel_speed2 = float(rel_vel @ rel_vel)

        if rel_speed2 > 1e-6:
            t_cpa = float(np.clip(
                -(rel @ rel_vel) / rel_speed2, 0.0, PREDICT_HORIZON
            ))
        else:
            t_cpa = 0.0

        pred = rel + rel_vel * t_cpa
        pred_gap = (
            float(np.linalg.norm(pred)) - ob["radius"] - DRONE_RADIUS
        )

        effective_gap = min(gap, pred_gap)

        strength = float(np.clip(
            (AVOID_DISTANCE - effective_gap) / AVOID_DISTANCE,
            0.0, 1.0
        ))

        if strength <= 0.0:
            continue

        threat = True

        # ---- Direction: away from where the obstacle WILL be ----
        away = -pred if np.linalg.norm(pred) > 1e-3 else -rel

        if np.linalg.norm(away) < 1e-6:
            away = np.array([-1.0, 0.0, 0.0])

        away = away / np.linalg.norm(away)

        # Spheres: allow some vertical escape, but gently.
        away[2] *= 0.5

        # Add a sideways component so the drone slides AROUND the
        # obstacle instead of just backing straight up against it.
        tangent = np.array([-away[1], away[0], 0.0])
        side = 1.0 if float(tangent @ vel) >= 0.0 else -1.0

        direction = away + 0.5 * side * tangent
        direction = direction / np.linalg.norm(direction)

        total += strength * direction

    return total, nearest, threat


# ============================================================
# VELOCITY LIMITER
# ============================================================

def slew_velocity(current, desired, max_delta):
    """Move `current` toward `desired` by at most `max_delta` (m/s)."""
    diff = desired - current
    norm = float(np.linalg.norm(diff))

    if norm > max_delta:
        diff *= max_delta / norm

    return current + diff


# ============================================================
# CAMERA FUNCTIONS
# ============================================================

def camera_image(client, eye, target, up):
    view = p.computeViewMatrix(
        cameraEyePosition=eye.tolist(),
        cameraTargetPosition=target.tolist(),
        cameraUpVector=up.tolist()
    )

    projection = p.computeProjectionMatrixFOV(
        fov=80, aspect=CAM_W / CAM_H, nearVal=0.05, farVal=20.0
    )

    _, _, rgba, _, _ = p.getCameraImage(
        width=CAM_W, height=CAM_H,
        viewMatrix=view, projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER, physicsClientId=client
    )

    rgba = np.asarray(rgba, dtype=np.uint8).reshape(CAM_H, CAM_W, 4)

    return cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR)


def add_hud(image, lines, color):
    for i, line in enumerate(lines):
        cv2.putText(
            image, line, (10, 25 + i * 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA
        )


# ============================================================
# MAIN SIMULATION
# ============================================================

def main():
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=START_POS.reshape(1, 3),
        initial_rpys=np.zeros((1, 3)),
        physics=Physics.PYB,
        pyb_freq=PYB_FREQ,
        ctrl_freq=CTRL_FREQ,
        gui=True,
        record=False,
        obstacles=False,
        user_debug_gui=False
    )

    keys = None

    try:
        env.reset()
        client = env.CLIENT
        dt = env.CTRL_TIMESTEP

        p.setRealTimeSimulation(0, physicsClientId=client)
        p.configureDebugVisualizer(
            p.COV_ENABLE_GUI, 0, physicsClientId=client
        )

        # ---- Static + dynamic obstacles ----
        obstacles = [
            create_pillar(client),
            create_box(client),
            create_dynamic_sphere(
                client, 0.25, [0.9, 0.1, 0.1, 1.0], motion_patrol
            ),
            create_dynamic_sphere(
                client, 0.25, [0.1, 0.4, 0.95, 1.0], motion_orbit
            ),
            create_dynamic_sphere(
                client, 0.22, [0.2, 0.85, 0.3, 1.0], motion_bob
            ),
        ]

        controller = DSLPIDControl(drone_model=DroneModel.CF2X)
        keys = KeyInput()

        state = env._getDroneStateVector(0)
        target_pos = state[0:3].copy()
        target_yaw = 0.0                 # yaw is held fixed
        cmd_vel = np.zeros(3)            # smoothed commanded velocity

        mode = "MANUAL"
        clear_timer = 0.0
        auto_timer = 0.0
        grace_timer = 0.0

        start_time = time.time()
        step_count = 0

        front = chase = None

        print("\n========================================")
        print(" MANUAL DRONE + AUTO OBSTACLE AVOIDANCE")
        print("   (static + dynamic obstacles)")
        print("========================================")
        print("W/S = Forward/Back | A/D = Left/Right")
        print("E/Q = Up/Down      | ESC = Quit")
        print("Release the keys to let the drone avoid obstacles.")
        print("Red / blue / green spheres are moving obstacles.\n")

        while True:
            if keys.down("esc"):
                break

            sim_time = step_count * dt

            # Move dynamic obstacles BEFORE stepping physics.
            update_dynamic_obstacles(client, obstacles, sim_time)

            state = env._getDroneStateVector(0)

            pos = state[0:3].copy()
            quat = state[3:7].copy()
            rpy = state[7:10].copy()
            vel = state[10:13].copy()
            yaw = float(rpy[2])

            # ------------------------------------------------
            # MANUAL INPUT
            # ------------------------------------------------
            forward_cmd = float(keys.down("w")) - float(keys.down("s"))
            left_cmd = float(keys.down("a")) - float(keys.down("d"))
            vertical_cmd = float(keys.down("e")) - float(keys.down("q"))

            manual_active = (
                forward_cmd != 0.0
                or left_cmd != 0.0
                or vertical_cmd != 0.0
            )

            fwd = np.array([np.cos(yaw), np.sin(yaw)])
            lft = np.array([-np.sin(yaw), np.cos(yaw)])

            manual_velocity = np.array([
                forward_cmd * fwd[0] + left_cmd * lft[0],
                forward_cmd * fwd[1] + left_cmd * lft[1],
                vertical_cmd
            ])

            # Normalise so diagonals are not faster.
            mnorm = np.linalg.norm(manual_velocity)
            if mnorm > 1.0:
                manual_velocity /= mnorm
            manual_velocity *= MAX_SPEED

            # ------------------------------------------------
            # THREAT ASSESSMENT
            # ------------------------------------------------
            escape_vec, nearest, threat = assess_obstacles(
                pos, vel, obstacles
            )

            emergency = nearest < EMERGENCY_DISTANCE

            # ------------------------------------------------
            # MODE SWITCHING
            # ------------------------------------------------
            grace_timer = max(0.0, grace_timer - dt)

            if mode == "MANUAL":
                clear_timer = 0.0
                auto_timer = 0.0

                # AUTO takes over when:
                #  - an obstacle is dangerously close (always), or
                #  - there is a threat, the pilot released the keys,
                #    and we are not in the post-handover grace period.
                if threat and (
                    emergency
                    or (not manual_active and grace_timer <= 0.0)
                ):
                    mode = "AUTO"
                    print("[AUTO] Obstacle detected. Taking control.")

            else:  # AUTO
                auto_timer += dt

                if threat:
                    clear_timer = 0.0
                else:
                    clear_timer += dt

                release_reason = None

                if clear_timer >= RELEASE_HOLD:
                    release_reason = "Path clear."
                elif manual_active and not emergency:
                    # Pilot presses a key -> pilot wins.
                    release_reason = "Pilot override."
                elif auto_timer >= AUTO_MAX_TIME and not emergency:
                    # Safety net: never stay in AUTO forever.
                    release_reason = "AUTO timeout."

                if release_reason is not None:
                    mode = "MANUAL"
                    target_pos = pos.copy()
                    grace_timer = MANUAL_GRACE
                    print(f"[MANUAL] {release_reason} Control restored.")

            # ------------------------------------------------
            # DESIRED VELOCITY
            # ------------------------------------------------
            if mode == "MANUAL":
                desired_vel = manual_velocity
                accel = MAX_ACCEL

            else:
                mag = float(np.linalg.norm(escape_vec))

                if threat and mag > 1e-6:
                    speed = AVOID_SPEED * min(1.0, mag)

                    if emergency:
                        speed = EMERGENCY_SPEED

                    desired_vel = escape_vec / mag * speed
                else:
                    # Nothing to avoid right now: brake and hover.
                    desired_vel = np.zeros(3)

                accel = EMERGENCY_ACCEL if emergency else MAX_ACCEL

            # Smooth the command: this is what keeps the drone calm.
            cmd_vel = slew_velocity(cmd_vel, desired_vel, accel * dt)

            # ------------------------------------------------
            # SETPOINT (integrated, clipped, and on a short leash)
            # ------------------------------------------------
            target_pos = target_pos + cmd_vel * dt

            target_pos[0] = np.clip(target_pos[0], -XY_LIMIT, XY_LIMIT)
            target_pos[1] = np.clip(target_pos[1], -XY_LIMIT, XY_LIMIT)
            target_pos[2] = np.clip(target_pos[2], Z_MIN, Z_MAX)

            err = target_pos - pos
            err_norm = float(np.linalg.norm(err))
            if err_norm > LEASH:
                target_pos = pos + err * (LEASH / err_norm)

            # ------------------------------------------------
            # PID MOTOR CONTROL
            # ------------------------------------------------
            rpm, _, _ = controller.computeControlFromState(
                control_timestep=dt,
                state=state,
                target_pos=target_pos,
                target_rpy=np.array([0.0, 0.0, target_yaw]),
                target_vel=cmd_vel
            )

            env.step(np.asarray(rpm).reshape(1, 4))

            # ------------------------------------------------
            # CAMERAS (not every step, to keep real-time pace)
            # ------------------------------------------------
            if step_count % CAM_EVERY == 0 or front is None:
                rotation = np.array(
                    p.getMatrixFromQuaternion(quat)
                ).reshape(3, 3)

                drone_forward = rotation[:, 0]
                drone_up = rotation[:, 2]

                front = camera_image(
                    client,
                    pos + 0.08 * drone_forward,
                    pos + 5.0 * drone_forward,
                    drone_up
                )

                horizontal_forward = np.array([
                    np.cos(yaw), np.sin(yaw), 0.0
                ])

                chase = camera_image(
                    client,
                    pos - 3.0 * horizontal_forward
                    + np.array([0.0, 0.0, 1.5]),
                    pos,
                    np.array([0.0, 0.0, 1.0])
                )

                color = (
                    (0, 200, 0) if mode == "MANUAL" else (0, 0, 255)
                )

                gap_text = (
                    f"{nearest:.2f} m" if np.isfinite(nearest) else "N/A"
                )

                add_hud(front, [
                    "FRONT CAMERA",
                    f"MODE: {mode}",
                    f"Altitude: {pos[2]:.2f} m",
                    f"Speed: {np.linalg.norm(vel):.2f} m/s",
                    f"Nearest obstacle: {gap_text}",
                ], color)

                add_hud(chase, [
                    "WIDE CHASE CAMERA",
                    f"MODE: {mode}",
                    "W/S fwd/back | A/D left/right | E/Q up/down",
                ], color)

                cv2.imshow("Front Camera", front)
                cv2.imshow("Wide Chase Camera", chase)

            if (cv2.waitKey(1) & 0xFF) == 27:
                break

            step_count += 1
            sync(step_count, start_time, dt)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    finally:
        if keys is not None:
            keys.close()

        cv2.destroyAllWindows()
        env.close()

        print("Drone simulation closed.")


if __name__ == "__main__":
    main()