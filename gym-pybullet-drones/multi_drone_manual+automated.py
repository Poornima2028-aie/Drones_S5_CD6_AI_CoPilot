#!/usr/bin/env python3
"""
Two Drones: Manual Control + Automatic Avoidance (per drone)
============================================================

You fly BOTH drones at the same time, one with each hand:

                 DRONE 1 (blue)        DRONE 2 (pink)
    Forward          W                     I
    Backward         S                     K
    Left             A                     J
    Right            D                     L
    Up               E                     O
    Down             Q                     U
    Turn left        Z                     N
    Turn right       C                     M

    ESC = Exit

HOW IT WORKS (independently for each drone):
    - While you hold a key for a drone, that drone is MANUAL.
    - If you release ALL of that drone's keys and an obstacle is in
      its way, that drone switches to AUTO and avoids it by itself,
      while the other drone stays fully under your control.
    - AUTO hands control back as soon as you press one of that
      drone's keys again, when the path is clear, or after
      AUTO_MAX_TIME seconds.
    - If a drone gets dangerously close to an obstacle (or to the
      other drone), AUTO overrides your keys until it is safe.

Obstacles: static pillar + box, three moving spheres, and the other
drone (each drone treats the other one as a moving obstacle).

NOTE: pynput reads the keyboard globally, so the keys work even when
the window is not focused. Some keyboards cannot register many keys
pressed at once ("ghosting"); if a key seems ignored while you hold
many others, that is a hardware limit, not the code.
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
# CONFIGURATION
# ============================================================

PYB_FREQ = 240
CTRL_FREQ = 48

NUM_DRONES = 2

STARTS = np.array([
    [-3.5, -0.8, 1.0],
    [-3.5, -2.4, 1.5],
])

# Key bindings, one dict per drone
KEYMAPS = [
    dict(fwd="w", back="s", left="a", right="d",
         up="e", down="q", yaw_left="z", yaw_right="c"),
    dict(fwd="i", back="k", left="j", right="l",
         up="o", down="u", yaw_left="n", yaw_right="m"),
]

DRONE_RGBA = [(0.1, 0.45, 1.0, 1.0), (1.0, 0.2, 0.65, 1.0)]
DRONE_BGR = [(255, 115, 25), (165, 50, 255)]

# Arena limits
Z_MIN = 0.35
Z_MAX = 2.5
XY_LIMIT = 4.5

# ---- Motion limits (gentle = stable) ----
MAX_SPEED = 0.50
AVOID_SPEED = 0.60
EMERGENCY_SPEED = 0.90
MAX_ACCEL = 1.0
EMERGENCY_ACCEL = 2.2
LEASH = 0.30
YAW_RATE = 0.8            # rad/s while a turn key is held
WARMUP_TIME = 1.0         # seconds of hover before keys are accepted

# ---- Avoidance (surface-to-surface gaps), metres ----
DRONE_RADIUS = 0.10
OTHER_DRONE_RADIUS = 0.12
AVOID_DISTANCE = 1.5
EMERGENCY_DISTANCE = 0.65
PREDICT_RANGE = 2.5
PREDICT_HORIZON = 2.0

# ---- Mode switching ----
RELEASE_HOLD = 0.8        # path must be clear this long to leave AUTO
AUTO_MAX_TIME = 3.0       # AUTO never lasts longer (unless emergency)
MANUAL_GRACE = 1.5        # after handover, no new AUTO unless emergency

# ---- Cameras ----
TILE_W = 320
TILE_H = 240
TRAILS = True

# ---- Static obstacles ----
PILLAR = (2.0, 0.0, 0.25, 2.5)          # x, y, radius, height
BOX_POS = np.array([3.0, 1.5, 1.0])
BOX_HALF = 0.30


# ============================================================
# KEYBOARD
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
# OBSTACLES (static + dynamic)
# ============================================================

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

    return {
        "id": body, "kind": "sphere", "radius": BOX_HALF * 1.8,
        "pos": BOX_POS.copy(), "vel": np.zeros(3), "motion": None,
    }


def motion_patrol(t):
    """Sweeps along Y at x = 1.0 (max ~0.7 m/s)."""
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
    """Moves up and down at (2.5, -2.2) between ~0.6 m and ~1.8 m."""
    w, amp = 0.80, 0.6
    pos = np.array([2.5, -2.2, 1.2 + amp * np.sin(w * t)])
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
# THREAT ASSESSMENT (static, moving, and the other drone)
# ============================================================

def assess_obstacles(pos, vel, heading_xy, obstacles, side_memory):
    """
    Predict the closest approach to every obstacle and build an
    avoidance vector. The drone slips past each obstacle on a
    COMMITTED side (kept in side_memory) so it does not dither.

    heading_xy: horizontal unit vector the drone is "facing" in terms
    of intended travel; used to choose the side to pass on.

    Returns: escape_vec, nearest_gap, threat, max_strength
    """
    total = np.zeros(3)
    nearest = float("inf")
    max_strength = 0.0
    active = set()

    left_of_heading = np.array([-heading_xy[1], heading_xy[0], 0.0])

    for key, ob in enumerate(obstacles):
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

        active.add(key)
        max_strength = max(max_strength, strength)

        away = -pred if np.linalg.norm(pred) > 1e-3 else -rel

        if np.linalg.norm(away) < 1e-6:
            away = -np.array([heading_xy[0], heading_xy[1], 0.0])

        away = away / np.linalg.norm(away)
        away[2] *= 0.5

        if key not in side_memory:
            offset = float(rel @ left_of_heading)

            if offset > 0.05:
                lateral = -left_of_heading
            elif offset < -0.05:
                lateral = left_of_heading
            else:
                lateral = -left_of_heading   # dead ahead -> pass right

            side_memory[key] = lateral

        direction = away + 0.9 * side_memory[key]
        direction = direction / np.linalg.norm(direction)

        total += strength * direction

    for key in list(side_memory.keys()):
        if key not in active:
            del side_memory[key]

    return total, nearest, len(active) > 0, max_strength


# ============================================================
# HELPERS
# ============================================================

def slew_velocity(current, desired, max_delta):
    diff = desired - current
    norm = float(np.linalg.norm(diff))

    if norm > max_delta:
        diff *= max_delta / norm

    return current + diff


def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def camera_image(client, eye, target, up, fov=70):
    view = p.computeViewMatrix(
        cameraEyePosition=np.asarray(eye).tolist(),
        cameraTargetPosition=np.asarray(target).tolist(),
        cameraUpVector=np.asarray(up).tolist()
    )

    projection = p.computeProjectionMatrixFOV(
        fov=fov, aspect=TILE_W / TILE_H, nearVal=0.05, farVal=30.0
    )

    _, _, rgba, _, _ = p.getCameraImage(
        width=TILE_W, height=TILE_H,
        viewMatrix=view, projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER, physicsClientId=client
    )

    rgba = np.asarray(rgba, dtype=np.uint8).reshape(TILE_H, TILE_W, 4)

    return cv2.cvtColor(rgba[:, :, :3], cv2.COLOR_RGB2BGR)


def label_tile(image, lines, scale=0.5):
    """lines: list of (text, bgr_colour)."""
    for i, (text, color) in enumerate(lines):
        cv2.putText(
            image, text, (8, 20 + i * 20),
            cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA
        )


def mode_color(mode):
    return (0, 200, 0) if mode == "MANUAL" else (0, 0, 255)


# ============================================================
# MAIN SIMULATION
# ============================================================

def main():
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=NUM_DRONES,
        initial_xyzs=STARTS.copy(),
        initial_rpys=np.zeros((NUM_DRONES, 3)),
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

        p.resetDebugVisualizerCamera(
            cameraDistance=9.0, cameraYaw=0, cameraPitch=-45,
            cameraTargetPosition=[0, 0, 1], physicsClientId=client
        )

        for i in range(NUM_DRONES):
            try:
                p.changeVisualShape(
                    env.DRONE_IDS[i], -1,
                    rgbaColor=DRONE_RGBA[i], physicsClientId=client
                )
            except Exception:
                pass

        obstacles = [
            create_pillar(client),
            create_box(client),
            create_dynamic_sphere(
                client, 0.25, [0.9, 0.1, 0.1, 1.0], motion_patrol
            ),
            create_dynamic_sphere(
                client, 0.25, [0.1, 0.9, 0.9, 1.0], motion_orbit
            ),
            create_dynamic_sphere(
                client, 0.22, [0.2, 0.85, 0.3, 1.0], motion_bob
            ),
        ]

        controllers = [
            DSLPIDControl(drone_model=DroneModel.CF2X)
            for _ in range(NUM_DRONES)
        ]

        keys = KeyInput()

        # ---- Per-drone state ----
        target_pos = [STARTS[i].copy() for i in range(NUM_DRONES)]
        cmd_vel = [np.zeros(3) for _ in range(NUM_DRONES)]
        target_yaw = [0.0] * NUM_DRONES
        mode = ["MANUAL"] * NUM_DRONES
        clear_timer = [0.0] * NUM_DRONES
        auto_timer = [0.0] * NUM_DRONES
        grace_timer = [0.0] * NUM_DRONES
        side_memory = [dict() for _ in range(NUM_DRONES)]
        nearest_gap = [float("inf")] * NUM_DRONES
        prev_pos = [STARTS[i].copy() for i in range(NUM_DRONES)]

        NUM_VIEWS = 6
        tiles = [
            np.zeros((TILE_H, TILE_W, 3), dtype=np.uint8)
            for _ in range(NUM_VIEWS)
        ]

        start_time = time.time()
        step_count = 0

        print("\n========================================")
        print(" TWO DRONES: MANUAL + AUTO AVOIDANCE")
        print("========================================")
        print("Drone 1 (blue): W/S/A/D move, Q/E down/up, Z/C turn")
        print("Drone 2 (pink): I/K/J/L move, U/O down/up, N/M turn")
        print("Release a drone's keys -> it avoids obstacles by itself.")
        print("ESC = quit.\n")

        while True:
            if keys.down("esc"):
                break

            sim_time = step_count * dt

            update_dynamic_obstacles(client, obstacles, sim_time)

            states = [
                env._getDroneStateVector(i) for i in range(NUM_DRONES)
            ]

            rpms = np.zeros((NUM_DRONES, 4))
            warming_up = sim_time < WARMUP_TIME

            for i in range(NUM_DRONES):
                state = states[i]
                pos = state[0:3].copy()
                vel = state[10:13].copy()
                yaw = float(state[9])
                km = KEYMAPS[i]

                # ---------------- MANUAL INPUT ----------------
                if warming_up:
                    forward_cmd = left_cmd = vertical_cmd = yaw_cmd = 0.0
                else:
                    forward_cmd = (
                        float(keys.down(km["fwd"]))
                        - float(keys.down(km["back"]))
                    )
                    left_cmd = (
                        float(keys.down(km["left"]))
                        - float(keys.down(km["right"]))
                    )
                    vertical_cmd = (
                        float(keys.down(km["up"]))
                        - float(keys.down(km["down"]))
                    )
                    yaw_cmd = (
                        float(keys.down(km["yaw_left"]))
                        - float(keys.down(km["yaw_right"]))
                    )

                manual_active = (
                    forward_cmd != 0.0 or left_cmd != 0.0
                    or vertical_cmd != 0.0 or yaw_cmd != 0.0
                )

                fwd = np.array([np.cos(yaw), np.sin(yaw)])
                lft = np.array([-np.sin(yaw), np.cos(yaw)])

                manual_velocity = np.array([
                    forward_cmd * fwd[0] + left_cmd * lft[0],
                    forward_cmd * fwd[1] + left_cmd * lft[1],
                    vertical_cmd
                ])

                mnorm = np.linalg.norm(manual_velocity)
                if mnorm > 1.0:
                    manual_velocity /= mnorm
                manual_velocity *= MAX_SPEED

                # ---------------- THREAT ASSESSMENT ----------------
                # The other drone counts as a moving spherical obstacle.
                others = [
                    {
                        "kind": "sphere",
                        "radius": OTHER_DRONE_RADIUS,
                        "pos": states[j][0:3].copy(),
                        "vel": states[j][10:13].copy(),
                        "motion": None,
                    }
                    for j in range(NUM_DRONES) if j != i
                ]

                # Heading used to pick the side to pass on:
                # pilot's intent > current motion > nose direction.
                hxy = manual_velocity[:2]
                if np.linalg.norm(hxy) < 1e-3:
                    hxy = vel[:2]
                if np.linalg.norm(hxy) < 0.1:
                    hxy = fwd
                heading_xy = hxy / np.linalg.norm(hxy)

                escape, nearest, threat, strength = assess_obstacles(
                    pos, vel, heading_xy, obstacles + others,
                    side_memory[i]
                )

                nearest_gap[i] = nearest
                emergency = nearest < EMERGENCY_DISTANCE

                # ---------------- MODE SWITCHING ----------------
                grace_timer[i] = max(0.0, grace_timer[i] - dt)

                if mode[i] == "MANUAL":
                    clear_timer[i] = 0.0
                    auto_timer[i] = 0.0

                    if threat and (
                        emergency
                        or (not manual_active and grace_timer[i] <= 0.0)
                    ):
                        mode[i] = "AUTO"
                        print(f"[Drone {i + 1}] AUTO: obstacle detected.")

                else:  # AUTO
                    auto_timer[i] += dt

                    if threat:
                        clear_timer[i] = 0.0
                    else:
                        clear_timer[i] += dt

                    reason = None

                    if clear_timer[i] >= RELEASE_HOLD:
                        reason = "path clear"
                    elif manual_active and not emergency:
                        reason = "pilot override"
                    elif auto_timer[i] >= AUTO_MAX_TIME and not emergency:
                        reason = "auto timeout"

                    if reason is not None:
                        mode[i] = "MANUAL"
                        target_pos[i] = pos.copy()
                        grace_timer[i] = MANUAL_GRACE
                        print(
                            f"[Drone {i + 1}] MANUAL: {reason}."
                        )

                # ---------------- DESIRED VELOCITY ----------------
                if mode[i] == "MANUAL":
                    desired_vel = manual_velocity
                    accel = MAX_ACCEL

                    # Pilot-commanded turning
                    target_yaw[i] = wrap_angle(
                        target_yaw[i] + yaw_cmd * YAW_RATE * dt
                    )

                else:
                    mag = float(np.linalg.norm(escape))

                    if threat and mag > 1e-6:
                        speed = AVOID_SPEED * min(1.0, mag)
                        if emergency:
                            speed = EMERGENCY_SPEED
                        desired_vel = escape / mag * speed
                    else:
                        desired_vel = np.zeros(3)   # brake and hover

                    accel = EMERGENCY_ACCEL if emergency else MAX_ACCEL

                cmd_vel[i] = slew_velocity(
                    cmd_vel[i], desired_vel, accel * dt
                )

                # ---------------- LEASHED SETPOINT ----------------
                tp = target_pos[i] + cmd_vel[i] * dt
                tp[0] = np.clip(tp[0], -XY_LIMIT, XY_LIMIT)
                tp[1] = np.clip(tp[1], -XY_LIMIT, XY_LIMIT)
                tp[2] = np.clip(tp[2], Z_MIN, Z_MAX)

                err = tp - pos
                err_norm = float(np.linalg.norm(err))
                if err_norm > LEASH:
                    tp = pos + err * (LEASH / err_norm)

                target_pos[i] = tp

                # ---------------- PID -> MOTOR RPM ----------------
                rpm, _, _ = controllers[i].computeControlFromState(
                    control_timestep=dt,
                    state=state,
                    target_pos=target_pos[i],
                    target_rpy=np.array([0.0, 0.0, target_yaw[i]]),
                    target_vel=cmd_vel[i]
                )

                rpms[i, :] = rpm

            env.step(rpms)

            # ------------------------------------------------
            # TRAILS
            # ------------------------------------------------
            if TRAILS and step_count % 4 == 0:
                for i in range(NUM_DRONES):
                    cur = states[i][0:3]
                    p.addUserDebugLine(
                        prev_pos[i].tolist(), cur.tolist(),
                        lineColorRGB=list(DRONE_RGBA[i][:3]),
                        lineWidth=2.0, lifeTime=8.0,
                        physicsClientId=client
                    )
                    prev_pos[i] = cur.copy()

            # ------------------------------------------------
            # CAMERAS (one tile per step, round-robin)
            #   0 overview | 1 chase D1 | 2 chase D2
            #   3 top-down | 4 front D1 | 5 front D2
            # ------------------------------------------------
            v = step_count % NUM_VIEWS

            if v == 0:
                img = camera_image(
                    client,
                    eye=[0.0, -7.5, 5.0], target=[0.0, 0.0, 1.0],
                    up=[0, 0, 1], fov=60
                )
                label_tile(img, [
                    ("OVERVIEW", (255, 255, 255)),
                    ("D1: WASD move, Q/E dn/up, Z/C turn",
                     DRONE_BGR[0]),
                    ("D2: IJKL move, U/O dn/up, N/M turn",
                     DRONE_BGR[1]),
                ], scale=0.42)

            elif v == 1 or v == 2:
                i = v - 1
                s = states[i]
                yaw = float(s[9])
                hf = np.array([np.cos(yaw), np.sin(yaw), 0.0])

                img = camera_image(
                    client,
                    eye=s[0:3] - 2.5 * hf + np.array([0, 0, 1.4]),
                    target=s[0:3] + 0.5 * hf,
                    up=[0, 0, 1], fov=75
                )
                label_tile(img, [
                    (f"CHASE DRONE {i + 1}", DRONE_BGR[i]),
                    (mode[i], mode_color(mode[i])),
                ])

            elif v == 3:
                img = camera_image(
                    client,
                    eye=[0.0, 0.0, 9.0], target=[0.0, 0.0, 0.0],
                    up=[0, 1, 0], fov=60
                )
                label_tile(img, [
                    ("TOP-DOWN", (255, 255, 255)),
                    (f"D1: {mode[0]}", mode_color(mode[0])),
                    (f"D2: {mode[1]}", mode_color(mode[1])),
                ])

            else:
                i = v - 4
                s = states[i]
                rot = np.array(
                    p.getMatrixFromQuaternion(s[3:7])
                ).reshape(3, 3)
                fwd_b = rot[:, 0]
                up_b = rot[:, 2]

                img = camera_image(
                    client,
                    eye=s[0:3] + 0.10 * fwd_b,
                    target=s[0:3] + 5.0 * fwd_b,
                    up=up_b, fov=85
                )

                gap = nearest_gap[i]
                gap_text = f"{gap:.2f} m" if np.isfinite(gap) else "N/A"

                label_tile(img, [
                    (f"FRONT DRONE {i + 1}", DRONE_BGR[i]),
                    (mode[i], mode_color(mode[i])),
                    (f"Nearest: {gap_text}", (255, 255, 255)),
                    (f"Alt: {s[2]:.2f} m", (255, 255, 255)),
                ])

            tiles[v] = img

            row1 = np.hstack([tiles[0], tiles[1], tiles[2]])
            row2 = np.hstack([tiles[3], tiles[4], tiles[5]])
            cv2.imshow("Two-Drone View", np.vstack([row1, row2]))

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