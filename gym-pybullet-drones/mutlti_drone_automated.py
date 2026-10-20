#!/usr/bin/env python3
"""
Multi-Drone Autonomous Flight + Static AND Dynamic Obstacle Avoidance
=====================================================================

Two CF2X drones fly waypoint missions completely on their own.
Each drone avoids:
    - the static pillar and box
    - three moving spheres (patrol / orbit / bob)
    - the OTHER drone (predicted closest-approach, passing on the right)

VIEW (one window, 2 x 3 grid):
    Overview   | Chase DRONE 1 | Chase DRONE 2
    Top-down   | Front DRONE 1 | Front DRONE 2

    Drone 1 = blue, Drone 2 = pink. Coloured trails show each path.
    The PyBullet 3D window is also moved to a wide overview angle.

CONTROLS:
    ESC = Exit   (focus the OpenCV window)
"""

import time

import cv2
import numpy as np
import pybullet as p

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

# Start positions (x, y, z) and waypoint missions (ping-pong loops).
# Different cruise altitudes (1.0 m and 1.5 m) add extra separation.
STARTS = np.array([
    [-3.5, -0.8, 1.0],
    [3.5, -3.0, 1.5],
])

MISSIONS = [
    [np.array([3.5, -0.8, 1.0]), np.array([-3.5, -0.8, 1.0])],
    [np.array([-3.5, 3.0, 1.5]), np.array([3.5, -3.0, 1.5])],
]

# BGR colours for OpenCV text, RGBA for PyBullet
DRONE_RGBA = [(0.1, 0.45, 1.0, 1.0), (1.0, 0.2, 0.65, 1.0)]
DRONE_BGR = [(255, 115, 25), (165, 50, 255)]

# Arena limits
Z_MIN = 0.35
Z_MAX = 2.5
XY_LIMIT = 4.5

# ---- Motion limits (kept gentle for stability) ----
CRUISE_SPEED = 0.50
AVOID_SPEED = 0.60
EMERGENCY_SPEED = 0.90
MAX_ACCEL = 1.0
EMERGENCY_ACCEL = 2.2
LEASH = 0.30
YAW_RATE = 0.8            # rad/s, how fast the nose turns toward travel
WARMUP_TIME = 1.0         # seconds of hover before missions start

# ---- Waypoints ----
ARRIVE_RADIUS = 0.6
WAYPOINT_TIMEOUT = 12.0   # skip a waypoint that cannot be reached

# ---- Avoidance (surface-to-surface gaps), metres ----
DRONE_RADIUS = 0.10
OTHER_DRONE_RADIUS = 0.12
AVOID_DISTANCE = 1.5
EMERGENCY_DISTANCE = 0.65
PREDICT_RANGE = 2.5
PREDICT_HORIZON = 2.0

# ---- Cameras ----
TILE_W = 320
TILE_H = 240
TRAILS = True

# ---- Static obstacles ----
PILLAR = (2.0, 0.0, 0.25, 2.5)          # x, y, radius, height
BOX_POS = np.array([3.0, 1.5, 1.0])
BOX_HALF = 0.30


# ============================================================
# OBSTACLES (static + dynamic)
# ============================================================
# dict keys: id, kind ("cylinder" | "sphere"), radius, pos, vel, motion

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
# THREAT ASSESSMENT
# ============================================================

def assess_obstacles(pos, vel, goal_dir, obstacles, side_memory):
    """
    Predict the closest approach to every obstacle (static, moving,
    or another drone) and build an avoidance vector.

    The drone slips past each obstacle on a COMMITTED side (stored in
    side_memory) so it does not dither left/right. The side is chosen
    relative to the drone's goal direction, so two drones meeting head
    on both pass on their own right (like traffic).

    Returns: escape_vec, nearest_gap, threat, max_strength
    """
    total = np.zeros(3)
    nearest = float("inf")
    max_strength = 0.0
    active = set()

    # Horizontal unit vector pointing to the LEFT of the goal direction
    left_of_goal = np.array([-goal_dir[1], goal_dir[0], 0.0])

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

        # ---- Closest point of approach ----
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

        # ---- Push directly away from the predicted position ----
        away = -pred if np.linalg.norm(pred) > 1e-3 else -rel

        if np.linalg.norm(away) < 1e-6:
            away = -goal_dir.copy()

        away = away / np.linalg.norm(away)
        away[2] *= 0.5                      # gentle vertical escape

        # ---- Committed sideways slip ----
        if key not in side_memory:
            offset = float(rel @ left_of_goal)

            if offset > 0.05:
                lateral = -left_of_goal     # obstacle on left -> go right
            elif offset < -0.05:
                lateral = left_of_goal      # obstacle on right -> go left
            else:
                lateral = -left_of_goal     # dead ahead -> pass on right

            side_memory[key] = lateral

        direction = away + 0.9 * side_memory[key]
        direction = direction / np.linalg.norm(direction)

        total += strength * direction

    # Forget the side choice for obstacles that are no longer a threat.
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


def label_tile(image, lines, color):
    for i, line in enumerate(lines):
        cv2.putText(
            image, line, (8, 20 + i * 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA
        )


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

    try:
        env.reset()
        client = env.CLIENT
        dt = env.CTRL_TIMESTEP

        p.setRealTimeSimulation(0, physicsClientId=client)
        p.configureDebugVisualizer(
            p.COV_ENABLE_GUI, 0, physicsClientId=client
        )

        # Wide overview angle for the 3D window.
        p.resetDebugVisualizerCamera(
            cameraDistance=9.0,
            cameraYaw=0,
            cameraPitch=-45,
            cameraTargetPosition=[0, 0, 1],
            physicsClientId=client
        )

        # Colour the drones so they can be told apart.
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

        # ---- Per-drone state ----
        controllers = [
            DSLPIDControl(drone_model=DroneModel.CF2X)
            for _ in range(NUM_DRONES)
        ]

        target_pos = [STARTS[i].copy() for i in range(NUM_DRONES)]
        cmd_vel = [np.zeros(3) for _ in range(NUM_DRONES)]
        target_yaw = [0.0] * NUM_DRONES
        wp_index = [0] * NUM_DRONES
        wp_timer = [0.0] * NUM_DRONES
        side_memory = [dict() for _ in range(NUM_DRONES)]
        status = ["CRUISE"] * NUM_DRONES
        nearest_gap = [float("inf")] * NUM_DRONES
        prev_pos = [STARTS[i].copy() for i in range(NUM_DRONES)]

        # ---- Camera tiles (rendered round-robin, 1 per step) ----
        tiles = [
            np.zeros((TILE_H, TILE_W, 3), dtype=np.uint8)
            for _ in range(6)
        ]
        NUM_VIEWS = 6

        start_time = time.time()
        step_count = 0

        print("\n========================================")
        print(" MULTI-DRONE AUTONOMOUS AVOIDANCE")
        print("========================================")
        print("Drone 1 = blue, Drone 2 = pink.")
        print("Press ESC in the camera window to quit.\n")

        while True:
            sim_time = step_count * dt

            update_dynamic_obstacles(client, obstacles, sim_time)

            states = [
                env._getDroneStateVector(i) for i in range(NUM_DRONES)
            ]

            rpms = np.zeros((NUM_DRONES, 4))

            for i in range(NUM_DRONES):
                state = states[i]
                pos = state[0:3].copy()
                vel = state[10:13].copy()

                # ---- Other drones are treated as moving spheres ----
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

                all_obstacles = obstacles + others

                # ---- Waypoint following ----
                waypoint = MISSIONS[i][wp_index[i]]
                to_goal = waypoint - pos
                goal_dist = float(np.linalg.norm(to_goal))

                wp_timer[i] += dt

                if goal_dist < ARRIVE_RADIUS or (
                    wp_timer[i] > WAYPOINT_TIMEOUT
                ):
                    wp_index[i] = (wp_index[i] + 1) % len(MISSIONS[i])
                    wp_timer[i] = 0.0
                    waypoint = MISSIONS[i][wp_index[i]]
                    to_goal = waypoint - pos
                    goal_dist = float(np.linalg.norm(to_goal))

                if goal_dist > 1e-6:
                    goal_unit = to_goal / goal_dist
                else:
                    goal_unit = np.array([1.0, 0.0, 0.0])

                goal_dir_xy = goal_unit.copy()
                goal_dir_xy[2] = 0.0
                n_xy = np.linalg.norm(goal_dir_xy)
                if n_xy < 1e-6:
                    goal_dir_xy = np.array([1.0, 0.0, 0.0])
                else:
                    goal_dir_xy /= n_xy

                if sim_time < WARMUP_TIME:
                    goal_vel = np.zeros(3)
                else:
                    speed = min(CRUISE_SPEED, 0.8 * goal_dist)
                    goal_vel = goal_unit * speed

                # ---- Avoidance ----
                escape, nearest, threat, strength = assess_obstacles(
                    pos, vel, goal_dir_xy, all_obstacles,
                    side_memory[i]
                )

                nearest_gap[i] = nearest
                emergency = nearest < EMERGENCY_DISTANCE

                if threat:
                    desired = (
                        goal_vel * (1.0 - 0.7 * strength)
                        + escape * AVOID_SPEED
                    )

                    cap = EMERGENCY_SPEED if emergency else AVOID_SPEED
                    n = float(np.linalg.norm(desired))
                    if n > cap:
                        desired *= cap / n

                    accel = EMERGENCY_ACCEL if emergency else MAX_ACCEL
                    new_status = "AVOID"
                else:
                    desired = goal_vel
                    accel = MAX_ACCEL
                    new_status = "CRUISE"

                if new_status != status[i]:
                    print(f"[Drone {i + 1}] {new_status}")
                    status[i] = new_status

                # ---- Smooth velocity + leashed setpoint ----
                cmd_vel[i] = slew_velocity(cmd_vel[i], desired, accel * dt)

                tp = target_pos[i] + cmd_vel[i] * dt
                tp[0] = np.clip(tp[0], -XY_LIMIT, XY_LIMIT)
                tp[1] = np.clip(tp[1], -XY_LIMIT, XY_LIMIT)
                tp[2] = np.clip(tp[2], Z_MIN, Z_MAX)

                err = tp - pos
                err_norm = float(np.linalg.norm(err))
                if err_norm > LEASH:
                    tp = pos + err * (LEASH / err_norm)

                target_pos[i] = tp

                # ---- Turn the nose toward the travel direction ----
                if np.hypot(cmd_vel[i][0], cmd_vel[i][1]) > 0.15:
                    want = np.arctan2(cmd_vel[i][1], cmd_vel[i][0])
                    diff = wrap_angle(want - target_yaw[i])
                    step = np.clip(diff, -YAW_RATE * dt, YAW_RATE * dt)
                    target_yaw[i] = wrap_angle(target_yaw[i] + step)

                # ---- PID -> motor RPM ----
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
            # TRAILS (coloured path lines)
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
            # ------------------------------------------------
            v = step_count % NUM_VIEWS

            if v == 0:
                # Wide overview: sees the whole arena and both drones.
                img = camera_image(
                    client,
                    eye=[0.0, -7.5, 5.0], target=[0.0, 0.0, 1.0],
                    up=[0, 0, 1], fov=60
                )
                label_tile(img, ["OVERVIEW"], (255, 255, 255))

            elif v == 1 or v == 2:
                # Chase camera behind drone (v-1)
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
                label_tile(
                    img,
                    [f"CHASE DRONE {i + 1}", status[i]],
                    DRONE_BGR[i]
                )

            elif v == 3:
                # Top-down: clearest picture of the paths.
                img = camera_image(
                    client,
                    eye=[0.0, 0.0, 9.0], target=[0.0, 0.0, 0.0],
                    up=[0, 1, 0], fov=60
                )
                label_tile(img, ["TOP-DOWN"], (255, 255, 255))

            else:
                # Front camera of drone (v-4)
                i = v - 4
                s = states[i]
                rot = np.array(
                    p.getMatrixFromQuaternion(s[3:7])
                ).reshape(3, 3)
                fwd = rot[:, 0]
                up = rot[:, 2]

                img = camera_image(
                    client,
                    eye=s[0:3] + 0.10 * fwd,
                    target=s[0:3] + 5.0 * fwd,
                    up=up, fov=85
                )

                gap = nearest_gap[i]
                gap_text = f"{gap:.2f} m" if np.isfinite(gap) else "N/A"

                label_tile(
                    img,
                    [
                        f"FRONT DRONE {i + 1}",
                        status[i],
                        f"Nearest: {gap_text}",
                        f"Alt: {s[2]:.2f} m",
                    ],
                    DRONE_BGR[i]
                )

            # tile slots: 0 overview, 1 chase1, 2 chase2,
            #             3 top-down, 4 front1, 5 front2
            tiles[v] = img

            row1 = np.hstack([tiles[0], tiles[1], tiles[2]])
            row2 = np.hstack([tiles[3], tiles[4], tiles[5]])
            cv2.imshow("Multi-Drone View", np.vstack([row1, row2]))

            if (cv2.waitKey(1) & 0xFF) == 27:
                break

            step_count += 1
            sync(step_count, start_time, dt)

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    finally:
        cv2.destroyAllWindows()
        env.close()
        print("Drone simulation closed.")


if __name__ == "__main__":
    main()