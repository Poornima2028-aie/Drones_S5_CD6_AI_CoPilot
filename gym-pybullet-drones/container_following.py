import os
import time
import math
import numpy as np
import cv2
import pybullet as p

from ultralytics import YOLO

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl


# ============================================================
# STAGE 25 (REBUILT)
# MANUAL FLIGHT -> AUTONOMOUS CIRCULAR WALL FOLLOWING
# INSIDE A STANDING (VERTICAL) CYLINDER
#
# BEHAVIOUR
# ------------------------------------------------------------
# 1. The cylinder stands upright. Its side wall is fully closed
#    all the way around. The TOP is open (sky). The bottom is
#    the ground.
#
# 2. The drone starts at the centre. YOU fly it manually with
#    the keyboard (fast).
#
# 3. When the drone gets close to the wall, it:
#       a) STOPS (hovers) for ~1 second
#       b) automatically figures out which way to turn
#       c) switches to AUTONOMOUS mode and starts circling
#          along the inside of the wall, continuously, at
#          high speed, yawing to face its direction of travel.
#
# 4. Manual control is disabled once autonomous mode starts.
#
# KEYBOARD (focus the OpenCV camera window):
#   w / s   -> move +X / -X
#   a / d   -> move -Y / +Y
#   r / f   -> move up / down
#   x       -> stop / hover
#   q       -> quit
# ============================================================


print()
print("======================================================")
print("STAGE 25 - MANUAL FLIGHT -> AUTONOMOUS WALL FOLLOWING")
print("======================================================")
print()


# ============================================================
# SETTINGS
# ============================================================

DRONE_MODEL = DroneModel("cf2x")

INITIAL_POSITION = np.array([0.0, 0.0, 1.5], dtype=float)
INITIAL_RPY = np.array([0.0, 0.0, 0.0], dtype=float)

SIMULATION_FREQ = 240
CONTROL_FREQ = 48


# ============================================================
# CYLINDER (STANDING / VERTICAL)
# ============================================================

CYLINDER_RADIUS = 6.0          # radius of the arena
CYLINDER_HEIGHT = 6.0          # wall height (top is open - "sky")
CYLINDER_WALL_THICKNESS = 0.20
CYLINDER_SEGMENTS = 90         # more segments = smoother wall


# ============================================================
# SPEED (increased significantly vs. the original script)
# ============================================================

MANUAL_SPEED = 4.0             # m/s, manual flight speed
CRUISE_SPEED = 4.0             # m/s, autonomous wall-follow speed
MAX_RADIAL_CORRECTION = 2.5    # m/s, max inward/outward correction


# ============================================================
# WALL FOLLOWING / DETECTION
# ============================================================

DESIRED_WALL_DISTANCE = 1.50   # how far from the wall to cruise
WALL_DETECTION_DISTANCE = 1.80 # distance that triggers auto mode
EMERGENCY_WALL_DISTANCE = 0.60 # hard override distance

STOP_DURATION = 1.0            # seconds to hover before circling

RADIAL_KP = 1.10
RADIAL_KD = 0.25


# ============================================================
# SENSOR (ray casting, mainly used for the on-screen readout
# and emergency checks - the exact wall distance is also
# computed geometrically since the cylinder is a perfect circle)
# ============================================================

SENSOR_RAY_LENGTH = 4.0


# ============================================================
# ALTITUDE
# ============================================================

TARGET_ALTITUDE = 1.50
MIN_ALTITUDE = 0.80
MAX_ALTITUDE = CYLINDER_HEIGHT - 0.5


# ============================================================
# CAMERA
# ============================================================

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV = 75
CAMERA_NEAR = 0.02
CAMERA_FAR = 15.0
CAMERA_INTERVAL = 4


# ============================================================
# YOLO
# ============================================================

YOLO_ENABLED = True
YOLO_MODEL_NAME = "yolo11n.pt"
YOLO_CONFIDENCE = 0.35


# ============================================================
# TEST
# ============================================================

MAX_DURATION = 180

RESULT_DIR = "results/stage25_cylindrical_wall_following"
os.makedirs(RESULT_DIR, exist_ok=True)


# ============================================================
# ENVIRONMENT
# ============================================================

print("Creating PyBullet environment...")

env = CtrlAviary(
    drone_model=DRONE_MODEL,
    num_drones=1,
    initial_xyzs=np.array([INITIAL_POSITION]),
    initial_rpys=np.array([INITIAL_RPY]),
    physics=Physics.PYB,
    pyb_freq=SIMULATION_FREQ,
    ctrl_freq=CONTROL_FREQ,
    gui=True,
    record=False,
    obstacles=False,
    user_debug_gui=False,   # turns off the per-drone axis gizmo that was blocking the view
)

physics_client = env.getPyBulletClient()

controller = DSLPIDControl(drone_model=DRONE_MODEL)


# ============================================================
# GROUND
# ============================================================

print("Creating ground...")

ground_collision = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[20.0, 20.0, 0.05], physicsClientId=physics_client
)
ground_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[20.0, 20.0, 0.05],
    rgbaColor=[0.25, 0.25, 0.25, 1.0],
    physicsClientId=physics_client,
)
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=ground_collision,
    baseVisualShapeIndex=ground_visual,
    basePosition=[0.0, 0.0, -0.05],
    physicsClientId=physics_client,
)


# ============================================================
# STANDING CYLINDRICAL WALL (closed on all sides, open on top)
#
# Built out of thin vertical box segments arranged in a circle,
# each spanning the full height of the arena. A single yaw
# rotation (theta) is enough to orient each segment - no pitch
# tricks needed because the cylinder's axis is already Z (up).
# ============================================================

print("Creating standing cylindrical wall...")

wall_ids = []
angular_step = 2.0 * math.pi / CYLINDER_SEGMENTS
half_tangent = math.pi * CYLINDER_RADIUS / CYLINDER_SEGMENTS
half_radial = CYLINDER_WALL_THICKNESS / 2.0
half_height = CYLINDER_HEIGHT / 2.0

for i in range(CYLINDER_SEGMENTS):
    theta = i * angular_step

    x_center = CYLINDER_RADIUS * math.cos(theta)
    y_center = CYLINDER_RADIUS * math.sin(theta)
    z_center = half_height

    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[half_radial, half_tangent, half_height],
        physicsClientId=physics_client,
    )
    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[half_radial, half_tangent, half_height],
        rgbaColor=[0.35, 0.40, 0.45, 0.35],
        physicsClientId=physics_client,
    )

    orientation = p.getQuaternionFromEuler([0.0, 0.0, theta])

    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x_center, y_center, z_center],
        baseOrientation=orientation,
        physicsClientId=physics_client,
    )
    wall_ids.append(body_id)

print(f"Cylindrical wall segments created: {len(wall_ids)}")
print(f"Cylinder radius: {CYLINDER_RADIUS:.2f} m")
print(f"Cylinder height: {CYLINDER_HEIGHT:.2f} m (top open)")
print()


# ============================================================
# SENSOR RAYS (forward/left/right relative to current heading,
# plus up/down) - used for the HUD and as a secondary check
# ============================================================

def cast_sensor_rays(position, heading):
    """Cast rays relative to the current heading direction."""
    origin = np.array(position, dtype=float)

    heading = np.array(heading, dtype=float)
    norm = np.linalg.norm(heading)
    if norm < 1e-6:
        heading = np.array([1.0, 0.0, 0.0])
    else:
        heading = heading / norm

    angles_deg = [0, -60, -30, 30, 60]
    directions = []
    for a in angles_deg:
        rad = math.radians(a)
        rot = np.array(
            [
                heading[0] * math.cos(rad) - heading[1] * math.sin(rad),
                heading[0] * math.sin(rad) + heading[1] * math.cos(rad),
                0.0,
            ]
        )
        directions.append(rot)

    directions.append(np.array([0.0, 0.0, 1.0]))   # up
    directions.append(np.array([0.0, 0.0, -1.0]))  # down

    ray_origins = []
    ray_ends = []
    for d in directions:
        d = d / (np.linalg.norm(d) + 1e-9)
        ray_origins.append(origin.tolist())
        ray_ends.append((origin + d * SENSOR_RAY_LENGTH).tolist())

    results = p.rayTestBatch(ray_origins, ray_ends, physicsClientId=physics_client)

    distances = []
    for result in results:
        hit_fraction = result[2]
        if hit_fraction >= 1.0:
            distances.append(SENSOR_RAY_LENGTH)
        else:
            distances.append(max(0.01, hit_fraction * SENSOR_RAY_LENGTH))

    # index: 0 forward, 1 -60, 2 -30, 3 +30, 4 +60, 5 up, 6 down
    return distances


# ============================================================
# GEOMETRIC WALL DISTANCE (exact, since the arena is a circle)
# ============================================================

def wall_distance_and_direction(position):
    x, y = position[0], position[1]
    radial_distance = math.sqrt(x * x + y * y)
    distance_to_wall = CYLINDER_RADIUS - radial_distance

    if radial_distance > 1e-6:
        outward = np.array([x / radial_distance, y / radial_distance, 0.0])
    else:
        outward = np.array([1.0, 0.0, 0.0])

    theta = math.atan2(y, x)
    return distance_to_wall, outward, theta


# ============================================================
# CAMERA
# ============================================================

def capture_camera(position, heading):
    heading = np.array(heading, dtype=float)
    norm = np.linalg.norm(heading)
    if norm < 1e-6:
        heading = np.array([1.0, 0.0, 0.0])
    else:
        heading = heading / norm

    camera_position = np.array(position, dtype=float)
    camera_target = camera_position + heading * 6.0
    camera_target[2] = camera_position[2]

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position.tolist(),
        cameraTargetPosition=camera_target.tolist(),
        cameraUpVector=[0.0, 0.0, 1.0],
        physicsClientId=physics_client,
    )
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=CAMERA_FOV,
        aspect=float(CAMERA_WIDTH) / float(CAMERA_HEIGHT),
        nearVal=CAMERA_NEAR,
        farVal=CAMERA_FAR,
    )

    image = p.getCameraImage(
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=physics_client,
    )

    raw_rgb = image[2]
    rgba = np.asarray(raw_rgb, dtype=np.uint8)

    expected_rgba = CAMERA_WIDTH * CAMERA_HEIGHT * 4
    expected_rgb = CAMERA_WIDTH * CAMERA_HEIGHT * 3

    if rgba.ndim == 1:
        if rgba.size == expected_rgba:
            rgba = rgba.reshape(CAMERA_HEIGHT, CAMERA_WIDTH, 4)
        elif rgba.size == expected_rgb:
            rgba = rgba.reshape(CAMERA_HEIGHT, CAMERA_WIDTH, 3)
        else:
            raise RuntimeError("Unexpected camera buffer")

    rgb = rgba[:, :, :3]
    return np.ascontiguousarray(rgb, dtype=np.uint8)


# ============================================================
# YOLO
# ============================================================

model = None
if YOLO_ENABLED:
    print("Loading YOLO model...")
    try:
        model = YOLO(YOLO_MODEL_NAME)
        print("YOLO model loaded.")
    except Exception as exc:
        print("[WARNING] YOLO could not be loaded:", exc)
        YOLO_ENABLED = False
print()


# ============================================================
# STATE MACHINE
# ============================================================

MODE_MANUAL = "MANUAL"
MODE_STOPPING = "STOPPING"
MODE_AUTONOMOUS = "AUTONOMOUS_WALL_FOLLOWING"

mode = MODE_MANUAL

manual_command = np.array([0.0, 0.0, 0.0])  # world-frame unit-ish direction
heading = np.array([1.0, 0.0, 0.0])         # used for camera + yaw

chase_cam_enabled = True   # toggle with 'c' - keeps the drone centred in the GUI

stop_mode_start_time = None
follow_direction_sign = 1.0   # +1 -> counter-clockwise, -1 -> clockwise

previous_radial_error = 0.0
previous_time = time.time()

emergency_avoidances = 0
camera_frames = 0
yolo_detections = 0
last_display_frame = None
last_yolo_detection = False

target_position = INITIAL_POSITION.copy()
target_yaw = 0.0


# ============================================================
# KEY HANDLING (manual mode)
# ============================================================

KEY_TO_DIRECTION = {
    ord("w"): np.array([1.0, 0.0, 0.0]),
    ord("s"): np.array([-1.0, 0.0, 0.0]),
    ord("a"): np.array([0.0, -1.0, 0.0]),
    ord("d"): np.array([0.0, 1.0, 0.0]),
    ord("r"): np.array([0.0, 0.0, 1.0]),
    ord("f"): np.array([0.0, 0.0, -1.0]),
}


def handle_key(key_code):
    global manual_command, heading

    if key_code in KEY_TO_DIRECTION:
        manual_command = KEY_TO_DIRECTION[key_code].copy()
        horizontal = manual_command.copy()
        horizontal[2] = 0.0
        if np.linalg.norm(horizontal) > 1e-6:
            heading = horizontal / np.linalg.norm(horizontal)
    elif key_code == ord("x"):
        manual_command = np.array([0.0, 0.0, 0.0])


# ============================================================
# START MESSAGE
# ============================================================

print("======================================================")
print("MANUAL FLIGHT -> AUTONOMOUS WALL FOLLOWING")
print("======================================================")
print()
print("Controls (click the camera window first):")
print("  w/s = forward/back   a/d = left/right   r/f = up/down")
print("  x   = stop / hover   q   = quit   c = toggle chase-cam")
print()
print("The PyBullet GUI window (not the OpenCV camera window) is where")
print("the chase-cam applies. Left-drag to orbit, scroll to zoom - the")
print("camera will keep re-centring on the drone every frame while")
print("chase-cam is ON, and stay wherever you leave it once it's OFF.")
print()
print(f"Manual speed:   {MANUAL_SPEED:.2f} m/s")
print(f"Cruise speed:   {CRUISE_SPEED:.2f} m/s (autonomous)")
print(f"Wall trigger:   {WALL_DETECTION_DISTANCE:.2f} m")
print()
print("Starting Stage 25...")
print()


# ============================================================
# MAIN LOOP
# ============================================================

try:
    total_steps = int(MAX_DURATION * CONTROL_FREQ)
    start_time = time.time()
    last_print_time = time.time()

    for step_count in range(total_steps):

        state = env._getDroneStateVector(0)
        current_position = np.array(state[0:3], dtype=float)
        current_quaternion = np.array(state[3:7], dtype=float)
        current_velocity = np.array(state[10:13], dtype=float)
        current_angular_velocity = np.array(state[13:16], dtype=float)

        now = time.time()
        dt = now - previous_time
        previous_time = now
        if dt <= 0:
            dt = 1.0 / CONTROL_FREQ

        distance_to_wall, outward_dir, theta = wall_distance_and_direction(
            current_position
        )

        # ----------------------------------------------------
        # MANUAL MODE
        # ----------------------------------------------------
        if mode == MODE_MANUAL:

            target_position = current_position + manual_command * MANUAL_SPEED * dt
            target_position[2] = np.clip(target_position[2], MIN_ALTITUDE, MAX_ALTITUDE)

            if np.linalg.norm(manual_command[:2]) > 1e-6:
                target_yaw = math.atan2(manual_command[1], manual_command[0])

            if distance_to_wall <= WALL_DETECTION_DISTANCE:
                mode = MODE_STOPPING
                stop_mode_start_time = time.time()

                # Decide which way to circle: continue in whichever
                # tangential direction best matches current velocity,
                # so the transition is smooth rather than a snap-turn.
                tangent_ccw = np.array([-math.sin(theta), math.cos(theta), 0.0])
                dot = np.dot(current_velocity[:2], tangent_ccw[:2])
                follow_direction_sign = 1.0 if dot >= 0.0 else -1.0

                manual_command = np.array([0.0, 0.0, 0.0])
                target_position = current_position.copy()

                print()
                print(f"[AUTO] Wall detected at {distance_to_wall:.2f} m -> stopping.")

        # ----------------------------------------------------
        # STOPPING MODE (brief hover before circling)
        # ----------------------------------------------------
        elif mode == MODE_STOPPING:

            target_position = current_position.copy()
            target_position[2] = TARGET_ALTITUDE

            if time.time() - stop_mode_start_time >= STOP_DURATION:
                mode = MODE_AUTONOMOUS
                previous_radial_error = 0.0
                print(f"[AUTO] Hover complete -> autonomous wall following engaged.")
                print()

        # ----------------------------------------------------
        # AUTONOMOUS WALL FOLLOWING (circling the inside wall)
        # ----------------------------------------------------
        elif mode == MODE_AUTONOMOUS:

            tangent = np.array(
                [
                    -math.sin(theta) * follow_direction_sign,
                    math.cos(theta) * follow_direction_sign,
                    0.0,
                ]
            )
            heading = tangent.copy()
            target_yaw = math.atan2(tangent[1], tangent[0])

            radial_error = DESIRED_WALL_DISTANCE - distance_to_wall
            radial_error_rate = (
                (radial_error - previous_radial_error) / dt if dt > 1e-5 else 0.0
            )
            previous_radial_error = radial_error

            radial_correction = RADIAL_KP * radial_error + RADIAL_KD * radial_error_rate
            radial_correction = np.clip(
                radial_correction, -MAX_RADIAL_CORRECTION, MAX_RADIAL_CORRECTION
            )
            # positive radial_error (too close) -> push inward (opposite of outward_dir)
            radial_vector = -outward_dir * radial_correction

            forward_speed = CRUISE_SPEED
            if distance_to_wall < EMERGENCY_WALL_DISTANCE:
                emergency_avoidances += 1
                forward_speed *= 0.5
                radial_vector = -outward_dir * MAX_RADIAL_CORRECTION

            target_position = current_position.copy()
            target_position += tangent * forward_speed * dt * 2.5
            target_position += radial_vector * dt * 2.5
            target_position[2] = TARGET_ALTITUDE
            target_position[2] = np.clip(target_position[2], MIN_ALTITUDE, MAX_ALTITUDE)

        # ----------------------------------------------------
        # SENSOR (for HUD only)
        # ----------------------------------------------------
        sensor_distances = cast_sensor_rays(current_position, heading)

        # ----------------------------------------------------
        # CAMERA + YOLO
        # ----------------------------------------------------
        if step_count % CAMERA_INTERVAL == 0:
            try:
                frame = capture_camera(current_position, heading)
                camera_frames += 1
                display_frame = frame.copy()

                if YOLO_ENABLED and model is not None:
                    results = model.predict(source=frame, conf=YOLO_CONFIDENCE, verbose=False)
                    result = results[0]
                    if len(result.boxes) > 0:
                        last_yolo_detection = True
                        yolo_detections += len(result.boxes)
                        display_frame = result.plot()
                    else:
                        last_yolo_detection = False

                image_path = os.path.join(RESULT_DIR, f"frame_{camera_frames:04d}.jpg")
                cv2.imwrite(image_path, display_frame)
                last_display_frame = display_frame.copy()

            except Exception as exc:
                print("[CAMERA WARNING]", exc)

        # ----------------------------------------------------
        # DISPLAY / HUD
        # ----------------------------------------------------
        if last_display_frame is not None:
            display = last_display_frame.copy()

            mode_color = (0, 255, 0) if mode != MODE_STOPPING else (0, 165, 255)
            cv2.putText(display, f"MODE: {mode}", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, mode_color, 2)

            cv2.putText(display, f"Wall distance: {distance_to_wall:.2f} m", (15, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            cv2.putText(display, f"Forward sensor: {sensor_distances[0]:.2f} m", (15, 88),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            speed_label = MANUAL_SPEED if mode == MODE_MANUAL else CRUISE_SPEED
            cv2.putText(display, f"Speed: {speed_label:.2f} m/s", (15, 116),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            if mode == MODE_MANUAL:
                cv2.putText(display, "Controls: w/s a/d r/f  x=stop  q=quit", (15, 144),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 0), 2)
            elif mode == MODE_AUTONOMOUS:
                direction_label = "CCW" if follow_direction_sign > 0 else "CW"
                cv2.putText(display, f"Circling: {direction_label}", (15, 144),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

            if distance_to_wall < EMERGENCY_WALL_DISTANCE:
                cv2.putText(display, "EMERGENCY WALL AVOIDANCE", (15, 175),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 0, 255), 2)

            cv2.putText(
                display,
                "YOLO: OBJECT DETECTED" if last_yolo_detection else "YOLO: CLEAR",
                (15, 205),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255) if last_yolo_detection else (0, 255, 0),
                2,
            )

            cv2.imshow("STAGE 25 - MANUAL -> AUTONOMOUS WALL FOLLOWING", display)

        # ----------------------------------------------------
        # PID CONTROL (yaw follows direction of travel)
        # ----------------------------------------------------
        action, pos_error, yaw_error = controller.computeControl(
            control_timestep=env.CTRL_TIMESTEP,
            cur_pos=current_position,
            cur_quat=current_quaternion,
            cur_vel=current_velocity,
            cur_ang_vel=current_angular_velocity,
            target_pos=target_position,
            target_rpy=np.array([0.0, 0.0, target_yaw]),
        )

        action = np.asarray(action, dtype=float)
        if action.ndim == 1:
            action = action.reshape(1, 4)

        env.step(action)

        # ----------------------------------------------------
        # CHASE CAMERA
        # Keeps the drone centred in the PyBullet GUI window
        # regardless of how far/fast it flies. Preserves whatever
        # yaw/pitch/zoom you last set with the mouse - it only
        # re-targets the camera onto the drone every frame.
        # ----------------------------------------------------
        if chase_cam_enabled:
            try:
                cam_info = p.getDebugVisualizerCamera(physicsClientId=physics_client)
                cam_yaw = cam_info[8]
                cam_pitch = cam_info[9]
                cam_distance = cam_info[10]
                p.resetDebugVisualizerCamera(
                    cameraDistance=cam_distance,
                    cameraYaw=cam_yaw,
                    cameraPitch=cam_pitch,
                    cameraTargetPosition=current_position.tolist(),
                    physicsClientId=physics_client,
                )
            except Exception:
                pass

        # ----------------------------------------------------
        # KEYS
        # ----------------------------------------------------
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print()
            print("Q pressed.")
            break
        elif key == ord("c"):
            chase_cam_enabled = not chase_cam_enabled
            print(f"[CAMERA] Chase cam {'ON' if chase_cam_enabled else 'OFF'}")
        elif mode == MODE_MANUAL:
            handle_key(key)

        # ----------------------------------------------------
        # STATUS
        # ----------------------------------------------------
        current_time = time.time()
        if current_time - last_print_time >= 1.0:
            print(
                f"Mode: {mode:26s} | "
                f"Pos: [{current_position[0]:.2f}, {current_position[1]:.2f}, "
                f"{current_position[2]:.2f}] | "
                f"Wall dist: {distance_to_wall:.2f} m"
            )
            last_print_time = current_time

        if time.time() - start_time >= MAX_DURATION:
            print()
            print("Maximum test duration reached.")
            break

finally:
    print()
    print("Shutting down Stage 25...")

    try:
        cv2.destroyAllWindows()
    except Exception:
        pass

    try:
        final_state = env._getDroneStateVector(0)
        final_position = np.array(final_state[0:3], dtype=float)
    except Exception:
        final_position = np.array([np.nan, np.nan, np.nan])

    try:
        env.close()
    except Exception:
        pass


# ============================================================
# FINAL RESULTS
# ============================================================

final_distance_from_start = np.linalg.norm(final_position - INITIAL_POSITION)

print()
print("======================================================")
print("STAGE 25 COMPLETED")
print("======================================================")
print()
print("Initial position:", INITIAL_POSITION)
print("Final position:  ", final_position)
print(f"Distance travelled: {final_distance_from_start:.3f} m")
print()
print("Emergency wall avoidances:", emergency_avoidances)
print("Live camera frames:", camera_frames)
print("YOLO detections:", yolo_detections)
print()
print("Results directory:", RESULT_DIR)
print()
print("======================================================")
print("END OF STAGE 25")
print("======================================================")