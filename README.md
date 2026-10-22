<p align="center">
  <img src="amrita_logo.jpeg" alt="Amrita Vishwa Vidyapeetham Logo" width="580"/>
</p>

# CD6_AI-drone-copilot

## 👥 Team Members

<table align="center">
  <tr>
    <th>S.No.</th>
    <th>Name</th>
    <th>Roll Number</th>
    <th>Gmail</th>
  </tr>
  <tr>
    <td align="center">1</td>
    <td>Iniya</td>
    <td align="center">CB.SC.U4AIE24219</td>
    <td>cb.sc.u4aie24219@cb.students.amrita.edu</td>
  </tr>
  <tr>
    <td align="center">2</td>
    <td>Sowmya A</td>
    <td align="center">CB.SC.U4AIE24357</td>
    <td>cb.sc.u4aie24357@cb.students.amrita.edu</td>
  </tr>
  <tr>
    <td align="center">3</td>
    <td>Likitha Reddy</td>
    <td align="center">CB.SC.U4AIE24361</td>
    <td>cb.sc.u4aie24361@cb.students.amrita.edu</td>
  </tr>
  <tr>
    <td align="center">4</td>
    <td>Poornima P</td>
    <td align="center">CB.SC.U4AIE24343</td>
    <td>cb.sc.u4aie24343@cb.students.amrita.edu</td>
  </tr>
</table>

## Data-Driven Optimal Shared Control of Unmanned Aerial Vehicles

A **Human-in-the-Loop (HiTL) AI Co-Pilot** system for UAVs that learns drone dynamics from human maneuver data, approximates an optimal controller online using reinforcement learning, and smoothly blends human pilot control with autonomous optimal control.

**Base Paper:**

> Junkai Tan, Shuangsi Xue, Zihang Guo, Huan Li, Hui Cao, Badong Chen.
> "Data-driven optimal shared control of unmanned aerial vehicles."
> *Neurocomputing*, Volume 622, 129428, 2025.
> DOI: [10.1016/j.neucom.2025.129428](https://doi.org/10.1016/j.neucom.2025.129428)

**Affiliation:** Xi'an Jiaotong University

---

## 📌 Introduction

Unmanned Aerial Vehicles (UAVs) are increasingly used in environments where
safe and reliable navigation is essential. However, fully autonomous UAV
operation can be challenging in dynamic or uncertain environments.

Human operators provide flexibility and decision-making capability during
flight, but continuous manual control may result in delayed reactions when
obstacles appear suddenly.

To address this limitation, this project proposes a **Human–Autonomy Shared
Control system** for UAV obstacle avoidance. Under normal flight conditions,
the UAV remains under human control. When a critical obstacle is detected,
the autonomous safety controller temporarily takes over to perform a safe
obstacle-avoidance maneuver.

Once the obstacle has been cleared and the UAV becomes stable, control
authority is automatically returned to the human operator.

The system is implemented and evaluated in the **GymPyBullet simulation
environment**, using a Crazyflie 2.x (CF2X) UAV and a PID-based position
controller.

---

# Two-Drone Collision Avoidance Simulation

A Python-based 3D drone simulation featuring two independently controlled drones, static and moving obstacles, predicted collision avoidance, PID-based flight control, and real-time camera visualization.

## Table of Contents

1. [Project Idea](#1-project-idea)
2. [Problem Statement](#2-problem-statement)
3. [Objectives](#3-objectives)
4. [Main Features](#4-main-features)
5. [Tools and Libraries](#5-tools-and-libraries)
6. [System Architecture](#6-system-architecture)
7. [How the Project Works](#7-how-the-project-works)
8. [Mathematical Formulas Used](#8-mathematical-formulas-used)
9. [Obstacle Types and Motion Equations](#9-obstacle-types-and-motion-equations)
10. [Manual Controls](#10-manual-controls)
11. [Control Modes and Safety Logic](#11-control-modes-and-safety-logic)
12. [PID Control and Drone Physics](#12-pid-control-and-drone-physics)
13. [Camera Display and Trails](#13-camera-display-and-trails)
14. [Project Configuration](#14-project-configuration)
15. [Installation](#15-installation)
16. [How to Run](#16-how-to-run)
17. [How to Test the Project](#17-how-to-test-the-project)
18. [Project Structure](#18-project-structure)
19. [Limitations](#19-limitations)
20. [Possible Future Improvements](#20-possible-future-improvements)
21. [Troubleshooting](#21-troubleshooting)
22. [Viva Questions and Answers](#22-viva-questions-and-answers)
23. [Conclusion](#23-conclusion)

---

## 1. Project Idea

The project creates a virtual 3D environment in which two drones fly simultaneously. The user controls both drones using separate keyboard keys. During flight, the drones must avoid a pillar, a box, three moving spherical obstacles, and each other.

Each drone has its own operating mode:

- **MANUAL mode:** The drone follows the user's keyboard commands.
- **AUTO mode:** The drone generates an avoidance movement when an obstacle becomes a threat.
- **Emergency avoidance:** Automatic avoidance can override manual commands when the drone gets dangerously close to an obstacle.

The drones make decisions independently. For example, Drone 1 can enter automatic avoidance while Drone 2 remains under manual control.

This project demonstrates obstacle detection, relative-motion prediction, collision avoidance, smooth velocity control, and low-level flight control in a repeatable simulation.

**Note:** This is an educational simulation, not a certified real-world drone safety system.

## 2. Problem Statement

A manually controlled drone may collide with an obstacle if the pilot reacts too late, particularly when the obstacle is moving or another drone is nearby. Controlling two drones simultaneously makes the task more challenging.

This project addresses the problem by continuously checking the distance and relative motion between each drone and its obstacles. It estimates the closest possible approach, calculates an avoidance direction, and temporarily changes the drone's desired velocity when required.

## 3. Objectives

- Simulate two quadrotor drones in a 3D physics environment.
- Provide independent keyboard control for both drones.
- Include static and moving obstacles.
- Treat the other drone as a moving obstacle.
- Estimate the closest approach between each drone and an obstacle.
- Generate smooth avoidance directions and velocity commands.
- Support manual and automatic control modes.
- Use PID-based control to convert target states into motor commands.
- Display multiple camera views and colored flight trails.
- Keep target positions within configured arena and altitude limits.

## 4. Main Features

| Feature | Description |
|---|---|
| Two-drone control | Controls two drones using separate keyboard layouts. |
| Static obstacles | Includes a pillar and a box. |
| Moving obstacles | Includes three spheres with different motion patterns. |
| Drone-to-drone avoidance | Treats the other drone as a moving obstacle. |
| Closest-approach prediction | Estimates the future closest relative position. |
| Graduated avoidance | Increases avoidance strength as the effective gap decreases. |
| Passing-side memory | Helps reduce unnecessary left-right switching. |
| Emergency override | Allows stronger avoidance when the drone is dangerously close to an obstacle. |
| Velocity smoothing | Limits sudden changes in commanded velocity. |
| Target-position leash | Limits how far the target can move from the current drone position. |
| Position constraints | Restricts the target to configured arena and altitude boundaries. |
| Camera tiles | Displays multiple simulation views using OpenCV. |
| Flight trails | Shows the recent movement of each drone. |

## 5. Tools and Libraries

| Technology | Purpose |
|---|---|
| Python | Main programming language and simulation logic. |
| gym-pybullet-drones | Provides drone models, simulation environment, and control interfaces. |
| PyBullet | Simulates physics, collision shapes, and 3D rendering. |
| NumPy | Performs vector calculations, distance measurements, normalization, and numerical clipping. |
| OpenCV (`cv2`) | Displays camera images and combines camera tiles. |
| pynput | Reads keyboard events while the simulation is running. |
| threading | Allows keyboard listening to run separately from the simulation loop. |
| time | Tracks elapsed time and supports real-time synchronization. |
| DSLPIDControl | Converts target states into four motor RPM commands using PID-based control. |

### Is this an AI or machine-learning project?

The current implementation uses **geometric, rule-based obstacle avoidance**. It does not train a neural network or use a learned policy. Collision-risk estimation is based on relative position, relative velocity, distance thresholds, and mathematical equations.

## 6. System Architecture

The program repeatedly executes the following control loop:

```text
Keyboard Input
      |
      v
Build Manual Movement Commands
      |
      v
Read Drone Positions, Velocities, and Headings
      |
      v
Update Moving Obstacles
      |
      v
Calculate Gaps and Predict Closest Approach
      |
      v
Calculate Avoidance Direction and Threat Strength
      |
      v
Select MANUAL or AUTO Mode
      |
      v
Smooth Velocity and Update Target Position
      |
      v
PID Controller Calculates Motor RPM Commands
      |
      v
PyBullet Advances the Physics Simulation
      |
      v
Update Camera Views and Flight Trails
      |
      +------------- Repeat -------------+
```

Each drone has its own control and mode-selection logic. Both drones share the same environment, allowing each to consider the other drone's position and velocity.

## 7. How the Project Works

### Step 1: Create the simulation

The program creates a `CtrlAviary` environment containing two CF2X quadrotor models. Each drone starts at a configured 3D position. The environment also contains the floor and obstacles.

### Step 2: Start keyboard listening

A keyboard listener records pressed keys. The program uses a short one-second warm-up period before accepting movement commands, allowing the simulation to initialize.

### Step 3: Read each drone's state

The program reads the current position, velocity, and orientation information of each drone. These values are used to determine the drone's movement and heading.

### Step 4: Convert keyboard inputs into movement commands

The program converts forward, backward, left, right, up, and down commands into a 3D velocity vector. Horizontal movement is calculated relative to the drone's yaw angle.

If the combined input exceeds the allowed magnitude, the vector is normalized and scaled to the configured manual speed.

### Step 5: Update moving obstacles

The three moving spheres receive new positions based on simulation time. Their velocities are calculated from the derivatives of their motion equations.

### Step 6: Assess possible collisions

For each obstacle, the program calculates:

1. Relative position.
2. Relative velocity.
3. Current surface-to-surface gap.
4. Estimated time of closest approach.
5. Predicted relative position at that time.
6. Effective gap and avoidance strength.

The pillar is assessed horizontally, while spherical obstacles and the other drone are assessed in 3D.

### Step 7: Generate an avoidance vector

When an obstacle's effective gap falls below the avoidance threshold, the program calculates a direction away from the predicted close-approach point. It adds a lateral passing direction and remembers the selected passing side while the obstacle remains an active threat.

Contributions from multiple active obstacles are combined.

### Step 8: Select the operating mode

Each drone independently decides whether to remain in MANUAL mode or enter AUTO mode. Emergency proximity can trigger automatic avoidance even while movement keys are pressed.

When the path remains clear long enough, or another permitted exit condition is met, the drone can return to MANUAL mode.

### Step 9: Smooth and constrain movement

The program gradually changes the commanded velocity rather than applying abrupt changes. It updates the target position and constrains it to the configured arena and altitude boundaries.

A target-position leash limits the distance between the target and the current drone position.

### Step 10: Generate motor commands

The `DSLPIDControl` controller receives the current drone state and the desired target position, velocity, and yaw. It calculates four motor RPM commands.

The simulation environment applies those commands and advances the drone's physics.

### Step 11: Display the result

The program updates the camera tiles and draws colored flight trails. The control loop repeats until the user exits the simulation.

## 8. Mathematical Formulas Used

The following formulas describe the mathematical concepts used by the project. Equations are formatted using GitHub-supported Markdown math syntax.

### 8.1 Position and Velocity Vectors

The position vector of a drone or obstacle is:

$$
\mathbf{p} =
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
$$

Its velocity vector is:

$$
\mathbf{v} =
\begin{bmatrix}
v_x \\
v_y \\
v_z
\end{bmatrix}
$$

Where:

- `x`, `y`, and `z` represent position coordinates in metres.
- `v_x`, `v_y`, and `v_z` represent velocity components in metres per second.

The Euclidean magnitude of a vector is:

$$
\|\mathbf{a}\| =
\sqrt{a_x^2 + a_y^2 + a_z^2}
$$

Vector magnitude is used to calculate distances and normalize directions.

### 8.2 Relative Position

Let `p_d` represent the drone position and `p_o` represent the obstacle position.

The relative position is:

$$
\mathbf{r} = \mathbf{p}_o - \mathbf{p}_d
$$

This vector points from the drone toward the obstacle.

For the pillar, vertical separation is ignored:

$$
\mathbf{r}_{xy} =
\begin{bmatrix}
x_o - x_d \\
y_o - y_d \\
0
\end{bmatrix}
$$

For spherical obstacles, all three coordinates are used.

### 8.3 Relative Velocity

Let `v_d` be the drone velocity and `v_o` be the obstacle velocity.

$$
\mathbf{v}_r = \mathbf{v}_o - \mathbf{v}_d
$$

Relative velocity describes how the obstacle moves compared with the drone.

An obstacle may be far away but still pose a threat if their relative motion brings them closer together.

### 8.4 Current Surface-to-Surface Gap

The centre-to-centre distance is:

$$
d = \|\mathbf{r}\|
$$

The estimated surface gap is:

$$
g = d - R_o - R_d
$$

Where:

- `R_o` is the obstacle's effective radius.
- `R_d` is the drone radius, configured as `0.10 m`.

For the other drone, the effective obstacle radius is configured as `0.12 m`. The box and pillar use configured effective dimensions in the avoidance representation.

A negative gap indicates overlap according to this simplified radius-based model.

### 8.5 Time of Closest Approach (CPA)

The program estimates the time at which the relative distance between the drone and an obstacle will be smallest, assuming constant relative velocity.

The time of closest approach is calculated using:

$$
t_{\mathrm{CPA}} =
\min\left(
T,
\max\left(
0,
-\frac{\mathbf{r}\cdot\mathbf{v}_r}
{\mathbf{v}_r\cdot\mathbf{v}_r}
\right)
\right)
$$

Where:

- $\mathbf{r}$ is the relative position vector.
- $\mathbf{v}_r$ is the relative velocity vector.
- $T$ is the prediction horizon, configured as `PREDICT_HORIZON = 2.0 s`.
- The dot product is used to calculate how the relative position changes along the relative velocity direction.

If the relative velocity is nearly zero, the program uses $t_{\mathrm{CPA}} = 0$ to avoid division by a value close to zero.

**Interpretation:** The result estimates when the drone and obstacle will be closest within the configured prediction horizon. It is a constant-velocity approximation, not a complete simulation of their future trajectories.


### 8.6 Predicted Relative Position

The predicted relative position at the CPA time is:

$$
\mathbf{r}_{\mathrm{pred}} =
\mathbf{r} + \mathbf{v}_r t_{\mathrm{CPA}}
$$

The predicted surface gap is:

$$
g_{\mathrm{pred}} =
\|\mathbf{r}_{\mathrm{pred}}\| - R_o - R_d
$$

The effective gap is:

$$
g_{\mathrm{effective}} =
\min(g, g_{\mathrm{pred}})
$$

This allows the avoidance logic to consider both the current separation and the predicted closest separation.

### 8.7 Avoidance Strength

The avoidance strength is calculated using the following formula:

$$
s = \min\left(1,\max\left(0,\frac{D_{\mathrm{avoid}}-g_{\mathrm{effective}}}{D_{\mathrm{avoid}}}\right)\right)
$$

Where:

- $s$ is the avoidance strength, ranging from 0 to 1.
- $D_{\mathrm{avoid}}$ is the avoidance distance, configured as `1.5 m`.
- $g_{\mathrm{effective}}$ is the effective surface gap between the drone and the obstacle.

**Interpretation:**

- If the effective gap is greater than or equal to the avoidance distance, the avoidance strength is 0.
- As the effective gap decreases, the avoidance strength increases gradually.
- When the effective gap reaches zero or becomes negative, the avoidance strength reaches 1.

This formula provides a gradual response instead of simply switching obstacle avoidance on or off. It is a mathematical rule, not a machine-learning prediction.

### 8.8 Direction Away From the Predicted Approach

The initial avoidance vector points away from the predicted relative position:

$$
\mathbf{a} = -\mathbf{r}_{\mathrm{pred}}
$$

If the predicted vector is too small to define a reliable direction, the program uses the negative current relative position.

The normalized avoidance direction is:

$$
\hat{\mathbf{a}} =
\frac{\mathbf{a}}{\|\mathbf{a}\|}
$$

The vertical component is scaled by `0.5`, reducing the vertical contribution to the avoidance direction.

### 8.9 Lateral Passing Direction

The horizontal left-of-heading vector is:

$$
\mathbf{l} =
\begin{bmatrix}
-h_y \\
h_x \\
0
\end{bmatrix}
$$

The program selects a passing side and stores it in `side_memory` while the obstacle remains active.

The combined avoidance direction before normalization is:

$$
\mathbf{q} =
\hat{\mathbf{a}} + 0.9\mathbf{l}_{\mathrm{chosen}}
$$

The final normalized direction is:

$$
\hat{\mathbf{q}} =
\frac{\mathbf{q}}{\|\mathbf{q}\|}
$$

Remembering the selected passing side helps reduce repeated switching between left and right when small changes occur in the relative geometry.

### 8.10 Combining Avoidance Contributions

For multiple active obstacles, the total avoidance vector is:

$$
\mathbf{e} =
\sum_i s_i\hat{\mathbf{q}}_i
$$

Where:

- `s_i` is the avoidance strength for obstacle `i`.
- `q_i` is the corresponding normalized avoidance direction.
- `e` is the combined avoidance vector.

The program also records the maximum avoidance strength among active obstacles and whether a threat is active.

### 8.11 Manual Movement Relative to Yaw

Let `psi` be the drone's yaw angle.

The horizontal forward and left unit vectors are:

$$
\mathbf{f} =
\begin{bmatrix}
\cos\psi \\
\sin\psi
\end{bmatrix}
$$

$$
\mathbf{l} =
\begin{bmatrix}
-\sin\psi \\
\cos\psi
\end{bmatrix}
$$

If `u_f` is the forward/backward input and `u_l` is the left/right input, the horizontal velocity components are:

$$
v_x = u_f\cos\psi - u_l\sin\psi
$$

$$
v_y = u_f\sin\psi + u_l\cos\psi
$$

The vertical velocity command is:

$$
v_z = u_z
$$

The three components are combined into a manual velocity vector. If its magnitude exceeds one, the vector is normalized and scaled by the maximum manual speed:

$$
\mathbf{v}_{\mathrm{manual}}
\leftarrow
\mathbf{v}_{\mathrm{manual}}V_{\max}
$$

Where `V_max = 0.50 m/s`.

### 8.12 Automatic Avoidance Speed

The automatic avoidance speed is:

$$
V_{\mathrm{avoid}} =
V_a\min(1,\|\mathbf{e}\|)
$$

Where `V_a = 0.60 m/s`.

The desired velocity is:

$$
\mathbf{v}_{\mathrm{desired}} =
\frac{\mathbf{e}}{\|\mathbf{e}\|}
V_{\mathrm{avoid}}
$$

This expression applies when the avoidance vector is non-zero.

During emergency proximity, the configured speed is:

`EMERGENCY_SPEED = 0.90 m/s`

If AUTO mode has no active avoidance vector, the desired velocity is set to zero so the drone can brake toward a hover target.

### 8.13 Emergency Condition

Emergency proximity is detected when the nearest current surface gap is below the emergency distance:

$$
g_{\mathrm{nearest}} < D_{\mathrm{emergency}}
$$

Where:

`D_emergency = 0.65 m`

This condition can trigger automatic override and allow a higher velocity-change limit.

### 8.14 Velocity Slew Limiting

The velocity difference is:

$$
\Delta\mathbf{v} =
\mathbf{v}_{\mathrm{desired}} -
\mathbf{v}_{\mathrm{current}}
$$

If the velocity change is too large, the program limits its magnitude:

$$
\Delta\mathbf{v}
\leftarrow
\Delta\mathbf{v}
\min\left(
1,
\frac{\Delta v_{\max}}
{\|\Delta\mathbf{v}\|}
\right)
$$

The maximum permitted velocity change is:

$$
\Delta v_{\max} = a_{\max}\Delta t
$$

The new commanded velocity is:

$$
\mathbf{v}_{\mathrm{new}} =
\mathbf{v}_{\mathrm{current}} +
\Delta\mathbf{v}
$$

Normal acceleration is `1.0 m/s²`, and emergency acceleration is `2.2 m/s²`.

Velocity smoothing reduces abrupt changes while allowing a stronger response during emergency avoidance.

### 8.15 Target Position Update

The target position is updated using:

$$
\mathbf{p}_{\mathrm{target,new}} =
\mathbf{p}_{\mathrm{target,old}} +
\mathbf{v}_{\mathrm{cmd}}\Delta t
$$

The target is constrained to the configured arena boundaries:

$$
-4.5 \leq x \leq 4.5
$$

$$
-4.5 \leq y \leq 4.5
$$

$$
0.35 \leq z \leq 2.5
$$

The leash limits the target's distance from the current drone position to `0.30 m`.

Let the target error be:

$$
\mathbf{d} =
\mathbf{p}_{\mathrm{target}} -
\mathbf{p}_{\mathrm{drone}}
$$

If the target error exceeds the leash distance `L`, the target is reset to:

$$
\mathbf{p}_{\mathrm{target}} =
\mathbf{p}_{\mathrm{drone}} +
\mathbf{d}\frac{L}{\|\mathbf{d}\|}
$$

This limits target-position jumps but does not guarantee collision-free motion.

### 8.16 Yaw Update

The target yaw angle is updated when the drone's turn key is pressed.

The formula is:

$$
\psi_{\mathrm{target,new}} =
\operatorname{atan2}\left(
\sin(\psi_{\mathrm{target,old}} + u_{\mathrm{yaw}}\omega_{\mathrm{yaw}}\Delta t),
\cos(\psi_{\mathrm{target,old}} + u_{\mathrm{yaw}}\omega_{\mathrm{yaw}}\Delta t)
\right)
$$

Where:

- $\psi_{\mathrm{target,new}}$ is the updated target yaw angle.
- $\psi_{\mathrm{target,old}}$ is the previous target yaw angle.
- $u_{\mathrm{yaw}}$ represents the turning input.
- $\omega_{\mathrm{yaw}}$ is the configured yaw rate, `0.8 rad/s`.
- $\Delta t$ is the control time step.

The `atan2` expression wraps the angle into the range $[-\pi,\pi]$, keeping the yaw value within a standard angular interval.

This allows the drone to turn smoothly in the desired direction without its target yaw angle increasing or decreasing indefinitely.

### 8.17 Simulation Time Steps

The physics simulation frequency is:

`PYB_FREQ = 240 Hz`

The control frequency is:

`CTRL_FREQ = 48 Hz`

The physics time step is:

$$
\Delta t_{\mathrm{physics}} =
\frac{1}{240}
\approx 0.00417\ \mathrm{s}
$$

The control time step is:

$$
\Delta t_{\mathrm{control}} =
\frac{1}{48}
\approx 0.02083\ \mathrm{s}
$$

The physics simulation runs more frequently than the control logic. This allows physics integration to occur at a higher update rate than the controller.

### 8.18 PID Control

The general PID controller equation is:

$$
u(t) =
K_Pe(t) +
K_I\int_0^t e(\tau)\,d\tau +
K_D\frac{de(t)}{dt}
$$

Where:

- `K_P` is the proportional gain.
- `K_I` is the integral gain.
- `K_D` is the derivative gain.
- `e(t)` is the control error.
- `u(t)` is the controller output.

The project uses `DSLPIDControl` from `gym-pybullet-drones`. The library handles the actual controller calculations and motor mapping. The equation above explains the general PID principle rather than the exact implementation details of the library.

## 9. Obstacle Types and Motion Equations

The environment contains two static obstacles, three moving spheres, and the other drone.

### 9.1 Static Pillar

The pillar configuration is:

```python
PILLAR = (2.0, 0.0, 0.25, 2.5)
```

The values represent:

- X position: `2.0 m`
- Y position: `0.0 m`
- Radius: `0.25 m`
- Height: `2.5 m`

The avoidance calculation assesses the pillar horizontally by ignoring vertical separation. Therefore, this is a horizontal clearance estimate rather than an exact 3D cylinder-distance calculation.

### 9.2 Static Box

The box configuration is:

```python
BOX_POS = [3.0, 1.5, 1.0]
BOX_HALF = 0.30
```

The program creates the box in PyBullet and uses an effective radius for the avoidance calculation. This is an approximation rather than an exact distance-to-box-surface calculation.

### 9.3 Moving Sphere 1 — Patrol

The first sphere moves back and forth along the Y-axis:

$$
x(t) = 1.0
$$

$$
y(t) = 2\sin(0.35t)
$$

$$
z(t) = 1.0
$$

Its velocity is:

$$
v_x(t) = 0
$$

$$
v_y(t) = 0.7\cos(0.35t)
$$

$$
v_z(t) = 0
$$

This creates a predictable patrol path.

### 9.4 Moving Sphere 2 — Orbit

The second sphere follows a circular horizontal path around `(-1.5, 1.0)`:

$$
x(t) = -1.5 + 1.2\cos(0.40t)
$$

$$
y(t) = 1.0 + 1.2\sin(0.40t)
$$

$$
z(t) = 1.2
$$

Its velocity is:

$$
v_x(t) = -0.48\sin(0.40t)
$$

$$
v_y(t) = 0.48\cos(0.40t)
$$

$$
v_z(t) = 0
$$

The circle has a radius of `1.2 m` and an angular rate of `0.40 rad/s`.

### 9.5 Moving Sphere 3 — Vertical Bob

The third sphere moves vertically at a fixed X/Y position:

$$
x(t) = 2.5
$$

$$
y(t) = -2.2
$$

$$
z(t) = 1.2 + 0.6\sin(0.80t)
$$

Its vertical velocity is:

$$
v_z(t) = 0.48\cos(0.80t)
$$

The other velocity components are zero. Its height oscillates around `1.2 m`.

### 9.6 The Other Drone

Each drone includes the other drone's current position and velocity in its obstacle list. This allows the same relative-position and closest-approach calculations to be used for drone-to-drone avoidance.

## 10. Manual Controls

The two drones use separate keyboard layouts.

| Action | Drone 1 (Blue) | Drone 2 (Pink) |
|---|---|---|
| Forward | `W` | `I` |
| Backward | `S` | `K` |
| Left | `A` | `J` |
| Right | `D` | `L` |
| Up | `E` | `O` |
| Down | `Q` | `U` |
| Turn left | `Z` | `N` |
| Turn right | `C` | `M` |
| Exit | `ESC` | `ESC` |

**Note:** The keyboard listener reads global keyboard events, so keys may work even when the simulation window is not focused. Some keyboards cannot reliably register many simultaneous key presses because of hardware key rollover limitations.

## 11. Control Modes and Safety Logic

### MANUAL Mode

- The drone follows movement commands from its keyboard controls.
- Its target yaw changes when a turn key is held.
- The program continues assessing obstacles during manual flight.
- Emergency proximity can override manual commands and trigger AUTO mode.
- Outside emergency conditions, the drone normally enters AUTO after its keys have been released and the handover grace period has expired.

### AUTO Mode

- The drone uses the generated avoidance vector to determine its desired velocity.
- It can use a higher speed when the nearest gap is below the emergency threshold.
- If there is no active avoidance vector, the desired velocity becomes zero.
- The drone can return to MANUAL when the path has remained clear for the configured hold time, when manual control is requested outside emergency conditions, or when the normal AUTO timeout is reached.

### Mode-Related Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `AVOID_DISTANCE` | 1.5 m | Gap at which avoidance strength begins to increase. |
| `EMERGENCY_DISTANCE` | 0.65 m | Gap threshold for emergency behavior. |
| `PREDICT_RANGE` | 2.5 m | Current-gap cutoff for generating an avoidance contribution. |
| `PREDICT_HORIZON` | 2.0 s | Maximum future time used for closest-approach prediction. |
| `RELEASE_HOLD` | 0.8 s | Time the path must remain clear before leaving AUTO. |
| `AUTO_MAX_TIME` | 3.0 s | Normal maximum duration of AUTO mode; emergency can extend it. |
| `MANUAL_GRACE` | 1.5 s | Delay after handing control back before a new non-emergency AUTO handover. |

This is a hybrid control approach: manual commands are used most of the time, while a rule-based safety behavior can temporarily take over.

## 12. PID Control and Drone Physics

Obstacle avoidance decides where the drone should try to move. It does not directly control the four motors.

The program passes the target position, target velocity, and target yaw to `DSLPIDControl`. The controller compares the desired state with the current drone state and calculates the required motor RPM commands.

The simulation environment applies these commands and simulates the drone's physical response.

```text
Obstacle-Avoidance / Manual Logic
               |
               v
Desired Velocity and Target Position
               |
               v
DSLPIDControl
               |
               v
Four Motor RPM Commands
               |
               v
PyBullet Drone Physics
               |
               v
Updated Position, Velocity, and Orientation
```

The avoidance logic is the high-level decision layer, while the PID controller is the low-level flight-control layer.



## 13. Conclusion

The Two-Drone Collision Avoidance Simulation combines manual keyboard control, relative-motion analysis, closest-point-of-approach prediction, geometric obstacle avoidance, velocity smoothing, PID-based motor control, and 3D physics simulation.

The most important concept is that the system does not consider only where an obstacle is currently located. It also considers how the drone and obstacle are moving relative to one another.

Using their relative positions and velocities, the program estimates a potential closest approach and calculates an avoidance response. The desired movement is then smoothed and constrained before being passed to the PID-based controller, which generates motor RPM commands for the physics simulator.

This project provides a practical foundation for understanding multi-drone interaction, collision-risk estimation, hybrid control systems, and autonomous navigation concepts.

The implementation uses mathematical and rule-based obstacle avoidance rather than a trained machine-learning model. It is intended for learning and demonstration and does not guarantee collision-free movement.

