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

A Python-based 3D drone simulation that demonstrates manual flight control, automatic obstacle avoidance, relative-motion prediction, PID-based control, and multi-camera visualization using PyBullet.

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Problem Statement](#2-problem-statement)
3. [Objectives](#3-objectives)
4. [Main Features](#4-main-features)
5. [Technologies Used](#5-technologies-used)
6. [System Architecture](#6-system-architecture)
7. [How the Project Works](#7-how-the-project-works)
8. [Mathematical Formulas](#8-mathematical-formulas)
9. [Obstacle Types and Motion Equations](#9-obstacle-types-and-motion-equations)
10. [Keyboard Controls](#10-keyboard-controls)
11. [Control Modes and Safety Logic](#11-control-modes-and-safety-logic)
12. [PID Control and Drone Physics](#12-pid-control-and-drone-physics)
13. [Camera Display and Flight Trails](#13-camera-display-and-flight-trails)
14. [Configuration Parameters](#14-configuration-parameters)
15. [Installation](#15-installation)
16. [How to Run](#16-how-to-run)
17. [Testing the Project](#17-testing-the-project)
18. [Project Structure](#18-project-structure)
19. [Limitations](#19-limitations)
20. [Future Improvements](#20-future-improvements)
21. [Troubleshooting](#21-troubleshooting)
22. [Viva Questions and Answers](#22-viva-questions-and-answers)
23. [Conclusion](#23-conclusion)

---

## 1. Project Overview

The **Two-Drone Collision Avoidance Simulation** is a Python project that simulates two quadrotor drones flying simultaneously in a three-dimensional environment.

Both drones can be controlled independently using keyboard inputs. While moving, they must avoid static obstacles, moving obstacles, and each other.

The project implements a hybrid control system with two primary operating modes:

- **MANUAL Mode:** The drone follows the user's keyboard commands.
- **AUTO Mode:** The drone temporarily generates avoidance movements when a potential collision is detected.

An emergency avoidance mechanism can override manual commands when an obstacle becomes dangerously close.

Each drone makes its own control decisions. For example, Drone 1 can enter automatic avoidance while Drone 2 remains under manual control.

### Why this project?

In real-world drone navigation, following movement commands without considering nearby objects can cause collisions. This project demonstrates how position, velocity, distance calculations, motion prediction, and feedback control can be combined to reduce collision risk.

**Note:** This is a simulation and educational demonstration, not a certified real-world drone safety system.

## 2. Problem Statement

Manually controlling drones can be difficult when multiple obstacles are present, especially when obstacles are moving or multiple drones share the same environment.

A drone may collide with an obstacle if the pilot reacts too late or fails to account for relative motion.

The objective is to develop a simulation that continuously monitors drone and obstacle positions, predicts potential close approaches, calculates avoidance directions, and adjusts the desired velocity when necessary.

The system should also preserve manual control whenever automatic intervention is not required.

## 3. Objectives

- Simulate two CF2X quadrotor drones in a 3D physics environment.
- Provide independent keyboard controls for both drones.
- Implement static and moving obstacles.
- Treat the other drone as a potential collision obstacle.
- Predict the closest approach using relative position and velocity.
- Calculate graduated avoidance strength based on estimated clearance.
- Support MANUAL and AUTO operating modes.
- Implement emergency avoidance behavior.
- Smooth commanded velocity changes using acceleration limits.
- Convert desired drone states into motor RPM commands through PID-based control.
- Display multiple camera views and colored flight trails.
- Constrain target positions to configured arena boundaries.

## 4. Main Features

| Feature | Description |
|---|---|
| Two-drone simulation | Simulates two drones simultaneously. |
| Independent keyboard control | Uses separate keys for each drone. |
| Static obstacles | Includes a pillar and a box. |
| Moving obstacles | Includes three spheres following different motion patterns. |
| Drone-to-drone avoidance | Treats the other drone as a moving obstacle. |
| Closest-approach prediction | Estimates future relative separation over a short time horizon. |
| Graduated avoidance | Increases avoidance strength as the estimated gap decreases. |
| Passing-side memory | Retains a selected passing side to reduce left-right switching. |
| Emergency override | Applies stronger avoidance commands near obstacles. |
| Velocity smoothing | Limits the rate of change of commanded velocity. |
| Position constraints | Keeps target positions inside configured boundaries. |
| Target-position leash | Limits the distance between the target and current drone position. |
| Multi-camera visualization | Displays multiple simulation viewpoints. |
| Flight trails | Shows the recent movement paths of both drones. |

## 5. Technologies Used

| Technology | Purpose |
|---|---|
| Python | Main programming language and simulation logic. |
| `gym-pybullet-drones` | Provides drone models, simulation environments, and control interfaces. |
| PyBullet | Simulates 3D physics, collisions, objects, and camera rendering. |
| NumPy | Performs vector calculations, normalization, distance calculations, and numerical clipping. |
| OpenCV (`cv2`) | Displays and combines camera images. |
| `pynput` | Captures keyboard events. |
| `threading` | Allows keyboard listening to run separately from the simulation loop. |
| `time` | Supports timing and real-time synchronization. |
| `DSLPIDControl` | Converts desired drone states into motor RPM commands using a PID-based controller. |

### Is this an AI or machine-learning project?

The current implementation uses **geometric, mathematical, and rule-based obstacle avoidance**. It does not train a neural network or use a learned machine-learning policy.

Collision risk is estimated using relative-motion equations, and avoidance decisions are made using programmed rules and thresholds.

## 6. System Architecture

The system repeatedly performs the following operations:

```text
                 Keyboard Input
                       |
                       v
          Generate Manual Commands
                       |
                       v
       Read Drone Positions and Velocities
                       |
                       v
              Update Obstacles
                       |
                       v
        Calculate Relative Position
             and Relative Velocity
                       |
                       v
        Predict Closest Approach (CPA)
                       |
                       v
         Calculate Avoidance Direction
            and Threat Strength
                       |
                       v
          Select MANUAL or AUTO Mode
                       |
                       v
        Smooth and Constrain Velocity
                       |
                       v
          Update Target Position
                       |
                       v
              DSLPIDControl
                       |
                       v
          Four Motor RPM Commands
                       |
                       v
              PyBullet Physics
                       |
                       v
        Update Cameras and Flight Trails
                       |
                       +---- Repeat ----+
```

Each drone has independent mode-selection and avoidance logic, but both operate within the same simulation environment.

## 7. How the Project Works

### Step 1: Initialize the simulation

The program creates a `CtrlAviary` environment containing two CF2X quadrotor models. The drones are initialized at configured positions, and the environment includes a floor and multiple obstacles.

### Step 2: Capture keyboard input

A keyboard listener records pressed keys. A short warm-up period allows the simulation to initialize before movement commands are accepted.

### Step 3: Read drone states

The program obtains the current position, velocity, and orientation information for each drone.

These values are used to understand the drone's movement and determine how its commanded direction relates to its heading.

### Step 4: Generate manual movement commands

Keyboard inputs are converted into forward, backward, left, right, upward, and downward velocity commands.

Horizontal movement depends on the drone's yaw angle. The resulting vector is normalized when necessary and scaled to respect the configured manual speed limit.

### Step 5: Update moving obstacles

The positions of the three moving spheres are updated using mathematical functions of simulation time.

Their velocities are obtained from the derivatives of these functions.

### Step 6: Assess collision risk

For each obstacle, the program calculates:

1. Relative position.
2. Relative velocity.
3. Current surface-to-surface gap.
4. Predicted time of closest approach.
5. Predicted relative position.
6. Effective gap and avoidance strength.

The pillar is assessed in the horizontal plane, whereas spherical obstacles and the other drone are assessed in three dimensions.

### Step 7: Calculate an avoidance direction

When an obstacle is considered a threat, the program calculates a direction away from the predicted close-approach point.

It combines this direction with a lateral passing direction. The selected passing side is remembered while the obstacle remains active to reduce unnecessary left-right switching.

Contributions from multiple active obstacles are combined.

### Step 8: Select a control mode

Each drone independently decides whether to remain in MANUAL mode or switch to AUTO mode.

Emergency proximity can trigger automatic intervention even when the user is pressing movement keys.

The system can return to MANUAL mode after the configured clear-path period or when another permitted exit condition is met.

### Step 9: Smooth and constrain movement

The desired velocity is approached gradually using acceleration limits.

The target position is updated and constrained to the configured arena and altitude boundaries. A target-position leash prevents the target from moving too far away from the current drone position in one update.

### Step 10: Calculate motor commands

`DSLPIDControl` receives the current drone state and desired target state. It calculates the four motor RPM commands required to move toward the target.

PyBullet then simulates the drone's physical response.

### Step 11: Visualize the simulation

The program updates multiple camera views and draws colored trails representing the recent paths of both drones.

The control loop continues until the application is stopped.

## 8. Mathematical Formulas

This section describes the principal mathematical concepts used by the obstacle-avoidance system.

All position and velocity vectors use three-dimensional coordinates unless otherwise specified.

### 8.1 Position and velocity vectors

The position of a drone or obstacle is represented by:

$$
\mathbf{p} =
\begin{bmatrix}
x\\y\\z
\end{bmatrix}
$$

Its velocity is represented by:

$$
\mathbf{v} =
\begin{bmatrix}
v_x\\v_y\\v_z
\end{bmatrix}
$$

Where:

- \(x,y,z\): position coordinates in metres.
- \(v_x,v_y,v_z\): velocity components in metres per second.

The magnitude of a vector is:

$$
\|\mathbf{a}\| =
\sqrt{a_x^2+a_y^2+a_z^2}
$$

Vector magnitude is used to calculate distances, speeds, and normalized directions.

### 8.2 Relative position

Let \(\mathbf{p}_d\) represent the drone position and \(\mathbf{p}_o\) represent the obstacle position.

The relative position is:

$$
\mathbf{r}=\mathbf{p}_o-\mathbf{p}_d
$$

This vector points from the drone toward the obstacle.

For the pillar, vertical separation is ignored:

$$
\mathbf{r}_{xy} =
\begin{bmatrix}
x_o-x_d\\
y_o-y_d\\
0
\end{bmatrix}
$$

For spherical obstacles, all three coordinates are considered.

### 8.3 Relative velocity

Let \(\mathbf{v}_d\) be the drone velocity and \(\mathbf{v}_o\) be the obstacle velocity.

The relative velocity is:

$$
\mathbf{v}_r=\mathbf{v}_o-\mathbf{v}_d
$$

Relative velocity describes how the obstacle moves compared with the drone.

An obstacle can be far away but still present a threat if their relative motion is bringing them closer together.

### 8.4 Current surface-to-surface gap

The centre-to-centre distance is:

$$
d=\|\mathbf{r}\|
$$

The estimated free gap is:

$$
g=d-R_o-R_d
$$

Where:

- \(R_o\): configured effective obstacle radius.
- \(R_d\): configured effective drone radius.

The project uses:

- `DRONE_RADIUS = 0.10 m`
- `OTHER_DRONE_RADIUS = 0.12 m`

For boxes and pillars, configured effective sizes are used in the avoidance representation. These are approximations rather than exact distances to every obstacle surface.

A negative gap indicates overlap according to the simplified radius-based model.

### 8.5 Time of Closest Approach (CPA)

The Time of Closest Approach estimates when the drone and obstacle will have their minimum relative separation, assuming constant relative velocity.

The formula is:

$$
t_{\mathrm{CPA}} =
\operatorname{clip}
\left(
-\frac{\mathbf{r}\cdot\mathbf{v}_r}
{\mathbf{v}_r\cdot\mathbf{v}_r},
0,T
\right)
$$

Where:

- \(\mathbf{r}\): current relative position.
- \(\mathbf{v}_r\): relative velocity.
- \(\cdot\): dot product.
- \(T\): maximum prediction horizon.
- `clip`: restricts the result to the interval from \(0\) to \(T\).

The project uses:

`PREDICT_HORIZON = 2.0 s`

If relative velocity is almost zero, the implementation uses \(t_{\mathrm{CPA}}=0\) to avoid unstable division.

**Interpretation:** CPA estimates the time of minimum separation within the prediction horizon. It is a mathematical approximation, not a complete simulation of future trajectories.

### 8.6 Predicted relative position

The predicted relative position at the CPA time is:

$$
\mathbf{r}_{\mathrm{pred}}
=
\mathbf{r}+\mathbf{v}_r t_{\mathrm{CPA}}
$$

The predicted gap is:

$$
g_{\mathrm{pred}}
=
\|\mathbf{r}_{\mathrm{pred}}\|-R_o-R_d
$$

The effective gap is:

$$
g_{\mathrm{effective}}
=
\min(g,g_{\mathrm{pred}})
$$

This allows the avoidance logic to consider both the current separation and the predicted closest separation.

### 8.7 Avoidance strength

The avoidance strength is calculated as:

$$
s =
\operatorname{clip}
\left(
\frac{D_{\mathrm{avoid}}-g_{\mathrm{effective}}}
{D_{\mathrm{avoid}}},
0,1
\right)
$$

Where:

`AVOID_DISTANCE = 1.5 m`

Interpretation:

- If the effective gap is at or above the avoidance distance, \(s=0\).
- As the effective gap decreases, the avoidance strength increases.
- At zero effective gap or below, the strength reaches \(1\).

This is a linear rule-based calculation, not a probability or machine-learning prediction.

The implementation also uses `PREDICT_RANGE = 2.5 m` as a current-gap cutoff for generating avoidance contributions. The nearest current gap is tracked separately for emergency checks.

### 8.8 Direction away from the predicted approach

The initial avoidance vector points away from the predicted relative position:

$$
\mathbf{a}=-\mathbf{r}_{\mathrm{pred}}
$$

If the predicted vector is too small, the implementation falls back to the negative current relative position.

The vector is then normalized:

$$
\hat{\mathbf{a}}=
\frac{\mathbf{a}}{\|\mathbf{a}\|}
$$

The vertical component is scaled by \(0.5\), reducing its contribution relative to the unscaled direction.

### 8.9 Lateral passing direction

Let the horizontal heading direction be \((h_x,h_y)\).

The left-of-heading vector is:

$$
\mathbf{l} =
\begin{bmatrix}
-h_y\\h_x\\0
\end{bmatrix}
$$

The program selects a passing side and stores it in `side_memory`.

The combined direction is:

$$
\mathbf{q}
=
\hat{\mathbf{a}}
+
0.9\mathbf{l}_{\mathrm{chosen}}
$$

The resulting direction is normalized:

$$
\hat{\mathbf{q}}=
\frac{\mathbf{q}}{\|\mathbf{q}\|}
$$

The factor \(0.9\) controls the contribution of the selected lateral passing direction.

### 8.10 Combining avoidance contributions

For multiple active obstacles, the program adds their weighted avoidance directions:

$$
\mathbf{e}=\sum_i s_i\hat{\mathbf{q}}_i
$$

Where:

- \(s_i\): avoidance strength for obstacle \(i\).
- \(\hat{\mathbf{q}}_i\): normalized avoidance direction for obstacle \(i\).
- \(\mathbf{e}\): total avoidance vector.

The implementation also tracks the maximum active avoidance strength and whether any avoidance threat is active.

### 8.11 Manual movement relative to yaw

Let \(\psi\) be the drone's yaw angle.

The horizontal forward and left unit vectors are:

$$
\mathbf{f}=
\begin{bmatrix}
\cos\psi\\
\sin\psi
\end{bmatrix}
$$

$$
\mathbf{l}=
\begin{bmatrix}
-\sin\psi\\
\cos\psi
\end{bmatrix}
$$

For forward/backward input \(u_f\) and left/right input \(u_l\):

$$
v_x=u_f\cos\psi-u_l\sin\psi
$$

$$
v_y=u_f\sin\psi+u_l\cos\psi
$$

The vertical command is:

$$
v_z=u_z
$$

The resulting vector is normalized if its magnitude exceeds \(1\), then scaled by the maximum manual speed:

$$
\mathbf{v}_{\mathrm{manual}}
\leftarrow
\mathbf{v}_{\mathrm{manual}}V_{\max}
$$

Where:

`MAX_SPEED = 0.50 m/s`

This prevents diagonal keyboard inputs from exceeding the configured manual speed.

### 8.12 Automatic avoidance speed

When AUTO mode is active and the avoidance vector is non-zero:

$$
V_{\mathrm{avoid}}
=
V_a\min(1,\|\mathbf{e}\|)
$$

Where:

`AVOID_SPEED = 0.60 m/s`

The desired velocity becomes:

$$
\mathbf{v}_{\mathrm{desired}}
=
\frac{\mathbf{e}}{\|\mathbf{e}\|}
V_{\mathrm{avoid}}
$$

During emergency proximity, the configured speed is:

`EMERGENCY_SPEED = 0.90 m/s`

If AUTO mode has no active avoidance vector, the desired velocity is set to zero so the drone can brake toward a hover target.

### 8.13 Emergency condition

The emergency condition is based on the nearest current surface gap:

$$
\text{emergency}
=
(g_{\mathrm{nearest}}<D_{\mathrm{emergency}})
$$

Where:

`EMERGENCY_DISTANCE = 0.65 m`

When this condition is satisfied, the mode logic can override manual movement commands and use a higher velocity-change limit.

### 8.14 Velocity slew limiting

The difference between desired and current commanded velocity is:

$$
\Delta\mathbf{v}
=
\mathbf{v}_{\mathrm{desired}}
-
\mathbf{v}_{\mathrm{current}}
$$

The maximum permitted change during one time step is:

$$
\Delta v_{\max}=a_{\max}\Delta t
$$

If the change exceeds this limit, it is scaled:

$$
\Delta\mathbf{v}
\leftarrow
\Delta\mathbf{v}
\min\left(
1,
\frac{\Delta v_{\max}}{\|\Delta\mathbf{v}\|}
\right)
$$

The updated velocity is:

$$
\mathbf{v}_{\mathrm{new}}
=
\mathbf{v}_{\mathrm{current}}
+
\Delta\mathbf{v}
$$

Configuration values:

- `MAX_ACCEL = 1.0 m/s²`
- `EMERGENCY_ACCEL = 2.2 m/s²`

This reduces sudden changes in commanded velocity while allowing a stronger response during emergencies.

### 8.15 Target position update and leash

The target position is updated using the commanded velocity:

$$
\mathbf{p}_{\mathrm{target,new}}
=
\mathbf{p}_{\mathrm{target,old}}
+
\mathbf{v}_{\mathrm{cmd}}\Delta t
$$

The target is constrained to the configured arena:

$$
-4.5\leq x\leq4.5
$$

$$
-4.5\leq y\leq4.5
$$

$$
0.35\leq z\leq2.5
$$

The leash distance is:

`LEASH = 0.30 m`

If the target error is:

$$
\mathbf{d}
=
\mathbf{p}_{\mathrm{target}}
-
\mathbf{p}_{\mathrm{drone}}
$$

and \(\|\mathbf{d}\|>L\), the target is constrained to:

$$
\mathbf{p}_{\mathrm{target}}
=
\mathbf{p}_{\mathrm{drone}}
+
\mathbf{d}\frac{L}{\|\mathbf{d}\|}
$$

This limits target-position jumps but does not independently guarantee collision-free movement.

### 8.16 Yaw update

When a turn key is held, the target yaw changes according to:

$$
\psi_{\mathrm{target,new}}
=
\operatorname{wrap}
\left(
\psi_{\mathrm{target,old}}
+
u_{\mathrm{yaw}}\omega_{\mathrm{yaw}}\Delta t
\right)
$$

Where:

- \(\omega_{\mathrm{yaw}}=0.8\ \mathrm{rad/s}\).
- `YAW_RATE = 0.8 rad/s`.
- `wrap` keeps the angle within \([-\pi,\pi)\).

### 8.17 Physics and control time steps

The simulation uses the following configured frequencies:

- Physics frequency: `PYB_FREQ = 240 Hz`
- Control frequency: `CTRL_FREQ = 48 Hz`

The corresponding time steps are:

$$
\Delta t_{\mathrm{physics}}
=
\frac{1}{240}
\approx0.00417\ \mathrm{s}
$$

$$
\Delta t_{\mathrm{control}}
=
\frac{1}{48}
\approx0.02083\ \mathrm{s}
$$

The physics simulation runs at a higher frequency than the control logic, allowing more frequent physics integration.

### 8.18 PID control

A conventional PID controller is described by:

$$
u(t)
=
K_Pe(t)
+
K_I\int_0^t e(\tau)\,d\tau
+
K_D\frac{de(t)}{dt}
$$

Where:

- \(e(t)\): control error.
- \(K_P\): proportional gain.
- \(K_I\): integral gain.
- \(K_D\): derivative gain.
- \(u(t)\): controller output.

The project calls `DSLPIDControl.computeControlFromState(...)` to calculate the four motor RPM commands.

**Implementation detail:** The PID controller is provided by `gym-pybullet-drones`; the project does not implement the complete PID equation directly in the main script. The equation explains the general PID principle.

---

## 9. Obstacle Types and Motion Equations

The environment contains two static obstacles, three moving spheres, and the other drone.

### 9.1 Static pillar

Configuration:

```python
PILLAR = (2.0, 0.0, 0.25, 2.5)
# x, y, radius, height
```

The pillar remains at a fixed horizontal location.

The avoidance calculation ignores vertical separation when assessing the pillar. Therefore, its gap represents horizontal clearance rather than an exact three-dimensional cylinder-surface distance.

### 9.2 Static box

Configuration:

```python
BOX_POS = [3.0, 1.5, 1.0]
BOX_HALF = 0.30
```

The box is created in PyBullet. The avoidance logic uses an effective radius to approximate its size.

This approximation is not an exact distance-to-box-surface calculation.

### 9.3 Moving sphere 1 — Patrol

The sphere moves back and forth along the Y-axis:

$$
x(t)=1.0
$$

$$
y(t)=2\sin(0.35t)
$$

$$
z(t)=1.0
$$

Its velocity is:

$$
v_x(t)=0
$$

$$
v_y(t)=0.7\cos(0.35t)
$$

$$
v_z(t)=0
$$

This creates a predictable back-and-forth movement pattern.

### 9.4 Moving sphere 2 — Orbit

The sphere moves in a horizontal circular path around \((-1.5,1.0)\):

$$
x(t)=-1.5+1.2\cos(0.40t)
$$

$$
y(t)=1.0+1.2\sin(0.40t)
$$

$$
z(t)=1.2
$$

Its velocity is:

$$
v_x(t)=-0.48\sin(0.40t)
$$

$$
v_y(t)=0.48\cos(0.40t)
$$

$$
v_z(t)=0
$$

The orbit radius is \(1.2\) metres, and the angular rate is \(0.40\) radians per second.

### 9.5 Moving sphere 3 — Vertical bob

The sphere oscillates vertically:

$$
x(t)=2.5
$$

$$
y(t)=-2.2
$$

$$
z(t)=1.2+0.6\sin(0.80t)
$$

Its vertical velocity is:

$$
v_z(t)=0.48\cos(0.80t)
$$

The horizontal velocity components are zero.

The sphere oscillates around a height of \(1.2\) metres.

### 9.6 The other drone

Each drone includes the other drone's current position and velocity in its obstacle list.

This allows both drones to use the same relative-position, relative-velocity, and closest-approach calculations for drone-to-drone avoidance.

## 10. Keyboard Controls

The drones use separate keyboard layouts.

| Action | Drone 1 (Blue) | Drone 2 (Pink) |
|---|---|---|
| Forward | `W` | `I` |
| Backward | `S` | `K` |
| Left | `A` | `J` |
| Right | `D` | `L` |
| Move up | `E` | `O` |
| Move down | `Q` | `U` |
| Turn left | `Z` | `N` |
| Turn right | `C` | `M` |
| Exit | `ESC` | `ESC` |

**Note:** The keyboard listener uses global keyboard events, so controls may work even when the simulation window is not focused. Some keyboards may not reliably register many simultaneous key presses because of hardware key-rollover limitations.

## 11. Control Modes and Safety Logic

### MANUAL Mode

- The drone follows keyboard movement commands.
- Turning keys update the target yaw.
- Obstacle assessment continues during manual flight.
- Emergency proximity can override manual commands.
- Outside emergency conditions, the mode-selection logic determines when automatic avoidance can activate.

### AUTO Mode

- The drone follows the generated avoidance direction.
- A higher configured speed can be used during emergency proximity.
- If no avoidance vector is active, the desired velocity becomes zero.
- The drone can return to MANUAL mode after the path remains clear for the configured period.
- Other exit conditions include permitted manual-control requests and the normal AUTO timeout.

### Mode-related parameters

| Parameter | Default | Purpose |
|---|---:|---|
| `AVOID_DISTANCE` | 1.5 m | Distance at which avoidance strength begins to increase. |
| `EMERGENCY_DISTANCE` | 0.65 m | Threshold for emergency behavior. |
| `PREDICT_RANGE` | 2.5 m | Current-gap cutoff for generating avoidance contributions. |
| `PREDICT_HORIZON` | 2.0 s | Maximum future time used for closest-approach prediction. |
| `RELEASE_HOLD` | 0.8 s | Clear-path duration used when leaving AUTO mode. |
| `AUTO_MAX_TIME` | 3.0 s | Normal maximum duration of AUTO mode; emergency behavior may extend it. |
| `MANUAL_GRACE` | 1.5 s | Delay after handing control back before another non-emergency automatic handover. |

This is a hybrid control system because it combines user-driven commands with rule-based automatic intervention.

## 12. PID Control and Drone Physics

Obstacle avoidance determines **where the drone should try to move**. It does not directly calculate the four motor outputs.

The high-level logic produces a desired velocity and target position. The `DSLPIDControl` controller compares the current drone state with the target state and calculates motor RPM commands.

PyBullet then simulates the physical effects of those commands.

The process is:

```text
Manual Control / Obstacle Avoidance
                 |
                 v
       Desired Velocity and Position
                 |
                 v
            DSLPIDControl
                 |
                 v
        Four Motor RPM Commands
                 |
                 v
           PyBullet Physics
                 |
                 v
       Updated Drone State
                 |
                 +---- Feedback ----+
```

The updated state is used in the next control iteration.

The key distinction is that obstacle avoidance is the **high-level decision layer**, while PID-based control is the **low-level flight-control layer**.

## 13. Camera Display and Flight Trails

The project provides visual feedback through multiple camera viewpoints.

- Camera images are rendered from different viewpoints.
- OpenCV combines and labels the camera tiles.
- Each tile is configured to be `320 × 240` pixels.
- Blue and pink trails display the recent paths of the two drones.
- Trails help visualize movement but do not directly affect control or collision avoidance.

## 14. Configuration Parameters

The main configuration values are defined near the top of the Python source file.

| Parameter | Default | Purpose |
|---|---:|---|
| `PYB_FREQ` | 240 Hz | Physics simulation frequency. |
| `CTRL_FREQ` | 48 Hz | Control update frequency. |
| `NUM_DRONES` | 2 | Number of simulated drones. |
| `MAX_SPEED` | 0.50 m/s | Maximum manual commanded speed. |
| `AVOID_SPEED` | 0.60 m/s | Base automatic avoidance speed. |
| `EMERGENCY_SPEED` | 0.90 m/s | Speed configured for emergency avoidance. |
| `MAX_ACCEL` | 1.0 m/s² | Normal commanded-velocity change limit. |
| `EMERGENCY_ACCEL` | 2.2 m/s² | Emergency commanded-velocity change limit. |
| `LEASH` | 0.30 m | Maximum target distance from the current position. |
| `YAW_RATE` | 0.8 rad/s | Target yaw change rate. |
| `Z_MIN` | 0.35 m | Minimum target altitude. |
| `Z_MAX` | 2.5 m | Maximum target altitude. |
| `XY_LIMIT` | 4.5 m | Absolute X/Y target boundary. |
| `DRONE_RADIUS` | 0.10 m | Effective radius used for drone gap estimates. |
| `OTHER_DRONE_RADIUS` | 0.12 m | Effective radius used for the other drone. |
| `TILE_W` | 320 px | Camera tile width. |
| `TILE_H` | 240 px | Camera tile height. |

Changing these parameters can affect the system's behavior.

For example, increasing `AVOID_DISTANCE` causes avoidance strength to begin increasing at a larger gap. Parameters should be evaluated together because changing one value can affect the overall behavior.

## 15. Installation

### Prerequisites

- Python installed on your computer.
- A desktop environment that supports PyBullet GUI windows and OpenCV display windows.
- The project source file, such as `33(1).py`.

### Step 1: Create a project folder

Open PowerShell or the VS Code terminal and navigate to your project directory.

### Step 2: Create a virtual environment

```powershell
python -m venv .venv
```

### Step 3: Activate the environment

```powershell
.\.venv\Scripts\Activate.ps1
```

If PowerShell blocks activation, use the virtual environment's Python executable directly.

### Step 4: Install dependencies

```powershell
python -m pip install --upgrade pip
python -m pip install numpy opencv-python pynput pybullet gym-pybullet-drones
```

If the environment is not activated, use:

```powershell
.\.venv\Scripts\python.exe -m pip install numpy opencv-python pynput pybullet gym-pybullet-drones
```

**Compatibility note:** Package compatibility depends on the Python version and the versions of `gym-pybullet-drones`, PyBullet, and NumPy. If installation fails, check the error message and the package's supported versions rather than installing arbitrary versions.

## 16. How to Run

1. Save the Python source file in your project folder.
2. Open the project folder in Visual Studio Code.
3. Open **Terminal → New Terminal**.
4. Ensure the correct Python environment is selected.
5. Run the program.

If the file is named `33(1).py`:

```powershell
python "33(1).py"
```

If the file has been renamed to `main.py`:

```powershell
python main.py
```

The simulation should open in a graphical window.

Wait for the warm-up period and then use the keyboard controls.

To exit, press `ESC` as handled by the application.

## 17. Testing the Project

The following test scenarios can be used during demonstrations and project reviews.

| Test | Procedure | Expected observation |
|---|---|---|
| Manual flight | Move Drone 1 using `W/A/S/D`. | Drone 1 responds to keyboard commands. |
| Independent control | Move Drone 2 using `I/J/K/L`. | Drone 2 responds to its separate key set. |
| Vertical movement | Use `E/Q` and `O/U`. | The respective drones receive vertical movement commands. |
| Yaw control | Use `Z/C` or `N/M`. | Heading changes affect the horizontal movement direction. |
| Static obstacle | Guide a drone toward the pillar or box. | Automatic avoidance may activate according to the threat and mode logic. |
| Moving obstacle | Observe the moving spheres. | The avoidance system responds to their changing positions and velocities. |
| Drone-to-drone interaction | Guide the drones toward the same region carefully. | Each drone independently assesses the other drone as an obstacle. |
| Mode handover | Release movement keys when a threat is present. | Mode behavior follows the configured handover conditions. |
| Emergency response | Observe a sufficiently small obstacle gap. | Emergency behavior may override manual input. |
| Camera visualization | Inspect the camera panels. | Multiple viewpoints are displayed. |
| Flight trails | Move both drones. | Colored trails show their recent paths. |

Results depend on initial positions, obstacle timing, keyboard commands, and the current simulation state. A single successful run does not establish a general collision-avoidance guarantee.

## 18. Project Structure

A suggested GitHub repository structure is:

```text
two-drone-collision-avoidance/
├── README.md
├── 33(1).py
├── requirements.txt
└── screenshots/
    ├── simulation-overview.png
    └── obstacle-avoidance.png
```

The screenshot names are examples. Include the screenshot files only after creating them.

### Generating `requirements.txt`

After testing the project in the working Python environment, generate a dependency list using:

```powershell
python -m pip freeze > requirements.txt
```

Review the resulting file before committing it. A complete environment freeze may include packages unrelated to this project.

If the Python filename changes, update the execution command in this README.

## 19. Limitations

- **Simulation only:** The program is designed for PyBullet simulation and is not configured for direct control of a physical drone.
- **No trained machine-learning model:** The avoidance logic is mathematical and rule-based.
- **Simplified obstacle geometry:** Some obstacles are represented using effective radii rather than exact surface-distance calculations.
- **Horizontal pillar assessment:** The pillar's avoidance gap ignores vertical separation.
- **Short prediction horizon:** Closest approach is estimated under a constant-relative-velocity assumption over at most two seconds.
- **No formal safety guarantee:** The system may fail to avoid collisions in difficult scenarios or under abrupt movement changes.
- **Keyboard limitations:** Hardware key rollover can affect simultaneous key presses.
- **Parameter sensitivity:** Speeds, acceleration limits, effective radii, and distance thresholds influence behavior.
- **External controller dependency:** Low-level PID behavior depends on the `gym-pybullet-drones` implementation and its configuration.

The project demonstrates important navigation and control concepts, but it should not be described as a production-ready autonomous navigation system.

## 20. Future Improvements

Potential extensions include:

- Compare the current rule-based method with velocity obstacles, artificial potential fields, or model-predictive control.
- Implement more accurate distance calculations for boxes and non-spherical objects.
- Record minimum drone-to-obstacle separation and the number of collisions.
- Measure avoidance events across repeatable scenarios.
- Add a dashboard displaying speed, current mode, nearest gap, and avoidance strength.
- Allow users to configure obstacle layouts and simulation parameters.
- Add repeatable benchmarks for comparing different avoidance configurations.
- Investigate a learned policy as a separate experiment with appropriate training and evaluation.
- Add demonstration GIFs, screenshots, and performance plots to the repository.

These are proposed improvements, not features implemented in the current version.

## 21. Troubleshooting

### `ModuleNotFoundError`

Install the missing dependency using the same Python environment used to run the program:

```powershell
python -m pip install numpy opencv-python pynput pybullet gym-pybullet-drones
```

### Wrong Python interpreter in VS Code

1. Open the Command Palette.
2. Select **Python: Select Interpreter**.
3. Choose the virtual environment for this project.
4. Open a new terminal and retry.

### PyBullet or OpenCV windows do not appear

- Run the program in a local graphical desktop session.
- Check whether the program exited with an error.
- Read the terminal traceback.
- Ensure the environment supports graphical windows.

### Keyboard controls are ignored

- Wait for the warm-up period.
- Confirm that the program is still running.
- Try pressing fewer keys simultaneously.
- Check whether the keyboard supports the required simultaneous key combinations.

### The drone moves differently than expected

- Horizontal movement is relative to the drone's yaw.
- Automatic avoidance may temporarily override manual input.
- Velocity and acceleration limits affect movement.
- Obstacle thresholds determine when avoidance becomes active.

### Package installation fails

Check the complete error output and verify compatibility among Python, NumPy, PyBullet, and the installed `gym-pybullet-drones` release. A clean virtual environment can help avoid dependency conflicts.

## 22. Viva Questions and Answers

### 1. What is the main purpose of this project?

The project demonstrates two-drone control with rule-based collision avoidance in a three-dimensional physics simulation.

### 2. Why are two control modes used?

MANUAL mode allows direct keyboard control. AUTO mode temporarily adjusts movement to respond to potential collision threats.

### 3. How does the system predict a possible collision?

It calculates relative position and relative velocity, then estimates the time of closest approach under a constant-relative-velocity assumption.

### 4. What is relative position?

Relative position is the obstacle's position minus the drone's position. It describes the obstacle's location relative to the drone.

### 5. What is relative velocity?

Relative velocity is the obstacle's velocity minus the drone's velocity. It describes their motion relative to each other.

### 6. What does CPA mean?

CPA stands for **Closest Point of Approach**. It estimates the time when the relative distance is smallest within the prediction horizon.

### 7. Why calculate surface gap instead of only centre distance?

Centre distance measures the separation between object centres. Subtracting the configured radii gives an approximate surface-to-surface clearance.

### 8. Why is avoidance strength between zero and one?

It provides a normalized response. The strength is zero outside the configured avoidance distance and increases as the effective gap decreases.

### 9. Why does the program remember the passing side?

Remembering the side helps reduce repeated switching between left and right when small changes occur in the relative geometry.

### 10. How does the system avoid the other drone?

Each drone includes the other drone's position and velocity in its obstacle list and applies the same relative-motion assessment.

### 11. What is the role of PID control?

PID-based control converts the desired target state and current drone state into motor RPM commands.

### 12. Why is velocity smoothing necessary?

It limits sudden changes in commanded velocity, helping produce more gradual movement. Emergency behavior uses a higher configured acceleration limit.

### 13. Is this a machine-learning project?

No. The current implementation uses mathematical equations, geometric calculations, and programmed rules rather than a trained machine-learning model.

### 14. What is the main limitation of CPA prediction?

It assumes constant relative velocity during the prediction horizon. Sudden changes in drone or obstacle motion can make the prediction inaccurate.

### 15. Does the system guarantee collision-free movement?

No. It is an educational simulation with simplified geometry and heuristic avoidance rules. It does not provide a formal collision-free guarantee.

### 16. What is the difference between obstacle avoidance and PID control?

Obstacle avoidance decides the desired movement direction and target. PID-based control translates the target state into motor commands.

### 17. Why are the physics and control frequencies different?

The physics simulation runs at 240 Hz, while the control logic runs at 48 Hz. This allows physics integration to occur more frequently than control updates.

### 18. What are the main future improvements?

Possible improvements include accurate obstacle geometry, collision statistics, repeatable benchmarks, alternative avoidance algorithms, and a real-time monitoring dashboard.

## 23. Conclusion

The Two-Drone Collision Avoidance Simulation combines manual keyboard control, relative-motion analysis, closest-approach prediction, geometric obstacle avoidance, velocity smoothing, PID-based motor control, and 3D physics simulation.

The most important concept is that the system does not only consider **where an obstacle is currently located**. It also considers **how the drone and obstacle are moving relative to one another**.

Using these values, the program estimates a potential closest approach and generates an avoidance response. The target movement is then passed to a PID-based controller, which produces motor RPM commands for the physics simulator.

This project provides a practical foundation for understanding multi-drone coordination, collision-risk estimation, hybrid control systems, and autonomous navigation concepts.

---

**Project type:** Python / Robotics Simulation / Collision Avoidance

**Environment:** PyBullet 3D simulation

**Control approach:** Manual keyboard control + rule-based automatic avoidance

**Low-level control:** PID-based drone controller

**Machine learning:** Not used in the current implementation
