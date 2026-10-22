p align="center">
  <img src="amrita_logo.jpeg" alt="Amrita Vishwa Vidyapeetham Logo" width="580"/>
</p>

CD6_AI-drone-copilot
👥 Team Members
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

Data-Driven Optimal Shared Control of Unmanned Aerial Vehicles
A Human-in-the-Loop (HiTL) AI Co-Pilot system for UAVs that learns drone dynamics from human maneuver data, approximates an optimal controller online using reinforcement learning, and smoothly blends human pilot control with autonomous optimal control.

Base Paper:
Junkai Tan, Shuangsi Xue, Zihang Guo, Huan Li, Hui Cao, Badong Chen.
"Data-driven optimal shared control of unmanned aerial vehicles."
Neurocomputing, Volume 622, 129428, 2025.
DOI: 10.1016/j.neucom.2025.129428

Affiliation: Xi'an Jiaotong University
📌 Introduction
Unmanned Aerial Vehicles (UAVs) are increasingly used in environments where
safe and reliable navigation is essential. However, fully autonomous UAV
operation can be challenging in dynamic or uncertain environments.

Human operators provide flexibility and decision-making capability during
flight, but continuous manual control may result in delayed reactions when
obstacles appear suddenly.

To address this limitation, this project proposes a Human–Autonomy Shared
Control system for UAV obstacle avoidance. Under normal flight conditions,
the UAV remains under human control. When a critical obstacle is detected,
the autonomous safety controller temporarily takes over to perform a safe
obstacle-avoidance maneuver.

Once the obstacle has been cleared and the UAV becomes stable, control
authority is automatically returned to the human operator.

The system is implemented and evaluated in the GymPyBullet simulation
environment, using a Crazyflie 2.x (CF2X) UAV and a PID-based position
controller.
🚨 Problem Statement
Manual UAV operation provides human flexibility, but it depends heavily on
the operator's reaction time. When the pilot commands the UAV toward an
obstacle, continuing the same command may result in a collision.

On the other hand, a completely autonomous controller removes the human
operator from normal decision-making.

Therefore, there is a need for a shared-control mechanism that:
- Continuously monitors the UAV's surrounding environment.
- Detects obstacles that pose a critical risk.
- Determines when human control may result in a collision.
- Temporarily overrides unsafe forward motion.
- Performs local obstacle avoidance.
- Maintains stable UAV movement during autonomous intervention.
- Automatically returns control to the human operator after the obstacle
  has been cleared.
The main challenge is to perform this autonomous intervention without
causing abrupt or unsafe UAV movements.
🎯 Objectives
The main objectives of this project are:
1. Develop a UAV environment for human-controlled flight.
2. Continuously monitor the UAV's surrounding environment.
3. Identify obstacles that threaten the UAV's intended flight path.
4. Determine when human control may result in a collision.
5. Automatically activate autonomous safety assistance during critical
   situations.
6. Select a safe direction for obstacle avoidance.
7. Execute a local collision-free avoidance maneuver.
8. Maintain stable UAV motion during autonomous intervention.
9. Return control to the human operator after the obstacle is cleared.
10. Evaluate the effectiveness and safety of the proposed shared-control
    mechanism.
🛠️ Key Idea
The proposed system follows a simple shared-control principle:

Human Control → Obstacle Detection → Risk Assessment → Autonomous Takeover
→ Safe Avoidance → UAV Stabilization → Human Control

The UAV is therefore not fully autonomous throughout the flight. Instead,
autonomy is activated only when it is required for safety.
🔄 System Overview
1. The UAV starts in MANUAL mode.
2. The human operator provides flight commands.
3. The environment is continuously monitored for obstacles.
4. The system checks whether an obstacle lies within the UAV's forward
   flight corridor.
5. If there is no critical obstacle, human control continues.
6. If a critical obstacle is detected, autonomous safety control is
   activated.
7. Forward motion is reduced or stopped.
8. The available clearance around the obstacle is evaluated.
9. A safer direction is selected.
10. A local collision-free avoidance path is generated.
11. The UAV follows the selected path using the PID controller.
12. After clearing the obstacle, the UAV is stabilized.
13. Autonomous intervention is terminated.
14. Control is returned to the human operator.
Project Overview
This project implements a data-driven optimal shared control system for UAVs. The system:
1. Predicts nonlinear UAV dynamics using Koopman operators from human maneuver data.
2. Approximates the optimal controller online using Actor-Critic reinforcement learning.
3. Smoothly blends human and autonomous control using an adaptive shared control mechanism.
4. Validates across four simulation platforms: Gym-PyBullet, MuJoCo, Gazebo, and ArduPilot SITL.
Core Contributions (from Base Paper)
1. A data-driven optimal shared control method using Koopman operators and RL, learning online without a precise UAV dynamics model.
2. A smooth shared control mechanism that judges cooperative intention and integrates both human and autonomous control inputs via an adaptive parameter.
3. Online optimal controller approximation exploiting human operator experience, validated through Human-in-the-Loop simulations.
2. Complete Mathematical Formulation
This section describes the mathematical model used by the simulated shared-control drone to generate manual or waypoint movement, represent obstacles, predict possible close encounters, select an avoidance direction, smooth the requested velocity, and send bounded position and yaw references to the low-level flight controller.

Important scope note: The equations below document the high-level controller and obstacle-avoidance implementation described in the derivation PDF. They should not be confused with the Koopman-operator, Actor–Critic, and adaptive shared-control equations described in the base research paper. The simulator's obstacle positions are available directly from the simulation state; camera images are for visualization and are not used to infer obstacle positions.
2.1 Drone State and Control Time Step
At control step \(k\), the controlled drone's position and linear velocity are
\[
p_d[k]=\begin{bmatrix}x_d&y_d&z_d\end{bmatrix}^{T}, \qquad
v_d[k]=\begin{bmatrix}v_{x,d}&v_{y,d}&v_{z,d}\end{bmatrix}^{T}.
\]
Here, \(x_d,y_d,z_d\) are position coordinates in metres, and \(v_{x,d},v_{y,d},v_{z,d}\) are velocity components in metres per second. The subscript \(d\) denotes the drone being controlled, and \(T\) denotes transpose.

The physics engine runs at \(f_p=240\) Hz, while the high-level controller updates at \(f_c=48\) Hz. Therefore, the controller time step is
\[
\Delta t=\frac{1}{f_c}=\frac{1}{48}\approx0.020833\text{ s}.
\]
The simulation time at step \(k\) is
\[
t_k=k\Delta t.
\]
The physics engine may integrate the drone at a finer time resolution, but the high-level controller updates its command once per control step.

The simulator state contains position in entries 0–2, orientation quaternion in entries 3–6, roll, pitch, and yaw in entries 7–9, and linear velocity in entries 10–12. Yaw is represented by \(\psi\), in radians. The high-level equations in this section use position, velocity, and yaw. The imported low-level controller handles attitude tracking and motor response.
2.2 Movement Command Generation
2.2.1 Manual input
Each keyboard key is represented by an indicator that equals 1 while the key is pressed and 0 otherwise. Opposite keys are combined to form signed inputs:
\[
u_f=I_{\mathrm{forward}}-I_{\mathrm{back}},\qquad
u_\ell=I_{\mathrm{left}}-I_{\mathrm{right}},\qquad
u_z=I_{\mathrm{up}}-I_{\mathrm{down}}.
\]
Each indicator is either 0 or 1, so each signed input belongs to \(\{-1,0,1\}\). The inputs represent forward/backward, left/right, and vertical movement.

Because forward and left are body-relative directions, they must be rotated into the world \(x\)-\(y\) coordinate frame using the current yaw angle \(\psi\). The forward and left direction vectors are
\[
f=\begin{bmatrix}\cos\psi\\\sin\psi\end{bmatrix},\qquad
\ell=\begin{bmatrix}-\sin\psi\\\cos\psi\end{bmatrix}.
\]
The unscaled world-frame manual velocity direction is therefore
\[
v_{\mathrm{raw}}=
\begin{bmatrix}
u_f\cos\psi-u_\ell\sin\psi\\
u_f\sin\psi+u_\ell\cos\psi\\
u_z
\end{bmatrix}.
\]
The vertical component is not rotated by yaw. To stop diagonal keyboard input from producing a greater speed than straight input, calculate \(m=\|v_{\mathrm{raw}}\|\). If \(m>1\), divide the vector by \(m\); otherwise, leave it unchanged. The final manual velocity command is
\[
v_{\mathrm{manual}}=V_{\max}v_{\mathrm{raw}}
\begin{cases}
\min(1,1/m),&m>0,\\
1,&m=0,
\end{cases}
\]
where the manual speed limit is \(V_{\max}=0.50\text{ m/s}\). The \(m=0\) case avoids division by zero.
2.2.2 Waypoint-guided input
For autonomous waypoint flight, the baseline movement is directed toward the active waypoint \(p_w\). The goal displacement and its length are
\[
p_g=p_w-p_d,\qquad d_g=\|p_g\|.
\]
When \(d_g>10^{-6}\), the unit vector pointing toward the waypoint is
\[
g=\frac{p_g}{d_g}.
\]
If the drone is effectively at the waypoint and \(d_g\leq10^{-6}\), the implementation uses the fallback direction \((1,0,0)^T\) to avoid division by zero.

After the one-second warm-up, the requested goal velocity is
\[
v_g=g\min(V_c,0.8d_g),
\]
where \(V_c=0.50\text{ m/s}\) is the cruise-speed limit. The factor 0.8 is a distance-proportional gain; dimensionally, it has units of inverse seconds. This term reduces the requested speed as the drone approaches the waypoint. During warm-up, \(v_g=0\).

The baseline velocity \(v_b\) refers to \(v_{\mathrm{manual}}\) in manual piloting and \(v_g\) in waypoint flight.
2.3 Obstacle Position, Velocity, and Effective Radius
Each obstacle is represented by its current position \(p_o\), velocity \(v_o\), and effective radius \(r_o\). For a static obstacle, its position is constant and
\[
v_o=0.
\]
The pillar is treated as a cylinder. A box is approximated by a sphere for the avoidance calculation. For a box half-size of \(0.30\text{ m}\), the effective spherical radius is
\[
r_{\mathrm{box}}=1.8(0.30)=0.54\text{ m}.
\]
This is a conservative approximation used by the avoidance logic; it is not the exact distance to the box faces.
2.3.1 Patrolling sphere
The patrolling sphere moves sinusoidally along one horizontal axis. Its position and velocity are
\[
p_o(t)=\begin{bmatrix}1\\A\sin(\omega t)\\1\end{bmatrix},\qquad
v_o(t)=\frac{dp_o}{dt}=
\begin{bmatrix}0\\A\omega\cos(\omega t)\\0\end{bmatrix}.
\]
The oscillation amplitude is \(A=2.0\text{ m}\) and angular frequency is \(\omega=0.35\text{ rad/s}\). Its maximum speed is
\[
\|v_o\|_{\max}=A\omega=2.0(0.35)=0.70\text{ m/s}.
\]
2.3.2 Orbiting sphere
The orbiting sphere follows a circular path:
\[
p_o(t)=
\begin{bmatrix}
c_x+r\cos(\omega t)\\
c_y+r\sin(\omega t)\\
z_0
\end{bmatrix},\qquad
v_o(t)=
\begin{bmatrix}
-r\omega\sin(\omega t)\\
r\omega\cos(\omega t)\\
0
\end{bmatrix}.
\]
The orbit centre is \((c_x,c_y)=(-1.5,1.0)\text{ m}\), radius \(r=1.2\text{ m}\), angular frequency \(\omega=0.40\text{ rad/s}\), and height \(z_0=1.2\text{ m}\). The speed is constant:
\[
\|v_o\|=r\omega=1.2(0.40)=0.48\text{ m/s}.
\]
2.3.3 Vertically moving sphere
The vertically moving sphere is described by
\[
p_o(t)=
\begin{bmatrix}
2.5\\y_0\\z_0+A\sin(\omega t)
\end{bmatrix},\qquad
v_o(t)=
\begin{bmatrix}
0\\0\\A\omega\cos(\omega t)
\end{bmatrix}.
\]
Its parameters are \(z_0=1.2\text{ m}\), \(A=0.6\text{ m}\), and \(\omega=0.80\text{ rad/s}\). Its altitude ranges from \(0.6\text{ m}\) to \(1.8\text{ m}\), and its maximum vertical speed is
\[
A\omega=0.6(0.80)=0.48\text{ m/s}.
\]
The fixed lateral coordinate \(y_0\) is a scenario parameter.
2.3.4 Another drone as an obstacle
In a two-drone simulation, the other drone is added to the obstacle list. Its position and velocity become \(p_o\) and \(v_o\) for the currently controlled drone. The other drone's effective radius is \(r_o=0.12\text{ m}\), while the controlled drone's assumed radius is \(r_d=0.10\text{ m}\). The same relative-motion equations are therefore used for static obstacles, moving spheres, and other drones.
2.4 Relative Position, Distance, and Clearance
The implementation uses different distance calculations for a cylindrical pillar and for spherical obstacles. Define a component-wise mask
\[
m=\begin{cases}
(1,1,0)^T,&\text{for a cylindrical pillar},\\
(1,1,1)^T,&\text{for a sphere or another drone}.
\end{cases}
\]
The symbol \(\odot\) means component-wise multiplication. The relative position from the controlled drone to an obstacle and relative velocity are
\[
r=(p_o-p_d)\odot m,\qquad
u=(v_o-v_d)\odot m.
\]
Thus, \(r\) points from the drone toward the obstacle, and \(u\) describes how this separation vector changes under the current velocities. For a pillar, the vertical component is ignored and only horizontal separation is considered. For a spherical obstacle, all three dimensions are included.

The current centre-to-centre distance is
\[
d=\|r\|=\sqrt{r_x^2+r_y^2+r_z^2}.
\]
The surface-to-surface clearance, or gap, is
\[
g=d-r_o-r_d,
\]
where \(r_o\) is the obstacle's effective radius and \(r_d=0.10\text{ m}\) is the assumed drone radius.

Interpretation of the gap:
- \(g>0\): a gap remains between the effective shapes.
- \(g=0\): the effective shapes are touching.
- \(g<0\): the effective shapes overlap under the chosen approximation.
For all obstacles \(j\), the smallest current gap is
\[
g_{\min}=\min_j g_j.
\]
The nearest-gap value is calculated before applying the prediction-range check. Detailed closest-approach prediction is skipped for an obstacle if its current gap exceeds \(R_p=2.5\text{ m}\).
2.5 Time of Closest Approach (CPA)
The controller estimates whether the drone and an obstacle will approach one another during a prediction horizon of \(H=2.0\text{ s}\). Over this short interval, relative velocity is assumed constant.

At future time offset \(\tau\), the predicted relative position is
\[
r(\tau)=r+u\tau.
\]
The predicted squared centre distance is
\[
D(\tau)=\|r+u\tau\|^2=(r+u\tau)^T(r+u\tau).
\]
Expanding the product gives
\[
D(\tau)=r^Tr+2\tau r^Tu+\tau^2u^Tu.
\]
The closest point along the constant-relative-velocity trajectory occurs where the derivative of \(D(\tau)\) is zero:
\[
\frac{dD}{d\tau}=2r^Tu+2\tau u^Tu=0.
\]
Rearranging gives the unconstrained time of closest approach:
\[
\tau^*=-\frac{r^Tu}{u^Tu}=-\frac{r\cdot u}{\|u\|^2}.
\]
The dot product \(r\cdot u\) indicates whether the separation is closing or opening. If it is negative, the unconstrained closest point lies in the future. If it is positive, the unconstrained closest point lies in the past, so the closest future point is the current instant.

To avoid division by a near-zero relative speed, the implementation uses
\[
\tau_{\mathrm{CPA}}=
\begin{cases}
\operatorname{clip}\left(-\dfrac{r\cdot u}{\|u\|^2},0,H\right),
&\|u\|^2>10^{-6},\\
0,&\text{otherwise}.
\end{cases}
\]
Here, \(\operatorname{clip}(x,a,b)\) limits \(x\) to the interval \([a,b]\). The predicted relative position at closest approach and its predicted gap are
\[
q=r+u\tau_{\mathrm{CPA}},\qquad
g_p=\|q\|-r_o-r_d.
\]
The effective gap for threat assessment is the smaller of the current and predicted gaps:
\[
g_e=\min(g,g_p).
\]
This means avoidance can begin because an obstacle is already close or because the prediction indicates that the future pass may be closer than the current separation.
2.6 Threat Strength and Escape Direction
2.6.1 Threat-strength calculation
Avoidance strength starts increasing when the effective gap falls below \(D_a=1.5\text{ m}\). For obstacle \(j\), its strength is
\[
\alpha_j=\operatorname{clip}\left(\frac{D_a-g_{e,j}}{D_a},0,1\right).
\]
The value \(\alpha_j\) is dimensionless and lies between 0 and 1.
- If \(g_{e,j}\geq D_a\), then \(\alpha_j=0\): the obstacle contributes no avoidance direction.
- If the effective gap is between 0 and \(D_a\), the strength increases linearly as the gap becomes smaller.
- If the effective gap is zero or negative, the strength is clipped to 1.
A threat is considered active if at least one obstacle has \(\alpha_j>0\). The maximum active strength is
\[
\alpha_{\max}=\max_j\alpha_j.
\]
2.6.2 Direction away from the predicted close pass
The first avoidance direction points away from the predicted closest-approach position. Since \(q\) is the predicted relative vector from the drone to the obstacle, the vector pointing away from it is \(-q\).

If the predicted vector is nearly zero, the implementation uses the current relative vector instead. If that vector is also nearly zero, a fallback direction is used to avoid division by zero. Let the selected nonzero vector be \(a_0\). Its unit direction is
\[
a=\frac{a_0}{\|a_0\|}.
\]
The vertical component is then halved, \(a_z\leftarrow0.5a_z\), so that the escape response is more strongly horizontal. The direction is normalized again after combination with the lateral direction.
2.6.3 Selecting a lateral passing direction
A purely radial escape direction could make the drone move directly backward and remain near an obstacle. A sideways component is therefore added.

Let \(h=(h_x,h_y,0)^T\) be the horizontal travel or heading direction, normalized in the horizontal plane. Its left direction is
\[
\ell_h=(-h_y,h_x,0)^T.
\]
The signed lateral offset of the obstacle is
\[
\delta=r^T\ell_h.
\]
The side-selection rule is:
- If \(\delta>0.05\text{ m}\), the obstacle is on the left, so select the right-hand passing direction \(-\ell_h\).
- If \(\delta<-0.05\text{ m}\), the obstacle is on the right, so select the left-hand passing direction \(+\ell_h\).
- If \(|\delta|\leq0.05\text{ m}\), treat the obstacle as approximately ahead and choose the right-hand side.
For the committed-side versions, this lateral choice is stored while the obstacle remains active to prevent repeated switching between sides.

Let \(s_j\) be the stored unit lateral direction for obstacle \(j\). The combined unit avoidance direction is
\[
d_j=\frac{a_j+0.9s_j}{\|a_j+0.9s_j\|}.
\]
The coefficient 0.9 determines the relative contribution of the lateral direction.

In the single-drone version, the lateral side is instead selected using a horizontal tangent vector
\[
t=(-a_y,a_x,0)^T,\qquad
s_j=\begin{cases}
1,&t^Tv_d\geq0,\\
-1,&t^Tv_d<0.
\end{cases}
\]
The resulting direction in that version is
\[
d_j=\frac{a_j+0.5s_jt}{\|a_j+0.5s_jt\|}.
\]
Both versions implement the same principle: combine a direction away from the predicted close pass with a horizontal sideways component. The precise side-selection rule is implementation-dependent; the weighted aggregation described next is common to the method.
2.6.4 Combining multiple obstacle directions
For multiple obstacles, each direction is weighted by its threat strength and summed:
\[
e=\sum_j\alpha_jd_j.
\]
The vector \(e\) is the aggregate escape vector. Obstacles with zero strength contribute nothing. If several obstacles push in similar directions, \(\|e\|\) can exceed 1; if they push in opposing directions, their contributions may partially cancel. Therefore, \(e\) is not itself a physical velocity. It represents the combined avoidance direction and intensity used to construct the next velocity request.
2.7 Combining Normal Movement with Avoidance
The baseline velocity is the normalized manual command when the pilot is controlling the drone, or the waypoint velocity during autonomous waypoint flight. The avoidance calculation modifies this baseline according to the active control mode.
2.7.1 Manual control with automatic avoidance
For a manually controlled drone in AUTO mode, when a threat is active and \(\|e\|>10^{-6}\), the desired velocity points along the aggregate escape vector:
\[
v_{\mathrm{des}}=
\begin{cases}
V_e\dfrac{e}{\|e\|},&g_{\min}<D_e,\\[6pt]
V_a\min(1,\|e\|)\dfrac{e}{\|e\|},&\text{otherwise}.
\end{cases}
\]
The parameters are:
- \(D_e=0.65\text{ m}\): emergency gap.
- \(V_a=0.60\text{ m/s}\): normal avoidance speed.
- \(V_e=0.90\text{ m/s}\): emergency avoidance speed.
When AUTO is active but there is no usable escape vector, the desired velocity is set to zero, requesting braking or hovering. In MANUAL mode, the desired velocity remains the baseline manual command except when emergency avoidance overrides it.
2.7.2 Autonomous waypoint flight
For waypoint flight, the controller reduces the goal-directed velocity as threat strength increases and adds the escape contribution:
\[
v_{\mathrm{des}}=(1-0.7\alpha_{\max})v_g+V_ae.
\]
The coefficient 0.7 determines how strongly the goal-directed component is reduced. The resulting velocity is limited to \(V_a\) during normal avoidance and \(V_e\) during an emergency. If no threat exists, the desired velocity is simply
\[
v_{\mathrm{des}}=v_g.
\]
The intended trade-off is to continue moving toward the waypoint when the route is clear, while allowing avoidance to have increasing influence when a close pass is predicted.

The emergency flag is determined by the nearest current gap:
\[
\text{emergency}=(g_{\min}<D_e).
\]
It is not directly determined by \(g_e\). Therefore, a predicted future conflict can activate avoidance before the current gap becomes small enough to trigger emergency mode.
2.8 Limiting the Change in Commanded Velocity
A desired velocity may change abruptly when a key is pressed or an obstacle enters the avoidance range. To avoid a sudden jump in the command, the controller applies a velocity slew limiter.

Let \(v_c[k]\) be the current smoothed velocity command and \(v_{\mathrm{des}}[k]\) be the new desired velocity. Their difference and its magnitude are
\[
\Delta v=v_{\mathrm{des}}[k]-v_c[k],\qquad
d_v=\|\Delta v\|.
\]
For acceleration limit \(a_{\mathrm{lim}}\), the maximum allowed velocity change in one control step is
\[
\Delta v_{\max}=a_{\mathrm{lim}}\Delta t.
\]
The next command is
\[
v_c[k+1]=
\begin{cases}
v_c[k]+\dfrac{\Delta v}{d_v}\Delta v_{\max},
&d_v>\Delta v_{\max},\\[6pt]
v_{\mathrm{des}}[k],
&d_v\leq\Delta v_{\max}.
\end{cases}
\]
When the difference is too large, the controller scales the entire difference vector. This preserves the direction of the requested change while limiting its magnitude. Consequently,
\[
\|v_c[k+1]-v_c[k]\|\leq a_{\mathrm{lim}}\Delta t.
\]
The normal acceleration limit is \(a_{\mathrm{lim}}=1.0\text{ m/s}^2\), while emergency avoidance uses \(a_{\mathrm{lim}}=2.2\text{ m/s}^2\). At 48 Hz, the maximum command changes are approximately
\[
1.0\left(\frac{1}{48}\right)=0.02083\text{ m/s}
\]
for normal movement, and
\[
2.2\left(\frac{1}{48}\right)\approx0.04583\text{ m/s}
\]
for emergency avoidance per update. This bounds the commanded velocity change; it does not prove that the physical drone's acceleration is exactly equal to this value.
2.9 Position Reference and Target Leash
The smoothed velocity is integrated to update the target position given to the low-level controller:
\[
p_T^{\mathrm{raw}}[k+1]=p_T[k]+v_c[k+1]\Delta t.
\]
Here, \(p_T\) is the target position, not the measured drone position. This is forward Euler integration: velocity multiplied by the time step gives the target displacement during one update.

The target is constrained to the simulation arena:
\[
x_T\in[-X_{\mathrm{lim}},X_{\mathrm{lim}}],\qquad
y_T\in[-X_{\mathrm{lim}},X_{\mathrm{lim}}],\qquad
z_T\in[Z_{\min},Z_{\max}],
\]
where
\[
X_{\mathrm{lim}}=4.5\text{ m},\qquad
Z_{\min}=0.35\text{ m},\qquad
Z_{\max}=2.5\text{ m}.
\]
Each coordinate is clipped independently to its permitted interval.

The target is then kept within a distance \(L=0.30\text{ m}\) of the actual drone position. Define
\[
e_T=p_T^{\mathrm{raw}}-p_d,\qquad d_T=\|e_T\|.
\]
The final target is
\[
p_T=
\begin{cases}
p_d+\dfrac{e_T}{d_T}L,&d_T>L,\\[6pt]
p_T^{\mathrm{raw}},&d_T\leq L.
\end{cases}
\]
The direction from the actual position to the target is retained, but its length is shortened if it exceeds \(L\). This target leash prevents the reference from moving too far ahead of a drone that cannot follow the command immediately. When automatic control hands control back to the pilot, the target position is reset to the measured drone position so that the next reference begins at the drone's current location.
2.10 Yaw Reference
Yaw handling is separate from the translational obstacle-avoidance calculation.

For manual flight, the target yaw may be held at a fixed reference. In autonomous waypoint flight, the target yaw follows the horizontal direction of travel when the commanded horizontal speed is sufficiently large:
\[
\psi_{\mathrm{want}}=\operatorname{atan2}(v_{c,y},v_{c,x}).
\]
The function \(\operatorname{atan2}(y,x)\) returns the angle of a two-dimensional vector while correctly identifying its quadrant.

The yaw error is wrapped to the interval \([-\pi,\pi)\):
\[
e_\psi=\operatorname{wrap}(\psi_{\mathrm{want}}-\psi_T),
\]
where
\[
\operatorname{wrap}(a)=(a+\pi)\bmod 2\pi-\pi.
\]
The target yaw is updated using a limited step:
\[
\Delta\psi=\operatorname{clip}(e_\psi,-\omega_\psi\Delta t,\omega_\psi\Delta t),
\]
\[
\psi_T[k+1]=\operatorname{wrap}(\psi_T[k]+\Delta\psi),
\]
with yaw-reference rate limit \(\omega_\psi=0.8\text{ rad/s}\).

For pilot-controlled yaw, the target yaw changes according to the left/right yaw input \(u_\psi\in\{-1,0,1\}\):
\[
\psi_T[k+1]=\operatorname{wrap}\left(\psi_T[k]+u_\psi\omega_\psi\Delta t\right).
\]
In the described version, the target yaw is retained during automatic avoidance.
2.11 Waypoint and Automatic-Control Timing
2.11.1 Waypoint advancement
During waypoint flight, the current waypoint index advances when either the drone comes within \(R_{\mathrm{arrive}}=0.6\text{ m}\) of the waypoint or the waypoint timer exceeds \(T_{\mathrm{waypoint}}=12\text{ s}\):
\[
d_g<R_{\mathrm{arrive}}\quad\text{or}\quad \tau_w>T_{\mathrm{waypoint}}.
\]
The waypoint index advances cyclically:
\[
j\leftarrow(j+1)\bmod N_w,
\]
where \(N_w\) is the number of waypoints in the mission. The timer resets after switching. The timeout prevents the drone from remaining at an unreachable waypoint indefinitely; it does not mean the waypoint was physically reached.
2.11.2 MANUAL/AUTO mode switching
Each controlled drone has a mode
\[
m\in\{\mathrm{MANUAL},\mathrm{AUTO}\}.
\]
AUTO can begin when a threat exists and either the gap is in the emergency range or the pilot has released the relevant keys and the grace period has expired. The emergency test is
\[
g_{\min}<D_e.
\]
The grace timer decreases by the control time step:
\[
\tau_g[k+1]=\max(0,\tau_g[k]-\Delta t).
\]
After control is handed back to the pilot, the grace period is set to \(1.5\text{ s}\). This helps prevent a non-emergency threat from immediately triggering AUTO again.

While AUTO is active, its timer increases by \(\Delta t\). The clear-path timer resets to zero whenever a threat remains and increases by \(\Delta t\) while the path is clear. Control can return to MANUAL when any applicable release condition is met:
- The path has remained clear for \(T_{\mathrm{release}}=0.8\text{ s}\).
- The pilot presses a key outside an emergency.
- AUTO reaches \(T_{\mathrm{auto}}=3.0\text{ s}\) outside an emergency.
The emergency condition prevents normal pilot override and timeout release. In multi-UAV operation, these mode and timer variables are maintained independently for each vehicle, so one drone may be in AUTO while another remains in MANUAL.
2.12 Connection to the Low-Level Controller
The target position \(p_T\), target yaw \(\psi_T\), and smoothed velocity reference \(v_c\) are passed to the imported DSLPIDControl object. The simulator's measured drone position and velocity are fed back to the control system.

At a general level, position and velocity tracking errors can be written as
\[
e_p=p_T-p_d,\qquad e_v=v_c-v_d,
\]
where \(e_p\) is position error and \(e_v\) is velocity error.

A generic PID control law is
\[
u(t)=K_Pe(t)+K_I\int_0^t e(\tau)\,d\tau+K_D\frac{de(t)}{dt},
\]
where \(e(t)\) is a tracking error, \(K_P\), \(K_I\), and \(K_D\) are proportional, integral, and derivative gains, and \(u(t)\) is a controller output.

This equation explains the general PID principle. The available high-level control logic does not expose the imported controller's internal gains or its complete control equations, so those details cannot be determined from the high-level code alone.

The low-level controller returns four motor RPM commands, which are passed to the simulator. The simulator then updates the physical drone state. Thus, the obstacle-avoidance equations generate a reference trajectory; they do not directly calculate the four motor speeds.
2.13 Numerical Checks
These examples illustrate the closest-approach calculation, threat-strength calculation, and velocity slew limiter.
Example 1: Time of closest approach
Suppose the relative position and relative velocity are
\[
r=(2,0,0)^T\text{ m},\qquad
u=(-1,0,0)^T\text{ m/s}.
\]
The obstacle is 2 metres ahead, and the relative motion is closing. The time of closest approach is
\[
\tau_{\mathrm{CPA}}=-\frac{r\cdot u}{\|u\|^2}
=-\frac{(2)(-1)}{(-1)^2}=2\text{ s}.
\]
For prediction horizon \(H=2\text{ s}\), clipping leaves the time at 2 seconds. The predicted relative position is
\[
q=r+u\tau_{\mathrm{CPA}}
=(2,0,0)^T+(-1,0,0)^T(2)
=(0,0,0)^T.
\]
The predicted centre distance is zero. Consequently, the predicted gap is
\[
g_p=-r_o-r_d,
\]
and the threat strength clips to 1, indicating the maximum avoidance strength.
Example 2: Threat strength
If the effective gap is \(g_e=0.90\text{ m}\) and the avoidance activation distance is \(D_a=1.5\text{ m}\), then
\[
\alpha=\operatorname{clip}\left(\frac{1.5-0.90}{1.5},0,1\right)
=\frac{0.60}{1.5}=0.4.
\]
This obstacle contributes \(0.4d\) to the aggregate escape vector, where \(d\) is its unit avoidance direction.
Example 3: Velocity slew limiter
Suppose the current smoothed command is zero and the desired command is
\[
v_{\mathrm{des}}=(0.5,0,0)^T\text{ m/s}.
\]
With \(a_{\mathrm{lim}}=1.0\text{ m/s}^2\) and \(\Delta t=1/48\text{ s}\),
\[
\Delta v_{\max}=a_{\mathrm{lim}}\Delta t
=1.0\left(\frac{1}{48}\right)
\approx0.02083\text{ m/s}.
\]
Instead of immediately jumping to \(0.5\text{ m/s}\), the next command is approximately
\[
v_c[k+1]=(0.02083,0,0)^T\text{ m/s}.
\]
This demonstrates how the controller limits sudden changes in the commanded velocity.
2.14 Meaning, Assumptions, and Limitations
The mathematical formulation covers:
1. Generation of manual or waypoint movement commands.
2. Kinematics of static and moving obstacles.
3. Relative position, relative velocity, and geometric clearance.
4. Prediction of the time and position of closest approach.
5. Threat-strength calculation and construction of escape directions.
6. Combination of multiple obstacle-avoidance directions.
7. Combination of baseline movement with avoidance.
8. Velocity smoothing using an acceleration-based slew limit.
9. Position-reference integration, arena bounds, and target leash.
10. Yaw-reference handling.
11. Waypoint timing and MANUAL/AUTO handover logic.
12. Connection to the low-level PID-based controller and numerical checks.
The same clearance and closest-approach calculations apply to fixed obstacles, analytically moving obstacles, and other UAVs.

However, this method is an approximation and does not provide a formal guarantee of collision avoidance. The closest-approach prediction assumes constant relative velocity over the prediction horizon, even though the drone can accelerate and moving obstacles may follow curved or oscillating paths. The box is approximated by an effective sphere, while the pillar is assessed using horizontal distance only. In a multi-drone simulation, each drone makes local avoidance decisions rather than jointly solving a trajectory-optimization problem. Finally, camera images are used for visualization; obstacle positions are obtained directly from the simulation and are not inferred from camera pixels.
