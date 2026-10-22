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

## 🚨 Problem Statement

Manual UAV operation provides human flexibility, but it depends heavily on
the operator's reaction time. When the pilot commands the UAV toward an
obstacle, continuing the same command may result in a collision.

On the other hand, a completely autonomous controller removes the human
operator from normal decision-making.

Therefore, there is a need for a **shared-control mechanism** that:

- Continuously monitors the UAV's surrounding environment.
- Detects obstacles that pose a critical risk.
- Determines when human control may result in a collision.
- Temporarily overrides unsafe forward motion.
- Performs local obstacle avoidance.
- Maintains stable UAV movement during autonomous intervention.
- Automatically returns control to the human operator after the obstacle
  has been cleared.

The main challenge is to perform this autonomous intervention **without
causing abrupt or unsafe UAV movements**.

---

## 🎯 Objectives

The main objectives of this project are:

1. **Develop a UAV environment for human-controlled flight.**

2. **Continuously monitor the UAV's surrounding environment.**

3. **Identify obstacles that threaten the UAV's intended flight path.**

4. **Determine when human control may result in a collision.**

5. **Automatically activate autonomous safety assistance during critical
   situations.**

6. **Select a safe direction for obstacle avoidance.**

7. **Execute a local collision-free avoidance maneuver.**

8. **Maintain stable UAV motion during autonomous intervention.**

9. **Return control to the human operator after the obstacle is cleared.**

10. **Evaluate the effectiveness and safety of the proposed shared-control
    mechanism.**

---

## 🛠️ Key Idea

The proposed system follows a simple shared-control principle:

**Human Control → Obstacle Detection → Risk Assessment → Autonomous Takeover
→ Safe Avoidance → UAV Stabilization → Human Control**

The UAV is therefore not fully autonomous throughout the flight. Instead,
autonomy is activated only when it is required for safety.

---

## 🔄 System Overview

1. The UAV starts in **MANUAL mode**.
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

---

##  Project Overview

This project implements a data-driven optimal shared control system for UAVs. The system:

1. Predicts nonlinear UAV dynamics using **Koopman operators** from human maneuver data.
2. Approximates the optimal controller online using **Actor-Critic reinforcement learning**.
3. Smoothly blends human and autonomous control using an adaptive **shared control mechanism**.
4. Validates across **four simulation platforms**: Gym-PyBullet, MuJoCo, Gazebo, and ArduPilot SITL.

### Core Contributions (from Base Paper)

1. A data-driven optimal shared control method using Koopman operators and RL, learning online without a precise UAV dynamics model.
2. A smooth shared control mechanism that judges cooperative intention and integrates both human and autonomous control inputs via an adaptive parameter.
3. Online optimal controller approximation exploiting human operator experience, validated through Human-in-the-Loop simulations.

---

## 2. Complete Mathematical Formulation

### 2.1 UAV Attitude Dynamics Model

**Equation 1 — Euler-Lagrange Attitude Dynamics:**

$$
M\ddot{\Phi} = -C(\Phi, \dot{\Phi})\dot{\Phi} + \tau + W(\Phi, \dot{\Phi})
$$

Where:
- $M = \text{diag}([J_\phi, J_\theta, J_\psi]) \in \mathbb{R}^{3\times3}$ is the inertial matrix
- $\Phi = [\phi, \theta, \psi]^T \in \mathbb{R}^{3\times1}$ is the roll, pitch, yaw angle vector
- $C(\Phi, \dot{\Phi}): \mathbb{R}^{6\times1} \to \mathbb{R}^{3\times3}$ is the coupled Coriolis term
- $\tau = [\gamma_\phi, \gamma_\theta, \gamma_\psi]^T \in \mathbb{R}^{3\times1}$ is the input torque
- $W(\Phi, \dot{\Phi})$ is the uncertain disturbance

The input torques are generated by:
- $\gamma_\phi = \alpha_l \alpha_w u_\phi$, where $u_\phi = \omega_1^2 - \omega_3^2$
- $\gamma_\theta = \alpha_l \alpha_w u_\theta$, where $u_\theta = \omega_2^2 - \omega_1^2 + \omega_3^2 - \omega_2^2 - \omega_4^2$
- $\gamma_\psi = \alpha_\gamma u_\psi$

Where $\alpha_l$ is the distance from center of mass to each rotor, $\alpha_w$ is the thrust factor, $\alpha_\gamma$ is the drag factor, and $\omega_j$ ($j=1,...,4$) is the speed of the $j$-th rotor.

**Equation 2 — Reformulated Dynamics:**

Define $B_\gamma = M^{-1} \times \text{diag}([\alpha_l \alpha_w, \alpha_l \alpha_w, \alpha_\gamma]) \in \mathbb{R}^{3\times3}$:

$$
\ddot{\Phi} = -M^{-1}(C(\Phi, \dot{\Phi})\dot{\Phi}) + B_\gamma U + \Omega(\Phi, \dot{\Phi})
$$

Where $\Omega(\Phi, \dot{\Phi}) = M^{-1} \times W(\Phi, \dot{\Phi})$ is the transformed uncertain disturbance.

**Equation 3 — Nonlinear Affine-Input Form:**

Define state $x = [\Phi^T, \dot{\Phi}^T]^T \in \mathbb{R}^{6\times1}$:

$$
\dot{x} = f(x) + g(x)u + d(x)
$$

Where the drift dynamics matrix $f(x)$, control input matrix $g(x)$, and uncertain disturbance matrix $d(x)$ are defined as:

$$
f = \begin{bmatrix} 0_{3\times3} & I_{3\times3} \\ 0_{3\times3} & -M^{-1}C \end{bmatrix}x, \quad
g = \begin{bmatrix} 0_{3\times3} \\ B_\gamma \end{bmatrix}, \quad
d = \begin{bmatrix} 0_{3\times1} \\ \Omega \end{bmatrix}
$$

**Equation 4 — Tracking Error Dynamics:**

With desired trajectory $x_d = [\Phi_d^T, \dot{\Phi}_d^T]^T$ and tracking error $e = x - x_d$:

$$
\dot{e} = \dot{x} - \dot{x}_d = [f(x) - f_d(x_d)] + g(x)u + d(x)
$$

**Equation 5 — Augmented Dynamics:**

With augmented state $X = [e^T, x_d^T]^T \in \mathbb{R}^{12\times1}$ and augmented control $U = [u^T, 0_{1\times3}]^T \in \mathbb{R}^{6\times1}$:

$$
\dot{X} = F(X) + G(X)U + D(X)
$$

Where the augmented dynamics matrices are defined as:

$$
F(X) = \begin{bmatrix} f(e + x_d) - f_d(x_d) \\ f_d(x_d) \end{bmatrix}
$$

$$
G(X) = \begin{bmatrix} g(e + x_d) & 0_{6\times3} \\ 0_{6\times3} & 0_{6\times3} \end{bmatrix}
$$

$$
D(X) = \begin{bmatrix} d(e + x_d) \\ 0_{6\times1} \end{bmatrix}
$$

The augmented system combines the tracking error dynamics (top half) with the desired trajectory dynamics (bottom half). The desired trajectory evolves autonomously without control input, which is why the bottom row of $G(X)$ is zero.

**Assumption 1:**
1. The drift dynamics $f(x)$ and control input matrix $g(x)$ are Lipschitz continuous with respect to $x$.
2. The uncertain disturbance $D(X)$ is bounded by a known function $L_D(X)$, i.e., $\|D(X)\| \leq L_D(X)$ with $L_D(0) = 0$.

---

### 2.2 Koopman Operator Framework

**Equation 6 — Uncontrolled Nonlinear Dynamics:**

$$
x_{t+1} = \mathcal{F}(x_t)
$$

Where $\mathcal{F}: \mathbb{R}^n \to \mathbb{R}^n$ is the nonlinear drift dynamics.

**Equation 7 — Koopman Operator Definition (Uncontrolled):**

$$
(\mathcal{K}\Psi)(x_{t+1}) = \Psi(\mathcal{F}(x_t))
$$

Where $\Psi: \mathbb{R}^n \to \mathcal{H}$ is the transfer function mapping state-space to Hilbert space.

**Equation 8 — Extended System Dynamics (Controlled):**

With extended state $\chi_t = [x_t^T, u_t^T]^T$:

$$
\chi_{t+1} = \mathcal{F}(\chi_t) := \begin{bmatrix} \mathcal{F}(x, u(0)) \\ \mathcal{L}u \end{bmatrix}
$$

Where $\mathcal{L}$ is the left shift operator satisfying $(\mathcal{L}u)_i = u_{i+1}$.

**Equation 9 — Koopman Operator (Controlled):**

$$
(\mathcal{K}\Theta)(\chi) = \Theta(\mathcal{F}(\chi))
$$

Where $\Theta: \mathbb{R}^{n+l} \to \mathbb{R}$ is the transfer function for the extended system.

---

### 2.3 Extended Dynamic Mode Decomposition (EDMD)

**Equation 10 — EDMD Approximation:**

$$
\Theta(\chi_{t+1}) = \mathcal{K}^T \Theta(\chi_t) + \varepsilon(\chi_t)
$$

Where $\varepsilon(\chi_t)$ is the approximation error of the Koopman operator.

**Equation 11 — EDMD Objective Function:**

Given collected data $(\chi_j, \chi_{j+1})$, $j = 1, ..., N_\mathcal{K}$:

$$
E_\mathcal{K} = \sum_{j=1}^{N_\mathcal{K}} \|\varepsilon(\chi_j)\|^2 = \sum_{j=1}^{N_\mathcal{K}} \|\Theta(\chi_{j+1}) - K^T \Theta(\chi_j)\|^2
$$

**Equation 12 — Linearized Dynamics Matrices:**

With transformed state data $X_\mathcal{K} = [\Theta(\chi_1), ..., \Theta(\chi_{N_\mathcal{K}-1})]$, output data $Y_\mathcal{K} = [\Theta(\chi_2), ..., \Theta(\chi_{N_\mathcal{K}})]$, and control data $U_\mathcal{K} = [\Theta(u_1), ..., \Theta(u_{N_\mathcal{K}})]$:

$$
[A, B] = Y_\mathcal{K} [X_\mathcal{K}, U_\mathcal{K}]^{\dagger}
$$

Where $\dagger$ denotes the Moore-Penrose pseudo-inverse.

**Equation 13 — Estimated Augmented Dynamics Matrices:**

$$
\hat{F} = \begin{bmatrix} A \times (e + x_d) - f_d(x_d) \\ f_d(x_d) \end{bmatrix}, \quad
\hat{G} = \begin{bmatrix} B & 0_{6\times3} \\ 0_{6\times3} & 0_{6\times3} \end{bmatrix}
$$

---

### 2.4 Optimal Control Formulation

**Equation 14 — Quadratic Cost Function:**

$$
J(X, U) = \int_{t_0}^{\infty} r(X(\tau), U(\tau)) \, d\tau
$$

Where the saturated control input satisfies $-\mu \leq U(t) \leq \mu$.

**Equation 15 — Instantaneous Reward Function:**

$$
r(X, U) = X^T Q X + \Xi(U)
$$

Where $Q \in \mathbb{R}^{n\times n}$ is the positive definite state penalty matrix.

**Equation 16 — Control Input Penalty:**

$$
\Xi(U) = 2R \int_0^U \left(\mu \tanh^{-1}\left(\frac{\zeta_U}{\mu}\right)\right) d\zeta_U
$$

Where $R \in \mathbb{R}^{n\times n}$ is the positive definite control input penalty matrix.

**Equation 17 — Optimal Value Function:**

$$
J^*(X) = \min_{U(\tau) \in \Omega_U} \int_t^{\infty} r(X(\tau), U(\tau)) \, d\tau
$$

Where $\Omega_U \in \mathbb{R}^{m\times1}$ is the admissible set of control input.

**Equation 18 — Hamilton Function:**

$$
H(X, U, \nabla J^{\ast}) = X^T Q X + \Xi(U) + (\nabla J^{\ast})^T (F + GU + D)
$$

Where $\nabla J^{\ast} = \frac{\partial J^{\ast}}{\partial X}$ is the gradient of the optimal value function.

**Equation 19 — Optimal Control Input:**

$$
U^{\ast}(X) = -\mu \tanh\left(\frac{R^{-1} G^T (\nabla J^{\ast}(X))^T}{2\mu}\right)
$$

**Equation 20 — Hamilton-Jacobi-Bellman (HJB) Equation:**

$$
0 = X^T Q X + \Xi(U^{\ast}) + (\nabla J^{\ast})^T (F + GU^{\ast})
$$

This is obtained by combining the optimal control input (Eq. 19) with the Hamilton function (Eq. 18). Solving this HJB equation directly is intractable due to its nonlinearity and high dimensionality, which motivates the use of Actor-Critic reinforcement learning to approximate the optimal value function and control policy online.
---

### 2.5 Shared Control Mechanism ⭐ CORE INNOVATION

**Equation 21 — Shared Control Input:**

$$
\mathfrak{U} = U^* + \alpha U_h
$$

Where:
- $\mathfrak{U}$ is the shared control input applied to the UAV
- $U^*$ is the optimal control input from autonomy
- $U_h$ is the human control input
- $\alpha \in [0, 1]$ is the shared control parameter

**Equation 22 — Adaptive Authority Allocation:**

$$
\alpha = \begin{cases}
0, & \text{if } \eta \geq \beta_1 \\
1, & \text{if } \eta \leq \beta_2 \\
\frac{\eta - \beta_1}{\beta_2 - \beta_1}, & \text{otherwise}
\end{cases}
$$

Where:
- $\eta$ is the angle between the optimal control vector and human control vector
- $\beta_1 = \frac{2\pi}{3}$ (120°) — upper threshold
- $\beta_2 = \frac{\pi}{2}$ (90°) — lower threshold

**Interpretation:**

| Condition | Angle Range | α Value | Control Authority |
|---|---|---|---|
| $\eta \geq \beta_1$ | ≥ 120° | $\alpha = 0$ | Full autonomy (human input rejected) |
| $\eta \leq \beta_2$ | ≤ 90° | $\alpha = 1$ | Full cooperation (human + autonomy) |
| $\beta_2 < \eta < \beta_1$ | 90°–120° | $0 < \alpha < 1$ | Smooth transition zone |

**Remark 2 (Optimality Guarantee):** When $\alpha = 0$, only the optimal control input is applied, achieving global optimality. When $\alpha = 1$ and $\eta = 0$ (human input aligns with optimal input), the shared control achieves optimality despite full human authority.

---

### 2.6 Actor-Critic Approximation

**Equation 23 — Value Function Reconstruction (Critic NN):**

$$
J^*(X) = W_c^T \phi_c(X) + \varepsilon_c(X)
$$

Where $W_c \in \mathbb{R}^{n_{\phi_c} \times 1}$ are the ideal critic NN weights and $\varepsilon_c(X)$ is the reconstruction error.

**Equation 24 — Optimal Control Reconstruction (Actor NN):**

$$
U^*(X) = -\mu \tanh\left(\frac{R^{-1} \hat{G}^T (\nabla \phi_a)^T W_a}{2\mu}\right)
$$

Where $W_a \in \mathbb{R}^{n_{\phi_a} \times 1}$ are the ideal actor NN weights.

**Equation 25 — Estimated Value Function:**

$$
\hat{J}(X) = \hat{W}_c^T \phi_c(X)
$$

Where $\hat{W}_c$ are the estimated critic NN weights.

**Equation 26 — Estimated Optimal Control Input:**

$$
\hat{U}(X) = -\mu \tanh\left(\frac{R^{-1} \hat{G}^T \hat{W}_a^T \phi_a(X)}{2\mu}\right)
$$

Where $\hat{W}_a$ are the estimated actor NN weights.

**Equation 27 — Estimated Shared Control Input:**

$$
\hat{\mathfrak{U}} = \hat{U} + \alpha U_h
$$

**Equation 28 — Shared Control Bellman Error:**

$$
\delta(X, \hat{W}_c, \hat{U}, U_h) = \nabla \hat{J}^T (\hat{F} + \hat{G}\hat{\mathfrak{U}} + D) + r(X, \hat{\mathfrak{U}})
$$

$$
= \hat{W}_c^T \nabla \phi_c (\hat{F} + \hat{G}(\hat{U} + \alpha U_h) + D) + X^T Q X + \Xi(\hat{U} + \alpha U_h)
$$

---

### 2.7 Online Learning Update Laws

**Equation 29 — Critic NN Weight Update Law:**

$$
\dot{\hat{W}}_c = -\frac{k_{c1} \delta \sigma}{(\sigma^T \sigma + 1)^2} - \frac{k_{c2}}{N} \sum_{k=1}^{N} \frac{\delta_k \sigma_k}{((\sigma_k)^T \sigma_k + 1)^2}
$$

Where:
- $k_{c1}, k_{c2} > 0$ are critic learning rates
- $\sigma = \nabla \phi_c^T(X)(F + G\hat{\mathfrak{U}} + D)$ is the regression vector
- $\sigma_k = \nabla \phi_c^T(X_k)(F + G\hat{\mathfrak{U}}_k + D)$ is the $k$-th historical regression vector
- $N$ is the experience replay stack size

**Equation 30 — Actor NN Weight Update Law (Gradient Projection):**

$$
\dot{\hat{W}}_a = \text{Proj}(-k_a F_a (\hat{W}_a - \hat{W}_c))
$$

Where:
- $k_a > 0$ is the actor learning rate
- $F_a \in \mathbb{R}^{n_\phi \times n_\phi}$ is a positive definite matrix
- $\text{Proj}(\cdot)$ is a projection operator ensuring bounded weights

---

### 2.8 Stability Analysis

**Equation 31 — Control Approximation Bound:**

$$
\|U^*(X) - \hat{U}(X)\|^2 \leq \Sigma \tilde{W}_a^T \tilde{W}_a + \Pi_u
$$

**Equation 32 — Bellman Error Decomposition:**

$$
\delta = -\sigma^T \tilde{W}_c + \frac{1}{4} \tilde{W}_a G_\sigma \tilde{W}_a + \Delta(X) + \xi_H
$$

**Equation 33 — Historical Bellman Error:**

$$
\delta_k = -(\sigma_k)^T \tilde{W}_c + \frac{1}{4} \tilde{W}_a G_k^\sigma \tilde{W}_a + \Delta_k(X)
$$

**Equation 34 — UUB Stability Condition:**

$$
\|\mho\| \geq \left(\frac{\Upsilon_{res}}{\lambda_{\min}(\mathcal{M})}\right)^{1/2}
$$

Where $\mho = [X^T, \tilde{W}_c^T, \tilde{W}_a^T]^T$.

**Equation 35 — Lyapunov Function Derivative:**

$$
\dot{\mathcal{L}} = \nabla J^{\ast}(F + G \mho U^{\ast} + D) + \tilde{W}_c^T \dot{\hat{W}}_c^T + \tilde{W}_a^T \dot{\hat{W}}_a^T
$$

Where $\mho = \text{diag}(\mho_1, \mho_2, ..., \mho_m)$ with $\mho_i \in [1, 2]$ is the diagonal coefficient matrix associated with the shared control parameter $\alpha_i$. The shared control input is rewritten as $\hat{\mathfrak{U}}(i) = \hat{U}(i) + \alpha_i U_h(i) \approx \mho_i \hat{U}$.

**Equation 36 — Full Lyapunov Derivative Expansion:**

$$
\dot{\mathcal{L}} = -X^T Q X - \Xi(\mho U^{\ast}) + \tilde{W}_a^T\left(-k_a F_a(\hat{W}_a - \hat{W}_c)\right)
$$

$$
\quad - \tilde{W}_c^T\left(-\frac{k_{c1}\sigma}{\rho}\left(-\sigma^T \tilde{W}_c + \frac{1}{4}\tilde{W}_a^T G_\sigma \tilde{W}_a + \Delta\right)\right)
$$

$$
\quad - \tilde{W}_c^T\left(-\frac{k_{c2}}{N}\sum_{k=1}^{N} \frac{\sigma_k}{\rho_k} \cdot \frac{1}{4}\tilde{W}_a^T G_k^\sigma \tilde{W}_a\right)
$$

$$
\quad - \tilde{W}_c^T\left(-\frac{k_{c2}}{N}\sum_{k=1}^{N} \frac{\sigma_k}{\rho_k}\left(-(\sigma_k)^T \tilde{W}_a + \Delta_k\right)\right)
$$
Where $\rho = (\sigma^T\sigma + 1)^2$, $\rho_k = (\sigma_k^T\sigma_k + 1)^2$, and $\mho = \text{diag}(\mho_1, ..., \mho_m)$ with $\mho_i \in [1, 2]$ is the coefficient associated with the shared control parameter.

This expansion is used in Theorem 1 to prove that the closed-loop system states and NN weight errors are Ultimately Uniformly Bounded (UUB).

**Theorem 1:** Under Assumptions 2-4, the closed-loop system states $X$ and weight errors $[\tilde{W}_c^T, \tilde{W}_a^T]^T$ are **ultimately uniformly bounded (UUB)** when condition (34) is satisfied.

---

### 2.9 Simplified Position Dynamics (HiTL Simulation)

**Equation 37 — Simplified Position Control System:**

Assuming small attitude angles ($\sin\phi \approx \phi$, $\cos\phi \approx 1$, $\sin\theta \approx \theta$, $\cos\theta \approx 1$):

$$
\begin{cases}
\dot{p}_i = v_i, & i \in \{x, y, z\} \\
\dot{v}_x = -g(\phi_d \sin\psi + \theta_d \cos\psi) \\
\dot{v}_y = -g(-\phi_d \cos\psi + \theta_d \sin\psi) \\
\dot{v}_z = g - f/m
\end{cases}
$$

Where $p = [p_x, p_y, p_z]^T$ is position, $v = [v_x, v_y, v_z]^T$ is velocity, $\Theta = [\phi, \theta]^T$ is the desired attitude (human input), and tracking state is $X = [p - p_d, v - v_d]^T$.

**Equation 38 — Basis Functions for HiTL:**

$$
\phi_c = \phi_a = [X(1)X(4), \; X(2)X(5), \; X(1)^3 X(4), \; X(2)^3 X(5)]
$$

---

### 2.10 Performance Metrics

**Equation 39 — Position Smoothness Index (PSI):**

$$
\text{PSI} = \int_T \left\|\frac{d^3 p}{d\tau^3}\right\|^2 d\tau
$$

**Equation 40 — Attitude Smoothness Index (ASI):**

$$
\text{ASI} = \int_T \left\|\frac{d^3 \Theta}{d\tau^3}\right\|^2 d\tau
$$

**Equation 41 — Accumulated Tracking Error (ATE):**

$$
\text{ATE} = \int_T \|p - p_d\|^2 d\tau
$$

**Equation 42 — Accumulated Control Energy (ACE):**

$$
\text{ACE} = \int_T \|U\|^2 d\tau
$$

Where $U$ is the control input vector imposed on the UAV system. This metric measures the total control energy expended during the flight.





## Algorithm 1: Data-Driven Optimal Shared Control of UAVs

```text
Input: Actor-critic weights Ŵ_c, Ŵ_a, learning rates k_ci (i = 1, 2), k_a,
       projection matrices F_a, experience replay stack, Koopman data set
Output: Shared control input applied to UAV

1:  Initialize actor-critic weights Ŵ_c, Ŵ_a, learning rates k_ci (i = 1, 2),
    k_a, and projection matrices F_a
2:  Initialize experience replay stack {𝔘, δ, {𝔘_j, δ_j}_{j=1}^N}
    and Koopman data set {Y_K, X_K, U_K}
3:  while t < T_end do
4:      Collect human control input U_h and system state X
5:      if UAV model is unknown then
6:          Compute transfer function Θ([X^T, U_h^T]^T)
7:          Update Koopman data set with Θ
8:          Estimate dynamics matrices A, B using Eq. (12)
9:          Calculate F̂ and Ĝ using Eq. (13)
10:     end if
11:     Estimate optimal control Û(X) using Eq. (26)
12:     Compute and apply shared control input 𝔘 using Eq. (27)
13:     Calculate Bellman error δ(X, Ŵ_c, Û, U_h) using Eq. (28)
14:     Update experience replay stack with 𝔘 and δ
15:     Update actor-critic weights Ŵ_c and Ŵ_a using Eq. (29) and Eq. (30)
16: end while
```


---

## 4. Simulation Parameters

### 4.1 Example 1: Numerical Simulation

| Parameter | Value |
|---|---|
| Initial state $X_0$ | $0.03[1_3, 0_3]$ |
| Initial critic weights $W_{c0}$ | $0.15(1_9 + \text{rand}(9))$ |
| Initial actor weights $W_{a0}$ | $0.15(1_9 + \text{rand}(9))$ |
| Saturation bound $\mu_{sat}$ | $0.5$ |
| Inertia $\gamma_\phi$ | $0.0211$ kg·m² |
| Inertia $\gamma_\theta$ | $0.0219$ kg·m² |
| Inertia $\gamma_\psi$ | $0.0366$ kg·m² |
| Control gain $B_\gamma$ | $\text{diag}([41, 41, 110])$ |
| Control penalty $R$ | $I_2$ |
| State penalty $Q$ | $I_6$ |
| Critic learning rates $k_{c1}, k_{c2}$ | $2, 1$ |
| Actor learning rate $k_a$ | $1$ |
| Actor projection $F_a$ | $I_6$ |
| Human controller | PD: $U_h = -K_p X - K_d \dot{X}$, $K_p=3$, $K_d=0.5$ |
| Desired trajectory | $X_d = [A_\phi\sin(\omega t), A_\theta\cos(\omega t), A_\psi\sin(\omega t)]^T$ |
| Amplitude $A_\phi = A_\theta = A_\psi$ | $0.1$ |
| Frequency $\omega$ | $0.5$ |
| Simulation time | $10$ s |
| Step size | $0.001$ s |
| ODE Solver | Fourth-order Runge-Kutta |

**Basis functions (Example 1):**

$$
\phi_c = \phi_a = [X(1)^2, X(1)X(4), X(4)^2, X(2)^2, X(2)X(5), X(5)^2, X(3)^2, X(3)X(6), X(6)^2]
$$

**Compared methods:** Proposed, ADP [Ma et al. 2024], MDA [Broad et al. 2020]

**Result:** The proposed method achieves **19-48% RMSE improvement** and lowest control cost.

### 4.2 Example 2: Human-in-the-Loop Simulation

| Parameter | Value |
|---|---|
| Environment | RflySim + MATLAB R23b Simulink |
| Input device | Logitech F310 gamepad |
| Task | Fly through two circles (radius 10m) |
| Circle 1 center | $[250, 75, 100]$ m |
| Circle 2 center | $[500, -75, 100]$ m |
| Circle 1 time $T_{c1}$ | $30$ s |
| Circle 2 time $T_{c2}$ | $60$ s |
| State penalty $Q$ | $\text{diag}([10000, 10000, 10000, 0.1, 0.1, 0.1])$ |
| Control penalty $R$ | $10000 \times I_3$ |
| History stack size $N$ | $30$ |
| Learning rate $\alpha$ | $0.001$ |
| Initial weights $W_{c0} = W_{a0}$ | $2 \times 1_4$ |
| Simulation time | $60$ s |
| Step size | $0.001$ s |
| ODE Solver | Fourth-order Runge-Kutta |
| Joystick resolution | $1/256$ |
| Joystick value range | $[-0.5, 0.5]$ |

**Result:** The proposed method saves **35.65% control cost** compared to human-only control.

### 4.3 Performance Comparison Results

| Method | PSI ↓ | ASI | ATE ↓ | ACE ↓ |
|---|---|---|---|---|
| **Proposed** | **57.74** | 39.16 | **514.72** | **40.73** |
| ADP only | 66.95 | **5.51** | 945.93 | 46.30 |
| Human only | 98.54 | 127.58 | 918.97 | 47.01 |

### 4.4 Cooperation Proportion

| Index | Roll Control | Pitch Control | All Control |
|---|---|---|---|
| Cooperative time (%) | 54.04 | 80.38 | 90.05 |
| Cost of Human Control | 7.52 × 10³ | 1.50 × 10⁴ | 2.25 × 10⁴ |
| Cost of Shared Control | 2.89 × 10³ | 1.16 × 10⁴ | 1.45 × 10⁴ |
| **Control Cost Saving (%)** | **61.52** | **22.68** | **35.65** |

---

## 5. Multi-Simulator Validation Strategy

This project adapts the base paper's MATLAB/RflySim framework to an open-source multi-simulator stack:

| Simulator | Role | Purpose |
|---|---|---|
| **Gym-PyBullet Drones** | RL Training | Lightweight environment for Actor-Critic policy training and maneuver data generation |
| **MuJoCo** | Physics Validation | High-fidelity dynamics, disturbance rejection, robustness testing |
| **Gazebo / ROS** | Sensor Simulation | Camera, depth, LiDAR, obstacle environment rendering |
| **ArduPilot SITL** | Flight Controller Validation | MAVLink telemetry, waypoint missions, failsafes, real autopilot logic |

### Data Flow Across Simulators

Human Maneuver Data
↓
Koopman EDMD (Eq. 12) → Linearized Dynamics [A, B]
↓
Actor-Critic RL (Eq. 26) → Optimal Control U*
↓
Shared Control Allocator (Eq. 21-22) → U_shared
↓
Multi-Simulator Validation
├── PyBullet (RL baseline)
├── MuJoCo (disturbance robustness)
├── Gazebo (perception + obstacles)
└── ArduPilot SITL (real autopilot + MAVLink)


### Simulator-Specific Roles

| Simulator | Data Generated | Validation Focus |
|---|---|---|
| Gym-PyBullet | State transitions, control inputs | RL policy convergence, baseline performance |
| MuJoCo | High-fidelity state dynamics | Koopman model accuracy under disturbances |
| Gazebo | Camera/depth/LiDAR observations | Perception integration, obstacle avoidance |
| ArduPilot SITL | MAVLink telemetry, mission data | Real autopilot behavior, waypoint tracking |

---

## 6. Conceptual Explanation (Team Research)

A detailed explanatory report on the Koopman Operator and EDMD framework for UAV shared control has been prepared by the team. The key concepts are summarized below:

### Koopman Operator Concept

The Koopman operator provides a way to represent nonlinear dynamics through the evolution of observable functions:

$$
z_k = \Theta(x_k)
$$

Where $\Theta$ is a vector of observable functions that "lift" the original state into a higher-dimensional space. In this lifted space, the dynamics become approximately linear:

$$
z_{k+1} \approx Az_k + Bu_k
$$

### EDMD Workflow
Flight Data → Observable Transformation → EDMD → Koopman Model



The EDMD algorithm:
1. Collects state transitions $(x_k, u_k, x_{k+1})$
2. Applies observable functions to lift the state: $z_k = \Theta(x_k)$
3. Constructs data matrices $X_K$, $Y_K$, $U_K$
4. Computes the pseudo-inverse to estimate $[A, B]$
5. Validates prediction accuracy using MAE and RMSE

### Key Insight
    Koopman → Representation
EDMD → Learning Dynamics
Actor-Critic → Decision
Shared Control → Cooperation
ArduPilot/PX4 → Flight Control
YOLO → Perception


### Complete System Architecture

Camera → Perception → Decision Engine → AI Controller → Shared Control → ArduPilot → UAV → State Measurement → EDMD/Koopman

## 7. Repository Structure

## 7. Repository Structure

```text
CD6_AI-drone-copilot/
│
├── docs/
│   ├── base_paper_summary.md
│   └── team_research_report.md
│
├── architecture_diagrams/
│
├── src/
│   ├── koopman/
│   │   ├── edmd.py
│   │   └── observable_functions.py
│   │
│   ├── dynamics_estimator.py
│   │
│   ├── rl/
│   │   ├── actor_critic.py
│   │   ├── bellman_error.py
│   │   └── replay_buffer.py
│   │
│   ├── shared_control/
│   │   ├── authority_allocation.py
│   │   └── shared_controller.py
│   │
│   ├── ardupilot_bridge/
│   │
│   └── telemetry_reader.py
│
├── visualization/
│   └── plot_results.py
│
├── simulation/
│   ├── pybullet/
│   └── mujoco/
│
├── gazebo/
│   └── ardupilot_sitl/
│
├── data/
│   ├── human_maneuvers/
│   └── koopman_datasets/
│
├── telemetry_logs/
│
├── models/
│   ├── koopman_matrices/
│   ├── actor_weights/
│   └── critic_weights/
│
├── notebooks/
│   ├── edmd_demo.ipynb
│   └── shared_control_demo.ipynb
│
├── tests/
│   ├── test_shared_control.py
│   ├── test_koopman.py
│   └── test_telemetry_reader.py
│
├── requirements.txt
└── README.md
```
## 8. Team Responsibilities

| Member | Simulator / Module | Responsibility |
|---|---|---|
| Member 1 | Gym-PyBullet Drones | RL environment, baseline policy, maneuver data generation |
| Member 2 | MuJoCo | High-fidelity dynamics, disturbance testing, robustness |
| Member 3 | Gazebo / ROS | Sensor simulation, obstacle worlds, perception integration |
| Member 4 | ArduPilot SITL | MAVLink telemetry, mission validation, autopilot integration |

All members contribute to: Koopman model validation, shared control testing, experiment logging, performance comparison, and final report preparation.

---

## 9. Current Project Status

### Completed
- [x] Base paper selected and analyzed (Tan et al., Neurocomputing 2025)
- [x] All mathematical formulations extracted (Eq. 1-42)
- [x] Algorithm 1 documented
- [x] Shared control authority allocation logic implemented
- [x] Multi-simulator workflow designed
- [x] GitHub repository initialized
- [x] Team research report on Koopman/EDMD prepared

### In Progress
- [ ] ArduPilot SITL telemetry bridge (pymavlink)
- [ ] Koopman EDMD dynamics learning module
- [ ] Actor-Critic online learning implementation

### Planned
- [ ] Experience replay stack
- [ ] Gazebo perception integration
- [ ] Full Human-in-the-Loop joystick testing
- [ ] Multi-simulator performance comparison (PSI, ASI, ATE, ACE)

---

pip install numpy matplotlib pymavlink torch
