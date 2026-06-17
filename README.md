# optimal_planetary_landing_with_constraints
This is the Python CasADi implementation code for the article titled "Optimal planetary landing with pointing and glide-slope constraints".

# Optimal Control for Constrained Planetary Landing 🚀

This repository contains the numerical implementation and theoretical analysis of a vertical powered descent problem for planetary landing. The project models the optimal trajectory of a lander while strictly adhering to **glide-slope (state) constraints** and **thrust pointing (input) constraints**. 

The continuous optimal control problem is discretized and solved using Python and **CasADi/IPOPT**. The implementation numerically verifies the theoretical derivations from the source paper, successfully reproducing the "Max-Min-Max" (Bang-Bang) optimal control structure.

## 📄 Reference Article

This project is based on the theoretical framework and proofs established in the following research paper:

> **Title:** [Optimal planetary landing with pointing and glide-slope constraints]  
> **Authors:** [Clara Leparoux,  Bruno Hérissé, Frédéric Jean]  
> **Link:** [Read the full article here](https://ieeexplore.ieee.org/document/9992735/) 

## ⚙️ System Dynamics

The vehicle's motion is modeled using point-mass dynamics in a two-dimensional inertial frame.The state vector includes position $(r)$, velocity $(v)$, and mass $(m)$. The continuous-time differential equations governing the flight are:

* **Kinematics:** $\dot{r} = v$ 
* **Kinetics:** $\dot{v} = \frac{T}{m}u - g$ 
    *(where (T) is maximum thrust, (u) is the normalized thrust vector, and (g) is gravity)*
* **Mass Depletion:** $\dot{m} = -q ||u||$ 
    *(where $q$ is the maximum mass flow rate)*

## 📊 Simulation Results

The numerical optimization confirms the mathematical proofs. The solver successfully finds a trajectory that strictly respects the physical boundaries (the $5^\circ$ glide-slope cone and $45^\circ$ pointing limits) while maximizing remaining propellant. 

#### Simulation Parameters & Initial Conditions (Constant Mass)
* **Initial Position:** $(x_0, z_0) = (2000 \text{ m}, 1500 \text{ m})$ 
* **Initial Velocity:** $(v_{x0}, v_{z0}) = (100 \text{ m/s}, -75 \text{ m/s})$ 
* **Initial Mass ($m_0$):** $1905 \text{ kg}$ 
* **Maximum Thrust ($T$):** $16573 \text{ N}$ 
* **Thrust Bounds:** $0.3 \le ||u|| \le 0.8$ 
* **Gravity ($g_0$):** $3.71 \text{ m/s}^2$ (Mars) 
* **Mass Flow Rate ($q$):** $0 \text{ kg/s}$ 

![Simulation Results](assets/result_constant_mass.png)
*Figure: The simulated optimal trajectory (x vs. z), the bang-bang control norm profile (Max-Min-Max), and the saturated thrust pointing direction.*

## 🛠️ Dependency
* **CasADi** (Nonlinear Optimization Framework)
* **IPOPT** (Interior Point Optimizer)
* **Matplotlib** (Visualization)

