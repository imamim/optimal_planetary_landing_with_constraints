import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. PHYSICAL & NUMERICAL PARAMETERS
# ==========================================
T = 16573.0       # Maximum thrust force (N)
m0 = 1905.0       # Constant vehicle mass (kg)
g0 = 3.71         # Mars gravity (m/s^2)
u_min = 0.3       # Minimum throttle limit
u_max = 0.8       # Maximum throttle limit

N = 100           # Number of control intervals (Multiple Shooting nodes)

# Boundary Conditions
r0 = np.array([2000.0, 1500.0])
v0 = np.array([100.0, -75.0])
rf = np.array([0.0, 0.0])
vf = np.array([0.0, 0.0])

# ==========================================
# 2. CONTINUOUS DYNAMICS & INTEGRATOR
# ==========================================
# Define symbolic variables for the integrator
x_sym = ca.MX.sym('x')
z_sym = ca.MX.sym('z')
vx_sym = ca.MX.sym('vx')
vz_sym = ca.MX.sym('vz')
states = ca.vertcat(x_sym, z_sym, vx_sym, vz_sym)
n_states = states.size1()

ux_sym = ca.MX.sym('ux')
uz_sym = ca.MX.sym('uz')
controls = ca.vertcat(ux_sym, uz_sym)
n_controls = controls.size1()

# ODEs
rhs = ca.vertcat(vx_sym, 
                 vz_sym, 
                 (T / m0) * ux_sym, 
                 (T / m0) * uz_sym - g0)

f = ca.Function('f', [states, controls], [rhs])

# Runge-Kutta 4 (RK4) Integrator
dt_sym = ca.MX.sym('dt') 
X_init = ca.MX.sym('X_init', n_states)
U_init = ca.MX.sym('U_init', n_controls)

k1 = f(X_init, U_init)
k2 = f(X_init + dt_sym/2 * k1, U_init)
k3 = f(X_init + dt_sym/2 * k2, U_init)
k4 = f(X_init + dt_sym * k3, U_init)
X_next = X_init + dt_sym/6 * (k1 + 2*k2 + 2*k3 + k4)

F_rk4 = ca.Function('F_rk4', [X_init, U_init, dt_sym], [X_next])

# ==========================================
# 3. OPTIMIZATION FUNCTION
# ==========================================
def solve_landing_scenario(enforce_altitude, enforce_pointing):
    opti = ca.Opti()

    # --- Optimization Variables ---
    X = opti.variable(n_states, N+1) 
    U = opti.variable(n_controls, N) 
    tf = opti.variable()             
    dt_val = tf / N

    # --- Objective & Multiple Shooting Loop ---
    cost = 0
    for k in range(N):
        # 1. Close the gap: X_{k+1} must equal the integrated state from X_k
        x_next_rk4 = F_rk4(X[:, k], U[:, k], dt_val)
        opti.subject_to(X[:, k+1] == x_next_rk4)
        
        # 2. Accumulate cost (Integral of ||u|| dt)
        cost += ca.norm_2(U[:, k]) * dt_val

    opti.minimize(cost)

    # --- Base Constraints ---
    opti.subject_to(tf > 0)
    opti.subject_to(X[:, 0] == ca.vertcat(r0[0], r0[1], v0[0], v0[1]))
    opti.subject_to(X[:, N] == ca.vertcat(rf[0], rf[1], vf[0], vf[1]))

    theta_rad = 45.0 * np.pi / 180.0

    for k in range(N):
        u_norm = ca.norm_2(U[:, k])
        opti.subject_to(u_min <= u_norm)
        opti.subject_to(u_norm <= u_max)
        
        # Scenario-specific constraints
        if enforce_altitude:
            opti.subject_to(X[1, k] >= 0) # z >= 0
            
        if enforce_pointing:
            opti.subject_to(U[1, k] >= u_norm * np.cos(theta_rad))
            
    # Also enforce altitude at the final node if active
    if enforce_altitude:
        opti.subject_to(X[1, N] >= 0)

    # --- Initial Guesses ---
    x_guess = np.linspace(r0[0], rf[0], N+1)
    z_guess = np.linspace(r0[1], rf[1], N+1)
    vx_guess = np.linspace(v0[0], vf[0], N+1)
    vz_guess = np.linspace(v0[1], vf[1], N+1)

    opti.set_initial(X[0, :], x_guess)
    opti.set_initial(X[1, :], z_guess)
    opti.set_initial(X[2, :], vx_guess)
    opti.set_initial(X[3, :], vz_guess)
    opti.set_initial(tf, 80.0)
    opti.set_initial(U[0, :], 0.0)
    opti.set_initial(U[1, :], 0.5)

    # --- Solve ---
    p_opts = {"expand": True}
    # Print level 0 silences IPOPT to keep the terminal clean while looping
    s_opts = {"max_iter": 1000, "tol": 1e-6, "print_level": 0} 
    opti.solver('ipopt', p_opts, s_opts)

    sol = opti.solve()
    
    # --- Data Extraction ---
    tf_opt = sol.value(tf)
    ux_opt = sol.value(U[0, :])
    uz_opt = sol.value(U[1, :])
    
    return {
        "t_states": np.linspace(0, tf_opt, N+1),
        "t_controls": np.linspace(0, tf_opt, N),
        "x": sol.value(X[0, :]),
        "z": sol.value(X[1, :]),
        "u_norm": np.sqrt(ux_opt**2 + uz_opt**2),
        "angle": np.degrees(np.arctan2(ux_opt, uz_opt)),
        "tf": tf_opt
    }

# ==========================================
# 4. EXECUTE SCENARIOS
# ==========================================
scenarios = [
    {"label": "unconstrained", "alt": False, "point": False, "color": "blue"},
    {"label": r"$\gamma = 0^\circ$", "alt": True, "point": False, "color": "red"},
    {"label": r"$\gamma = 0^\circ$ and $\theta = 45^\circ$", "alt": True, "point": True, "color": "green"}
]

results = []

for s in scenarios:
    print(f"Solving scenario: {s['label']}...")
    try:
        data = solve_landing_scenario(enforce_altitude=s['alt'], enforce_pointing=s['point'])
        data['label'] = s['label']
        data['color'] = s['color']
        results.append(data)
        print(f"  -> Success! Optimal Time: {data['tf']:.2f} s")
    except RuntimeError:
        print(f"  -> Solver failed to converge for {s['label']}.")

# ==========================================
# 5. PLOTTING (Replicating Figure 3)
# ==========================================
plt.figure(figsize=(10, 8))

# --- Subplot 1: Trajectory ---
plt.subplot(2, 2, (1, 2))
plt.axhline(0, color='gray', linestyle='--') # Ground line
plt.plot(r0[0], r0[1], 'ko') # Start point
plt.plot(rf[0], rf[1], 'ko') # End point

for res in results:
    plt.plot(res["x"], res["z"], color=res["color"], linewidth=2, label=res["label"])

plt.title('Trajectory')
plt.xlabel('x (m)')
plt.ylabel('z (m)')
plt.grid(True)
plt.legend(loc='lower center')
#plt.gca().invert_xaxis() # Paper plots x descending from 2000 to 0

# --- Subplot 2: Control Norm ---
plt.subplot(2, 2, 3)
plt.axhline(u_max, color='gray', linestyle='--')
plt.axhline(u_min, color='gray', linestyle='--')

for res in results:
    plt.plot(res["t_controls"], res["u_norm"], color=res["color"], linewidth=2)

plt.title('Control norm')
plt.xlabel('t (s)')
plt.ylabel('||u||')
plt.yticks([0.3, 0.4, 0.6, 0.8])
plt.grid(True)

# --- Subplot 3: Thrust Direction ---
plt.subplot(2, 2, 4)
plt.axhline(45, color='gray', linestyle='--')
plt.axhline(-45, color='gray', linestyle='--')

for res in results:
    plt.plot(res["t_controls"], res["angle"], color=res["color"], linewidth=2)

plt.title('Thrust direction')
plt.xlabel('t (s)')
plt.ylabel('angle (°)')
plt.yticks([-45, 0, 45])
plt.grid(True)

plt.tight_layout()
plt.show()