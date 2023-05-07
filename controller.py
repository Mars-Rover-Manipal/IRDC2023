import casadi as ca
import matplotlib.pyplot as plt

# Define the MPC parameters
N = 10  # Prediction horizon
dt = 0.1  # Sampling time
nx = 6  # Number of states (position, velocity, and orientation)
nu = 4  # Number of control inputs (flapping angles for each wing)
Q = ca.diag([1] * nx)  # State cost matrix
R = ca.diag([1] * nu)  # Control cost matrix
lbx = [-ca.inf] * nx  # Lower bounds on states
ubx = [ca.inf] * nx  # Upper bounds on states
lbu = [-0.5] * nu  # Lower bounds on control inputs
ubu = [0.5] * nu  # Upper bounds on control inputs

# Define the symbolic variables and model equations
x = ca.SX.sym('x', nx)
u = ca.SX.sym('u', nu)
xdot = f(x, u)  # Model equations
obj = ca.mtimes(x.T, Q, x) + ca.mtimes(u.T, R, u)  # Objective function
g = []  # Constraints

# Define the optimization problem
opti = ca.Opti()
X = opti.variable(nx, N + 1)  # States over the prediction horizon
U = opti.variable(nu, N)  # Control inputs over the prediction horizon
X[:, 0] = x  # Initial state
for k in range(N):
    # Add the model equations as constraints
    opti.subject_to(X[:, k + 1] == X[:, k] + dt * xdot)

    # Add the control input bounds as constraints
    opti.subject_to(lbu <= U[:, k], f"lbu{k}")
    opti.subject_to(U[:, k] <= ubu, f"ubu{k}")

    # Add the state bounds as constraints
    opti.subject_to(lbx <= X[:, k], f"lbx{k}")
    opti.subject_to(X[:, k] <= ubx, f"ubx{k}")

    # Add the wing dynamics model and aerodynamic formulae
    wing_dynamics = [f_wing(X[:, k], U[:, k], i) for i in range(1, 5)]
    lift = [f_lift(X[:, k], U[:, k], i) for i in range(1, 5)]
    drag = [f_drag(X[:, k], U[:, k], i) for i in range(1, 5)]

    # Add the wing dynamics and aerodynamic forces and moments to the objective function
    for i in range(4):
        obj += ca.mtimes(lift[i].T, lift[i]) + ca.mtimes(drag[i].T, drag[i]) + ca.mtimes(wing_dynamics[i].T, wing_dynamics[i])

    # Add the wing deformation and induced drag to the objective function
    deformation = [f_deformation(X[:, k], U[:, k], i) for i in range(1, 5)]
    induced_drag = [f_induced_drag(X[:, k], U[:, k], i) for i in range(1, 5)]

    for i in range(4):
        obj += ca.mtimes(deformation[i].T, deformation[i]) + ca.mtimes(induced_drag[i].T, induced_drag[i])

    # Add the state and control input tracking terms to the objective function
    obj += ca.mtimes((X[:, k + 1] - X[:, k]).T, Q, (X[:, k + 1] - X[:, k])) + ca.mtimes(U[:, k].T, R, U[:, k])

    #Set the solver options and solve the optimization problem

    options = {'ipopt': {'max_iter': 100, 'print_level': 0, 'acceptable_tol': 1e-8, 'acceptable_obj_change_tol': 1e-6}}
    opti.solver('ipopt', options)
    sol = opti.solve()

    #Extract the optimal state and control input trajectories
    x_opt = sol.value(X)
    u_opt = sol.value(U)

    #Plot the optimal state and control input trajectories
    fig, axs = plt.subplots(nx + nu, sharex=True)
    fig.suptitle('Optimal state and control input trajectories')

    for i in range(nx):
        axs[i].plot(x_opt[i, :], label=f'x{i}')
        axs[i].legend()

    for i in range(nu):
        axs[nx + i].plot(u_opt[i, :], label=f'u{i}')
        axs[nx + i].legend()

    plt.show()
