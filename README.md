# bee-controller

This code uses CasADi to implement an MPC (Model Predictive Control) algorithm for a flapping-wing micro air vehicle. The goal of the MPC is to find the optimal flapping angles for each wing, so that the vehicle can fly along a given trajectory while minimizing a cost function that penalizes deviations from the desired state, control inputs, and wing deformation.

The code sets up the MPC problem by defining the prediction horizon (N), sampling time (dt), number of states (nx) and control inputs (nu), and the cost matrices (Q and R) for the state and control input, respectively. The state and control input bounds are also defined.

The model equations are defined symbolically, and the MPC optimization problem is constructed using the Opti class from CasADi. The problem is defined by defining the symbolic variables (states and control inputs over the prediction horizon), the objective function (minimizing the cost function), and the constraints (model equations, control input bounds, and state bounds).

The code then solves the MPC problem using the IPOPT solver, and plots the optimal state and control input trajectories. The solver options are defined in a dictionary, which can be customized if needed.

To run the code, simply execute the script. The optimal state and control input trajectories will be plotted in a new window.
