'''
Trajectory optimization tools
'''
from pydrake.solvers import MathematicalProgram, Solve
import numpy as np

# To get joint limits: (TODO CHECK)
# lower_limits, upper_limits = get_custom_limits(world.robot, world.arm_joints)
# JOINT_LIMITS = [lower_limits, upper_limits]

def optimize_trajectory(start, goal):
    # Decision var dimensions
    state_dim = len(path[0])

    # Create optimization problem
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(state_dim, path_len, "theta")

    ## Constraints
    # start and goal
    prog.addConstraint(x[:, 1] == path[ 0])
    prog.addConstraint(x[:,-1] == path[-1])
    # joint limits
    for i, lim in enumerate(JOINT_LIMITS):
        # apply for one joint along all paths
        prog.addConstraint(x[i,:] >= lim[0])
        prog.addConstraint(x[i,:] <= lim[1])
    # velocity limits
    for i in range(1,path_len):
        prog.addConstraint((x[:,i] - x[:,i-1])/dt <= JOINT_VEL_MAX)
        prog.addConstraint((x[:,i] - x[:,i-1])/dt >= -JOINT_VEL_MAX)

    ## Objective function
    for i in range()

    pass