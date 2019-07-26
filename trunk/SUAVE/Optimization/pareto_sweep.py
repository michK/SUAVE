## @ingroup Optimization
#  pareto_sweep.py
#
# Created  : Jul 2018, M. Kruger
# Modified :

# ----------------------------------------------------------------------
#  Imports
# -------------------------------------------

import SUAVE
from SUAVE.Core import Data
from .Package_Setups import pyoptsparse_setup

import numpy as np
import matplotlib.pyplot as plt

import progressbar

# ----------------------------------------------------------------------
#  pareto_sweep
# ----------------------------------------------------------------------


def pareto_sweep(problem, number_of_points, sweep_index):
    """
    Takes in an optimization problem and runs a Pareto sweep of the first  variable of sweep index
    sweep_index. i.e. sweep_index=0 means you want to sweep the first variable, sweep_index = 4 is the 5th variable)
    This function is based largely on a simplified version of the line_plot function,
    with the added functionality that it runs the optimization problem for every point in the sweep,
    not just evaluate the objective function with other design variables fixed at their initial values,
    such as in line_plot()

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:
        problem            [Nexus Class]
        number_of_points   [int]
        sweep_index        [int]


        Outputs:
            inputs     [array]
            objective  [array]
            constraint [array]

        Properties Used:
        N/A
    """

    idx0             = sweep_index # local name

    opt_prob         = problem.optimization_problem
    base_inputs      = opt_prob.inputs
    names            = base_inputs[:,0] # Names
    bnd              = base_inputs[:,2] # Bounds
    scl              = base_inputs[:,3] # Scaling
    base_objective   = opt_prob.objective
    obj_name         = base_objective[0][0] #objective function name (used for scaling)
    obj_scaling      = base_objective[0][1]
    base_constraints = opt_prob.constraints

    #define inputs, output, and constraints for sweep
    inputs = np.zeros([2,number_of_points])
    obj    = np.zeros([number_of_points])

    #create inputs matrix
    inputs[0,:] = np.linspace(bnd[idx0][0], bnd[idx0][1], number_of_points)

    bar = progressbar.ProgressBar(maxval=number_of_points, \
        widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])

    print("Performing variable sweep:")
    bar.start()
    #inputs defined; now run sweep
    for i in range(0, number_of_points):        
        opt_prob.inputs[:,1][idx0]= inputs[0,i]

        # obj[i] = problem.objective() * obj_scaling
        opt_prob.inputs[idx0][2] = (inputs[0,i], inputs[0,i])
        problem.optimization_problem = opt_prob
        sol = pyoptsparse_setup.Pyoptsparse_Solve(problem, solver='SNOPT', sense_step=1e-06)
        obj[i] = sol.fStar * obj_scaling

        bar.update(i)

    bar.finish()

    # Create plot
    fig, ax = plt.subplots()

    ax.plot(inputs[0,:], obj, lw = 2)
    ax.set_xlabel(names[idx0])
    ax.set_ylabel(obj_name)

    plt.show()

    return
