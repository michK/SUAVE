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
from SUAVE.Input_Output.Results import print_SNOPT_summary

import numpy as np
import matplotlib.pyplot as plt

import os

# ----------------------------------------------------------------------
#  pareto_sweep
# ----------------------------------------------------------------------


def pareto_sweep(problem, print_PSEC, number_of_points, sweep_index, datafile='data.out'):
    """
    Takes in an optimization problem and runs a Pareto sweep of the sweep index sweep_index.
    i.e. sweep_index=0 means you want to sweep the first variable, sweep_index = 4 is the 5th variable)
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
    PSEC   = np.zeros([number_of_points])

    #create inputs matrix
    inputs[0,:] = np.linspace(bnd[idx0][0], bnd[idx0][1], number_of_points)

    # Create file to write results into
    data_path = os.path.join(os.path.expanduser('.'), 'Data',datafile)
    with open(data_path, "w+") as f:
        f.write("Data file for sweep of PSEC vs fL\n")
        #inputs defined; now run sweep
        for i in range(0, number_of_points):
            opt_prob.inputs[:,1][idx0]= inputs[0,i]

            opt_prob.inputs[idx0][2] = (inputs[0,i], inputs[0,i])
            problem.optimization_problem = opt_prob
            sol = pyoptsparse_setup.Pyoptsparse_Solve(problem, solver='SNOPT', FD='parallel', sense_step=1e-06)
            obj[i] = problem.objective() * obj_scaling
            PSEC[i] = problem.summary.PSEC

            if print_SNOPT_summary('/home/michael/Dropbox/PhD/Research/Codes/CADA/CADA/Commuter/SNOPT_summary.out'):
                converged = 1
            else:
                converged = 0

            # Extract parameters from problem
            mto = problem.vehicle_configurations.base.mass_properties.max_takeoff

            f.write("{}, {}, {}, {}\n".format(inputs[0,i], PSEC[i], converged, mto))

    return
