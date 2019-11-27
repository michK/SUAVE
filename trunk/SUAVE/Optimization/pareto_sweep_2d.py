## @ingroup Optimization
#  pareto_sweep_2d.py
#
# Created  : Nov 2018, M. Kruger
# Modified :

# ----------------------------------------------------------------------
#  Imports
# -------------------------------------------

import SUAVE
from SUAVE.Core import Data
from .Package_Setups import pyoptsparse_setup
from SUAVE.Input_Output.Results import print_SNOPT_summary

import numpy as np
import pickle
import os

# ----------------------------------------------------------------------
#  pareto_sweep_2d
# ----------------------------------------------------------------------


def pareto_sweep_2d(problem, print_PSEC, number_of_points, sweep_index_0, sweep_index_1, datafile='data.out'):
    """
    Takes in an optimization problem and runs a Pareto sweep of the sweep indexes sweep_index_0 and sweep_index_1.
    i.e. sweep_index_0=0,weep_index_2=1 means you want to sweep the first 2 indexes to draw a carpet plot of the cost function
    versus variables 1 and 2.
    This function is based largely on the pareto_sweep() function, which itself is a
    simplified version of the line_plot function,
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

    idx0             = sweep_index_0 # local name
    idx1             = sweep_index_1 # local name

    opt_prob         = problem.optimization_problem
    base_inputs      = opt_prob.inputs
    names            = base_inputs[:,0] # Names
    bnd              = base_inputs[:,2] # Bounds
    scl              = base_inputs[:,3] # Scaling
    base_objective   = opt_prob.objective
    obj_name         = base_objective[0][0] #objective function name (used for scaling)
    obj_scaling      = base_objective[0][1]
    base_constraints = opt_prob.constraints

    # Define and initialize inputs
    inputs0 = np.zeros((2,number_of_points))
    inputs1 = np.zeros((2,number_of_points))
    # Define and initialize output
    converged = np.zeros((number_of_points, number_of_points))
    obj       = np.zeros((number_of_points, number_of_points))
    PSEC      = np.zeros((number_of_points, number_of_points))
    mto       = np.zeros((number_of_points, number_of_points))
    mdottot   = np.zeros((number_of_points, number_of_points))
    PKtot     = np.zeros((number_of_points, number_of_points))

    # Create inputs matrix
    inputs0[0,:] = np.linspace(bnd[idx0][0], bnd[idx0][1], number_of_points)
    inputs1[0,:] = np.linspace(bnd[idx1][0], bnd[idx1][1], number_of_points)

    # Initialize results dictionary
    res = dict()

    # Check if file exists - For now this only works with serial runs
    # filepath = os.path.join(os.path.expanduser('.'), 'Data',datafile)
    # if os.path.exists(filepath):
    #     q = input("This data file already exists, do you want to overwrite it? :")
    #     if q in ['YES', 'yes', 'y', 'Y']:
    #         pass
    #     else:
    #         raise FileExistsError("This file already exists and you chose not to overwrite it")

    # Create file to write results into
    data_path = os.path.join(os.path.expanduser('.'), 'Data',datafile)
    #inputs defined; now run sweep
    for i in range(0, number_of_points):
        for j in range(0, number_of_points):
            opt_prob.inputs[:,1][idx0] = inputs0[0,i]
            opt_prob.inputs[:,1][idx1] = inputs1[0,j]

            opt_prob.inputs[idx0][2] = (inputs0[0,i], inputs0[0,i])
            opt_prob.inputs[idx1][2] = (inputs1[0,j], inputs1[0,j])
            problem.optimization_problem = opt_prob
            sol = pyoptsparse_setup.Pyoptsparse_Solve(
                problem, solver='SNOPT', FD='parallel', sense_step=1e-06, is_Sweep=True)
            obj[i, j] = problem.objective() * obj_scaling
            PSEC[i, j] = problem.summary.PSEC

            if print_SNOPT_summary('/home/michael/Dropbox/PhD/Research/Codes/CADA/CADA/Commuter/SNOPT_summary.out'):  # Work machine
            # if print_SNOPT_summary('/Users/michael/Dropbox/PhD/Research/Codes/CADA/CADA/Commuter/SNOPT_summary.out'):  # Mac book
                converged[i,j] = 1
            else:
                converged[i,j] = 0

            # Extract parameters from problem
            mto[i,j] = problem.vehicle_configurations.base.mass_properties.max_takeoff
            mdottot[i,j] = problem.vehicle_configurations.base.mdottot
            PKtot[i,j] = problem.vehicle_configurations.base.PKtot

        # Delete history file
        # if os.path.exists(os.path.join(os.path.expanduser('.'), 'snopt.hist')):
            # os.remove('snopt.hist')

    # Populate results dictionary
    res["Input0"] = inputs0[0,:]
    res["Input1"] = inputs1[0,:]
    res["Obj"] = obj
    res["PSEC"] = PSEC
    res["Converged"] = converged
    res["Mto"] = mto
    res["mdottot"] = mdottot
    res["PKtot"] = PKtot

    # Write results dictionary to file
    pickle.dump(res, open(data_path, "wb"))

    return
