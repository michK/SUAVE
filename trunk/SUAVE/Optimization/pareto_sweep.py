## @ingroup Optimization
#  pareto_sweep.py
#
# Created  : Jul 2018, M. Kruger
# Modified :

# ----------------------------------------------------------------------
#  Imports
# -------------------------------------------

import sys
import os
import pdb

import numpy as np
import matplotlib.pyplot as plt

import progressbar

import multiprocessing, logging
from functools import partial
from contextlib import contextmanager
from itertools import repeat

import SUAVE
from SUAVE.Core import Data
from .Package_Setups import pyoptsparse_setup

class ForkablePdb(pdb.Pdb):

    _original_stdin_fd = sys.stdin.fileno()
    _original_stdin = None

    def __init__(self):
        pdb.Pdb.__init__(self, nosigint=True)

    def _cmdloop(self):
        current_stdin = sys.stdin
        try:
            if not self._original_stdin:
                self._original_stdin = os.fdopen(self._original_stdin_fd)
            sys.stdin = self._original_stdin
            self.cmdloop()
        finally:
            sys.stdin = current_stdin

# ----------------------------------------------------------------------
#  pareto_sweep
# ----------------------------------------------------------------------

def optimize_objective(i, args):
    """thread worker function to be run in parallel on multiple threads"""


    problem, opt_prob, obj_scaling, inputs, idx0 = args

    opt_prob.inputs[:,1][idx0]= inputs[0,i]
    opt_prob.inputs[idx0][2] = (inputs[0,i], inputs[0,i])
    problem.optimization_problem = opt_prob
    # ForkablePdb().set_trace()
    sol = pyoptsparse_setup.Pyoptsparse_Solve(problem, solver='SNOPT', sense_step=1e-06)
    obj = sol.fStar * obj_scaling

    return obj


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

    mpl = multiprocessing.log_to_stderr()
    mpl.setLevel(logging.INFO)

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

    print("Performing variable sweep:")

    # Run sweep on multiple threads
    num_workers = multiprocessing.cpu_count()
    args = [problem, opt_prob, obj_scaling, inputs, idx0]
    # with multiprocessing.Pool(processes=num_workers) as pool:
    with multiprocessing.Pool(processes=1) as pool:
        results = pool.starmap(optimize_objective, zip(range(number_of_points), repeat(args)))

    obj = results

    # Create plot
    fig, ax = plt.subplots()

    ax.plot(inputs[0,:], obj, lw = 2)
    ax.set_xlabel(names[idx0])
    ax.set_ylabel(obj_name)

    plt.show()

    return
