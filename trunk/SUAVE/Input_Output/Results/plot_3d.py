## @ingroup Input_Output-Results
# plot_3d.py

# Created: M. Kruger
# Updated:

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import pickle
from cycler import cycler


def plot_3d(ax, filename, sweepvar_0, sweepvar_1, plotvar, is_relative=None):
    """
    Create contour plot of data stored in specified data file

    sweepvar_0 and sweepvar_1 are the swept variables plotted on the
    x and y axes and plotvar is the variable whose magnitude is
    plotted on the z axis
    """

    # Read data stored in file into Python
    res = pickle.load(open(filename, "rb"))

    # Create grids to plot variables over
    XX, YY = np.meshgrid(res[sweepvar_0], res[sweepvar_1])

    conv   = res["Converged"]
    if 0 in conv:
        # print("Warning: {} out of {} points in sweep did not converge".format(conv.tolist().count(0), len(conv.tolist())))
        print("Warning: some points in sweep did not converge")

    if is_relative == None:
        CS = ax.contourf(XX, YY, np.rot90(res[plotvar],axes=(1,0)), cmap=plt.cm.bwr)
    else:
        norm_rel = is_relative
        val = np.rot90((res[plotvar] - norm_rel) / res[plotvar] * 100, axes=(1,0))
        # val = np.transpose((res[plotvar] - norm_rel) / res[plotvar] * 100)
        CS = ax.contourf(XX, YY, val, cmap=plt.cm.bwr)

    return ax, CS
