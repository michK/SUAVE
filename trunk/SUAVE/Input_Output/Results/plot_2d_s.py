## @ingroup Input_Output-Results
# plot_2d.py

# Created: M. Kruger
# Updated:

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import pickle
from cycler import cycler

cycler_1 = cycler('color', ['b', 'r', 'g']) * cycler('linestyle', ['-', '--'])
cycler_2 = cycler('color', ['r', 'g', 'b']) * cycler('linestyle', ['-', '--'])

def plot_2d_s(ax, filename, plot_vars_left, plot_vars_right=None):
    """
    Create 2D plot of data stored in specified data file

    _s designates that this function plots data of one result file

    plot_vars_left and plot_vars_right are lists of plot variable pairs,
    which enables various datasets to be plotted on both the (standard)
    left y-axis and the right y-axis
    """

    # Read data stored in file into Python
    res = pickle.load(open(filename, "rb"))

    ax.set_prop_cycle(cycler_1)
    for data in plot_vars_left:
        x_data = res[data[0]]
        y_data = res[data[1]]
        conv   = res["Converged"]

        if 0 in conv:
            print("Warning: {} out of {} points in sweep did not converge".format(conv.tolist().count(0), len(conv.tolist())))

        ax.plot(x_data, y_data)

    if plot_vars_right is not None:
        ax2 = ax.twinx()
        ax2.set_prop_cycle(cycler_2)
        for data in plot_vars_right:
            x_data = res[data[0]]
            y_data = res[data[1]]

            ax2.plot(x_data, y_data)

    return ax, ax2
