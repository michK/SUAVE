## @ingroup Input_Output-Results
# plot_2d.py

# Created: M. Kruger
# Updated:

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import pickle

def plot_2d(ax, filename, plot_vars_left, plot_vars_right=None):
    """
    Create 2D plot of data stored in specified data file

    plot_vars_left and plot_vars_right are lists of plot variable pairs,
    which enables various datasets to be plotted on both the (standard)
    left y-axis and the right y-axis
    """

    # Read data stored in file into Python
    res = pickle.load(open(filename, "rb"))

    for data in plot_vars_left:
        x_data = res[data[0]]
        y_data = res[data[1]]
        conv   = res["Converged"]

        if 0 in conv:
            print("Warning: {} out of {} points in sweep did not converge".format(conv.count(0), len(conv)))

        ax.plot(x_data, y_data)

    if plot_vars_right is not None:
        ax2 = ax.twinx()
        for data in plot_vars_right:
            x_data = res[data[0]]
            y_data = res[data[1]]

            ax2.plot(x_data, y_data)
