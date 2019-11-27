## @ingroup Input_Output-Results
# plot_2d.py

# Created: M. Kruger
# Updated:

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import pickle
from cycler import cycler


def plot_2d_m(ax, files, plot_vars, is_relative=None):
    """
    Create 2D plot of data stored in specified data files

    _m designates that this function plots data of multiple result files

    plot_vars are lists of plot variable pairs,
    which enables various datasets to be plotted
    """

    cycler_ = cycler('color', ['b', 'r', 'g']) * cycler('linestyle', ['-', '--'])
    ax.set_prop_cycle(cycler_)
    for filename in files:
    # Read data stored in file into Python
        res = pickle.load(open(filename, "rb"))

        for data in plot_vars:
            x_data = res[data[0]]
            y_data = res[data[1]]
            conv   = res["Converged"]

            if 0 in conv:
                print("Warning: {} out of {} points in sweep did not converge".format(conv.tolist().count(0), len(conv.tolist())))

            if is_relative is None:
                ax.plot(x_data, y_data)
            else:
                norm_rel = is_relative
                ax.plot(x_data, (y_data - norm_rel)/y_data*100)

    return ax
