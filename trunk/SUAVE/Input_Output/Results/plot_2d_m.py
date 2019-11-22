## @ingroup Input_Output-Results
# plot_2d.py

# Created: M. Kruger
# Updated:

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import pickle
from cycler import cycler


def plot_2d_m(ax, files, plot_vars, normalize_by=None, scale_by=1):
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

            if normalize_by is None:
                ax.plot(x_data, y_data*scale_by)
            else:
                norm_data = pickle.load(open(normalize_by[0], "rb"))
                norm_var = norm_data[normalize_by[1]]
                ax.plot(x_data, y_data/norm_var*scale_by)

    return ax
