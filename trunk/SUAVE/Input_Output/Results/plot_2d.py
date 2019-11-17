## @ingroup Input_Output-Results
# plot_2d.py

# Created: M. Kruger
# Updated:

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import csv

def plot_2d(ax, filename):
    """Create 2D plot of data stored in specified data file"""

    # Initialize lists to store data
    x_data = []
    y_data = []
    conv = []
    # Read data stored in file into Python
    with open(filename) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        next(readCSV)
        for row in readCSV:
            if len(row) != 0:
                x_data.append(float(row[0]))
                y_data.append(float(row[1]))
                conv.append(int(row[2]))

    if 0 in conv:
        print("Warning: {} out of {} points in sweep did not converge".format(conv.count(0), len(conv)))

    ax.xaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
    ax.plot(x_data, y_data)
