## @ingroup Input_Output-Results
# print_SNOPT_summary.py 

# Created: M. Kruger
# Updated:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import datetime
import re

def print_SNOPT_summary(filename = 'SNOPT_summary.out'):
    """Function reads and prints information from SNOPT summary file"""

    print('-------SNOPT output summery-------\n')

    linesList = [line.rstrip('\n') for line in open(filename)]

    for i, line in enumerate(linesList):
        if 'SNOPTC EXIT' in line:
            print(line)
        if 'SNOPTC INFO' in line:
            print(line)

    print('---------------END---------------\n')