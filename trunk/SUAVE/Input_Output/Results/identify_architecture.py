## @ingroup Input_Output-Results
# identify_architecture.py 

# Created: M. Kruger
# Updated:

def identify_arch(propsys):
    fS = propsys.info.fS_max
    fL = propsys.info.fL_max

    if (fS <= 0.01 and fL <= 0.01):
        arch = 'Conventional'
    elif (fS <= 0.01 and 0.01 <= fL <= 0.99):
        arch = 'Partial Turbo-electric'
    elif (fS <= 0.01 and 0.99 <= fL):
        arch = 'Full Turbo-electric'
    elif (0 <= fS <= 1 and fL <= 0.01):
        arch = 'Parallel Hybrid'
    elif (0 <= fS <= 1 and 0 <= fL <= 1):
        arch = 'Series/Parallel Partial Hybrid'
    elif (0 <= fS <= 1 and fL >= 0.99):
        arch = 'Series Hybrid'
    elif (fS >= 0.99 and fL >= 0.99):
        arch = 'All-electric'
    elif (fS >= 0.99 and fL <= 0.01) or (fS >= 0.99 and 0 <= fL <= 1):
        arch = 'Check design: Configuration not physical'

    return arch
