## @ingroup Input_Output-Results
# identify_architecture.py 

# Created: M. Kruger
# Updated:

def identify_arch(propsys):
    fS = propsys.info.fS_max
    fL = propsys.info.fL_max

    if (fS <= 0.05 and fL <= 0.05):
        arch = 'Conventional'
    elif (fS <= 0.05 and 0 <= fL <= 1):
        arch = 'Partial Turbo-electric'
    elif (fS >= 0.95 and 0 <= fL <= 1):
        arch = 'Full Turbo-electric'
    elif (0 <= fS <= 1 and fL <= 0.05):
        arch = 'Parallel Hybrid'
    elif (0 <= fS <= 1 and 0 <= fL <= 1):
        arch = 'Series/Parallel Partial Hybrid'
    elif (0 <= fS <= 1 and fL >= 0.95):
        arch = 'Series Hybrid'
    elif (fS >= 0.95 and fL >= 0.95):
        arch = 'All-electric'
    elif (fS >= 0.95 and fL <= 0.05) or (fS >= 0.95 and 0 <= fL <= 1):
        arch = 'Check design: Configuration not physical'

    return arch
