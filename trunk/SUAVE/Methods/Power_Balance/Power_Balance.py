## @ingroup Methods-Power_Balance-Power_Balance
# Power_Balance.py
# 
# Created:  Oct 2018, Michael Kruger
# Modified: 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np

import SUAVE
from SUAVE.Core import Data, Units

# ----------------------------------------------------------------------
#  Power Balance
# ----------------------------------------------------------------------

## @ingroup Methods-Power_Balance
def Power_Balance(vehicle, state_sizing):
    """
    Method runs once per outer iteration loop to size propulsion system
    Assumptions:
        None

    Source:
        Hall, D. K., Huang, A. C., Uranga, A., Greitzer, E. M., Drela, M., and Sato, S.,
        "Boundary Layer Ingestion Propulsion Benefit for Transport Aircraft",
        Journal of Propulsion and Power, Vol. 33, No. 5, 2017, pp. 1118-1129,
        doi:10.2514/1.B36321.
        
    Inputs: TODO - Expand to show all variables
      vehicle
      state
 
    Properties Used:
        N/A
    """
    # Unpack inputs
    nr_fans_mech = vehicle.nr_fans_mech
    nr_fans_elec = vehicle.nr_fans_elec
    PKtot = vehicle.PKtot
    state = state_sizing
    PK_tot = vehicle.PKtot
    
    fL = vehicle.fL
    fBLIm = vehicle.fBLIm
    fBLIe = vehicle.fBLIe

    # Calculate aerodynamics
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero_Unified()
    aerodynamics.geometry = vehicle
    aerodynamics.initialize()

    results_aero = aerodynamics.evaluate(state)

    # Extract drags from aero results
    CD_tot = results_aero.drag.total  # TODO - Check that this is actual CD_tot

    Vinf = state.conditions.freestream.cruise_speed

    Vjetm = 200.0 * Units.kt #  TODO - Calculate
    Vjete = 200.0 * Units.kt #  TODO - Calculate

    fsurf = 0.9

    # Calculate total drag
    qinf = 0.5 * state.conditions.freestream.density * Vinf**2.0
    Dp = CD_tot * qinf * vehicle.reference_area

    # Calculate PKm and PKe
    PKm_tot = (1.0 - fL) * PK_tot
    PKe_tot = fL * PK_tot    

    # Calculate required mass flows

    mdotm_tot = 2.0 * (PKm_tot - fBLIm * fsurf * Dp * Vinf) / (Vjetm**2.0 - Vinf**2.0)
    mdote_tot = 2.0 * (PKe_tot - fBLIe * fsurf * Dp * Vinf) / (Vjete**2.0 - Vinf**2.0)

    # Calculate individual propulsor stream mass flows and propulsive powers
    mdotm = np.zeros(nr_fans_mech)
    mdote = np.zeros(nr_fans_elec)
    PKm = np.zeros(nr_fans_mech)
    PKe = np.zeros(nr_fans_elec)

    for i in range(nr_fans_mech):
        mdotm[i] = mdotm_tot / nr_fans_mech
        PKm[i] = PKm_tot / nr_fans_mech

    for j in range(nr_fans_elec):
        mdote[j] = mdote_tot / nr_fans_elec
        PKe[j] = PKe_tot / nr_fans_elec

    results = Data()
    results.PKtot = PKtot
    results.mdotm = mdotm
    results.mdote = mdote
    results.PKm = PKm
    results.PKe = PKe

    results.PK_tot = PKm_tot + PKe_tot

    return results
