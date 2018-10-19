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
    nr_engines = vehicle.nr_engines
    nr_mech_fans = vehicle.nr_mech_fans
    nr_elec_fans = vehicle.nr_elec_fans
    PKtot = vehicle.PKtot
    state = state_sizing
    
    fL = vehicle.fL
    fBLIm = vehicle.fBLIm
    fBLIe = vehicle.fBLIe

    # Calculate aerodynamics
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.initialize()

    results_aero = aerodynamics.evaluate(state)

    # Extract drags from aero results
    CD_tot = results_aero.drag.total  # TODO - Check that this is actual CD_tot
    CD_par = results_aero.drag.parasite.total

    Vinf = state.conditions.freestream.cruise_speed

    delta_vjet_mech = 2.09  # FIXME - From LEARN model for TH, should be calculated
    delta_vjet_elec = 2.09  # FIXME - From LEARN model for TH, should be calculated
    Vjetm = delta_vjet_mech * Vinf
    Vjete = delta_vjet_elec * Vinf

    fsurf = 0.9

    # Calculate total drag
    qinf = 0.5 * state.conditions.freestream.density * Vinf**2.0
    Dp = CD_tot * qinf * vehicle.reference_area
    Dpp_DP = CD_par / CD_tot

    # Set up system of equations to solve power balance

    A = np.array((
                 [fL, fL - 1.0, 0                             , 0                            ],
                 [0 , 0       , Vjetm - Vinf                  , Vjete - Vinf                 ],
                 [1 , 0       , -0.5*(Vjetm**2.0 - Vinf**2.0) , 0                            ],
                 [0 , 1       , 0                             , -0.5*(Vjete**2.0 - Vinf**2.0)]
                ))

    b = np.array((
                 [0],
                 [Dp * (1.0 - fBLIm * Dpp_DP - fBLIe * Dpp_DP)],
                 [fBLIm * fsurf * Dpp_DP * Dp],
                 [fBLIe * fsurf * Dpp_DP * Dp]
                ))

    # Solve system
    [PKm_tot, PKe_tot, mdotm_tot, mdote_tot] = np.linalg.solve(A, b)

    # Calculate individual propulsor stream mass flows and propulsive powers
    mdotm = np.zeros(nr_mech_fans)
    mdote = np.zeros(nr_elec_fans)
    PKm = np.zeros(nr_mech_fans)
    PKe = np.zeros(nr_elec_fans)

    for i in range(nr_mech_fans):
        mdotm[i] = mdotm_tot / nr_mech_fans
        PKm[i] = PKm_tot / nr_mech_fans

    for j in range(nr_elec_fans):
        mdote[j] = mdote_tot / nr_elec_fans
        PKe[j] = PKe_tot / nr_elec_fans

    results = Data()
    results.PKtot = PKtot
    results.mdotm = mdotm
    results.mdote = mdote
    results.PKm = PKm
    results.PKe = PKe
    results.PK_tot = PKm_tot + PKe_tot

    return results
