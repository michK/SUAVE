## @ingroup Methods-Power_Balance-Power_Balance
# Power_Balance.py
# 
# Created:  Oct 2018, Michael Kruger
# Modified: 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
from scipy.optimize import fsolve

import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Methods.Power_Balance.calculate_powers import remove_negatives

# ----------------------------------------------------------------------
#  Power Balance
# ----------------------------------------------------------------------

## @ingroup Methods-Power_Balance
def Power_Balance(vehicle, propsys, state_sizing):
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

    dia_fan_mech = propsys.fan_diameter_mech
    dia_fan_elec = propsys.fan_diameter_elec

    # Calculate wing BLI from electrical propulsors
    fBLIe = (nr_fans_elec * dia_fan_elec) / (vehicle.wings['main_wing'].spans.projected -
        vehicle.fuselages['fuselage'].effective_diameter)

    # Calculate fan areas
    area_fan_mech = np.pi / 4.0 * dia_fan_mech**2.0
    area_fan_elec = np.pi / 4.0 * dia_fan_elec**2.0

    # Calculate jet area
    area_jet_mech = area_fan_mech * propsys.area_noz_fan * propsys.area_jet_noz
    area_jet_elec = area_fan_elec * propsys.area_noz_fan * propsys.area_jet_noz

    # Number of fans
    nr_mech_fans = propsys.number_of_engines_mech
    nr_elec_fans = propsys.number_of_engines_elec

    # Calculate aerodynamics
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero_Unified()
    aerodynamics.geometry = vehicle
    aerodynamics.initialize()

    results_aero = aerodynamics.evaluate(state)

    # Extract drags from aero results
    CD_tot = results_aero.drag.total  # FIXME - This value does not seem to match result plot of CD_tot

    Vinf = state.conditions.freestream.cruise_speed

    # delta_vjet_mech = 2.09  # FIXME - From LEARN model for TH, should be calculated
    # delta_vjet_elec = 2.09  # FIXME - From LEARN model for TH, should be calculated
    # Vjetm = delta_vjet_mech * Vinf
    # Vjete = delta_vjet_elec * Vinf

    fsurf = 0.9  # FIXME - Move to file of constants

    # Calculate total drag
    qinf = 0.5 * state.conditions.freestream.density * Vinf**2.0
    Dp = CD_tot * qinf * vehicle.reference_area

    # Calculate PKm and PKe
    PKm_tot = (1.0 - fL) * PK_tot
    PKe_tot = fL * PK_tot

    # Solve power balance equations

    def power_balance(params):
        """Function to calculate resisuals of power balance equations"""
        mdotm_tot, mdote_tot, Vjetm, Vjete = params

        res1 = PKm_tot - 0.5 * mdotm_tot * (Vjetm**2.0 - Vinf**2.0) - fBLIm * fsurf * Dp * Vinf

        res2 = PKe_tot - 0.5 * mdote_tot * (Vjete**2.0 - Vinf**2.0) - fBLIe * fsurf * Dp * Vinf

        res3 = mdotm_tot - nr_mech_fans * vehicle.cruise_density * area_jet_mech * Vjetm

        res4 = mdote_tot - nr_elec_fans * vehicle.cruise_density * area_jet_elec * Vjete

        residuals = [
                     abs(res1),
                     abs(res2),
                     abs(res3),
                     abs(res4),
                    ]

        return np.asarray(residuals).reshape(4,)

    args_init = [100.0, 100.0, 50.0, 50.0]  # FIXME - should be more clever guesses
    [mdotm_tot, mdote_tot, _, _] = remove_negatives(fsolve(power_balance, args_init))

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

    # results.PK_tot = PKm_tot + PKe_tot

    return results
