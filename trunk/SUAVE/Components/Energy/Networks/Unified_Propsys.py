## @ingroup Components-Energy-Networks
# Unified_Propsys.py
#
# Created:  Oct 2018, Michael Kruger
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from SUAVE.Core import Data
from SUAVE.Methods.Power.Battery.Variable_Mass import find_mass_gain_rate
from SUAVE.Components.Propulsors.Propulsor import Propulsor

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Unified_Propsys(Propulsor):
    """ Unified propulsion system model

        Assumptions:
        None

        Source:
        Hall, D. K., Huang, A. C., Uranga, A., Greitzer, E. M., Drela, M., and Sato, S.,
        "Boundary Layer Ingestion Propulsion Benefit for Transport Aircraft",
        Journal of Propulsion and Power, Vol. 33, No. 5, 2017, pp. 1118-1129,
        doi:10.2514/1.B36321.
    """


    def __defaults__(self):
        """ This sets the default values for the network to function.

            Assumptions:
            N/A

            Source:
            N/A

            Inputs:
            None

            Outputs:
            None

            Properties Used:
            N/A
        """

        self.propulsor        = None
        self.battery          = None
        self.motor_efficiency = .95
        self.tag              = 'Network'

        #areas needed for drag; not in there yet
        self.areas             = Data()
        self.areas.wetted      = 0.0

    # linking the different network components
    def evaluate_power(self, state):
        """ Calculate power given the current state of the vehicle

            Assumptions:
            None

            Source:
            N/A

            Inputs:
            state [state()]

            Outputs:
            results.vehicle_mass_rate   [kg/s]

            Properties Used:
            Defaulted values
        """

        # Unpack inputs
        # nr_engines = vehicle.nr_engines
        # nr_mech_fans = vehicle.nr_mech_fans
        # nr_elec_fans = vehicle.nr_elec_fans
        # PKtot = vehicle.PKtot
        # state = state_sizing

        fL = self.fL
        fBLIm = self.fBLIm
        fBLIe = self.fBLIe

        CD_tot = state.conditions.aerodynamics.drag_breakdown.total.mean()
        CD_par = state.conditions.aerodynamics.drag_breakdown.parasite.total.mean()

        Vinf = state.conditions.freestream.velocity.mean()

        delta_vjet_mech = 2.09  # FIXME - From LEARN model for TH, should be calculated
        delta_vjet_elec = 2.09  # FIXME - From LEARN model for TH, should be calculated
        Vjetm = delta_vjet_mech * Vinf
        Vjete = delta_vjet_elec * Vinf

        fsurf = 0.9

        # Calculate total drag
        qinf = 0.5 * state.conditions.freestream.density.mean() * Vinf**2.0
        Dp = CD_tot * qinf * self.reference_area
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
        # mdotm = np.zeros(nr_mech_fans)
        # mdote = np.zeros(nr_elec_fans)
        # PKm = np.zeros(nr_mech_fans)
        # PKe = np.zeros(nr_elec_fans)

        # for i in range(nr_mech_fans):
        #     mdotm[i] = mdotm_tot / nr_mech_fans
        #     PKm[i] = PKm_tot / nr_mech_fans

        # for j in range(nr_elec_fans):
        #     mdote[j] = mdote_tot / nr_elec_fans
        #     PKe[j] = PKe_tot / nr_elec_fans

        results = Data()
        results.PK_tot = PKm_tot + PKe_tot
        results.mdot_tot = mdotm_tot + mdote_tot
        # results.mdote = mdote
        # results.PKm = PKm
        # results.PKe = PKe
        # results.PK_tot = PKm_tot + PKe_tot

        return results

    __call__ = evaluate_power
