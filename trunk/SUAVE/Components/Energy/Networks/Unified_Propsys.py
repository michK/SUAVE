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
from SUAVE.Core import Units, Data
from SUAVE.Methods.Power.Battery.Variable_Mass import find_mass_gain_rate
from SUAVE.Components.Propulsors.Propulsor import Propulsor
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers

# package imports
import numpy as np

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

        # areas needed for drag; not in there yet
        self.areas             = Data()
        self.areas.wetted      = 0.0

        # max power tracker
        self.max_power = 0.0

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

        #Unpack
        conditions = state.conditions       

        # Constants
        hfuel = 43.0 * Units['MJ/kg']
        eta_th = 0.5

        # Unpack inputs
        # nr_engines = vehicle.nr_engines
        # nr_mech_fans = vehicle.nr_mech_fans
        # nr_elec_fans = vehicle.nr_elec_fans
        # PKtot = vehicle.PKtot
        # state = state_sizing

        fL = self.fL
        fS = self.fS
        fBLIm = self.fBLIm
        fBLIe = self.fBLIe

        # Efficiencies
        eta_pe  = 0.98
        eta_mot = 0.95
        eta_fan = 0.9

        CD_tot = conditions.aerodynamics.drag_breakdown.total

        CD_par = conditions.aerodynamics.drag_breakdown.parasite.total

        Vinf = conditions.freestream.velocity

        delta_vjet_mech = 2.09  # FIXME - From LEARN model for TH, should be calculated
        delta_vjet_elec = 2.09  # FIXME - From LEARN model for TH, should be calculated
        Vjetm = delta_vjet_mech * Vinf
        Vjete = delta_vjet_elec * Vinf

        fsurf = 0.9

        # Calculate total drag
        qinf = 0.5 * conditions.freestream.density * Vinf**2.0
        Dp = CD_tot * qinf * self.reference_area
        Dpp_DP = CD_par / CD_tot

        # Set up system of equations to solve power balance
        nr_elements = np.shape(CD_tot)[0]

        # Initialize solution arrays
        PKm_tot = np.zeros(nr_elements)
        PKe_tot = np.zeros(nr_elements)
        mdotm_tot = np.zeros(nr_elements)
        mdote_tot = np.zeros(nr_elements)
        PKe = np.zeros(nr_elements)
        PKm = np.zeros(nr_elements)
        mdot_fuel = np.zeros(nr_elements)

        for i in range(nr_elements):

            A = np.array((
                         [fL, fL - 1.0, 0                             , 0                            ],
                         [0 , 0       , Vjetm[i] - Vinf[i]                  , Vjete[i] - Vinf[i]                 ],
                         [1 , 0       , -0.5*(Vjetm[i]**2.0 - Vinf[i]**2.0) , 0                            ],
                         [0 , 1       , 0                             , -0.5*(Vjete[i]**2.0 - Vinf[i]**2.0)]
                        ))

            b = np.array((
                         [0],
                         [Dp[i] * (1.0 - fBLIm * Dpp_DP[i] - fBLIe * Dpp_DP[i])],
                         [fBLIm * fsurf * Dpp_DP[i] * Dp[i]],
                         [fBLIe * fsurf * Dpp_DP[i] * Dp[i]]
                        ))

            # Solve system
            [PKm, PKe, mdotm, mdote] = np.linalg.solve(A, b)

            PKm_tot[i] = PKm
            PKe_tot[i] = PKe
            mdotm_tot[i] = mdotm
            mdote_tot[i] = mdote

            PK_tot = PKm + PKe

            [PKe_i, PKm_i, PfanE_i, PfanM_i, Pmot_i, Pinv_i, Pbat_i, Pturb_i, Pmot_link_i, Pconv_i, Plink_i] = \
            calculate_powers(PK_tot[0], fS, fL, eta_pe, eta_mot, eta_fan)

            PKm = PKm_i
            PKe = PKe_i           

            # Calculate vehicle mass rate of change
            mdot_fuel[i] = Pturb_i / (hfuel * eta_th)

        results = Data()
        results.PK_tot = (PKm_tot + PKe_tot).reshape(nr_elements, 1)
        results.power_required = results.PK_tot
        results.mdot_tot = mdot_fuel.reshape(nr_elements, 1)
        results.vehicle_mass_rate = results.mdot_tot

        #  Update max power
        if PK_tot > self.max_power:
            self.max_power = PK_tot

        # store data
        results_conditions = Data
        conditions.propulsion = results_conditions(PK_tot = results.PK_tot)

        return results

    __call__ = evaluate_power
