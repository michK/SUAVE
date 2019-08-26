import unittest
import numpy as np

import SUAVE
from SUAVE.Core import Units

class TestPowerBalance(unittest.TestCase):

    def test_power_balance(self):

        thrust = SUAVE.Components.Energy.Processes.Unified_Thrust()

        # Inputs from higher level
        thrust.inputs.vertical_velocity = np.array([-2000]) * Units['ft/min']
        thrust.inputs.Vinf          = np.array([93.62888889])
        thrust.inputs.rho_inf       = np.array([0.771])
        thrust.inputs.Dp            = np.array([2351.26388961])
        thrust.inputs.Dpp_DP        = 0.77228994
        thrust.inputs.Dpar          = thrust.inputs.Dpp_DP * thrust.inputs.Dp
        thrust.inputs.nr_elements   = 1
        thrust.inputs.fS            = 0.0
        thrust.inputs.fL            = 1.0
        thrust.inputs.eta_propm     = 0.9
        thrust.inputs.eta_prope     = 0.9
        thrust.inputs.eta_th        = 0.5
        thrust.inputs.eta_pe        = 0.98
        thrust.inputs.eta_mot       = 0.95
        thrust.inputs.eta_fan       = 0.9
        thrust.inputs.fBLIe         = 0.0
        thrust.inputs.fBLIm         = 0.0
        thrust.inputs.fsurf         = 0.9
        thrust.inputs.nr_fans_elec  = 2
        thrust.inputs.nr_fans_mech  = 2
        thrust.inputs.area_jet_mech = 0.322
        thrust.inputs.area_jet_elec = 0.322
        thrust.inputs.hfuel         = 43 * Units['MJ/kg']
        thrust.inputs.max_bat_power = 1.350 * Units.MW
        thrust.inputs.Cp            = 289 * Units['g/kW/hr']

        # Create custom conditions
        conditions = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
        conditions.weights.total_mass  = np.array([5000])
        conditions.propulsion.throttle = np.array([1])

        thrust.compute(conditions)

        print(thrust.outputs)

        # self.assertAlmostEqual(variable, expected_value, places=0, msg="Should be about expected_value")

if __name__ == '__main__':
    unittest.main()
