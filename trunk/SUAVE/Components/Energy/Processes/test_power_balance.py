import unittest
import numpy as np
import SUAVE
from SUAVE.Core import Units

class TestPowerBalance(unittest.TestCase):

    def test_power_balance(self):

        thrust = SUAVE.Components.Energy.Processes.Unified_Thrust_tmp()

        # Inputs from higher level
        thrust.inputs.vertical_velocity = np.array([500]) * Units['ft/min']
        thrust.inputs.Vinf          = np.array([77])
        thrust.inputs.rho_inf       = np.array([0.77])
        thrust.inputs.Dp            = np.array([2900.0])
        thrust.inputs.Dpp_DP        = 0.5
        thrust.inputs.Dpar          = thrust.inputs.Dpp_DP * thrust.inputs.Dp
        thrust.inputs.nr_elements   = 1
        thrust.inputs.fS            = 0.5
        thrust.inputs.fL            = 0.5
        thrust.inputs.eta_propm     = 0.9
        thrust.inputs.eta_prope     = 0.9
        thrust.inputs.eta_th        = 0.5
        thrust.inputs.eta_pe        = 0.98
        thrust.inputs.eta_mot       = 0.95
        thrust.inputs.eta_fan       = 0.9
        thrust.inputs.fBLIe         = 0
        thrust.inputs.fBLIm         = 0
        thrust.inputs.fsurf         = 0.9
        thrust.inputs.nr_fans_elec  = 2
        thrust.inputs.nr_fans_mech  = 2
        thrust.inputs.area_jet_mech = 1
        thrust.inputs.area_jet_elec = 1
        thrust.inputs.hfuel         = 43 * Units['MJ/kg']

        # Create custom conditions
        conditions = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()
        conditions.weights.total_mass  = np.array([5000])
        conditions.propulsion.throttle = np.array([0.5])  # Dummy variable (not used)

        thrust.compute(conditions)

        print(thrust.outputs)

        # self.assertAlmostEqual(thrust.outputs.thrust, 2900.0, places=0, msg="Thrust should be 2900 N")
        # self.assertAlmostEqual(thrust.outputs.PKm,    111650, places=0, msg="Should be about 111650 W")
        # self.assertAlmostEqual(thrust.outputs.PKe,    111650, places=0, msg="Should be about 111650 W")
        # self.assertAlmostEqual(thrust.outputs.mdotm_tot, 114,    places=0, msg="Should be about 114")
        # self.assertAlmostEqual(thrust.outputs.mdote_tot, 114,    places=0, msg="Should be about 114")
        # self.assertAlmostEqual(thrust.outputs.Vjetm_tot, 94,     places=0, msg="Should be about 94")
        # self.assertAlmostEqual(thrust.outputs.Vjete_tot, 94,     places=0, msg="Should be about 94")

        # self.assertAlmostEqual(1, 1)

if __name__ == '__main__':
    unittest.main()
