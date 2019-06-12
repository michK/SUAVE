# test_unified_network_sizing.py
#
# Created:  May 2019, M. Kruger
# Modified:

import unittest
import numpy as np
import SUAVE
from SUAVE.Core import Data, Units

from unified_network_sizing import unified_network_sizing

class TestUnifiedNetworkSizing(unittest.TestCase):

    def test_unified_network_sizing(self):

        vehicle = SUAVE.Vehicle()
        # vehicle.propulsors = Data()
        propsys = SUAVE.Components.Energy.Networks.Unified_Network_tmp()
        propsys.tag = 'unified_propsys'

        # Define propsys input variables
        propsys.number_of_engines_mech = 2
        propsys.number_of_engines_elec = 2

        # vehicle.propulsors.append_component(propsys)
        vehicle.append_component(propsys)

        # Define vehicle input variables
        vehicle.mdottot = 1000 * Units['kg/s']
        vehicle.fL_cruise = 0.5
        vehicle.cruise_mach = 0.23943836

        propsys = unified_network_sizing(vehicle)

        self.assertAlmostEqual(propsys.mech_fan_dia, 0.712, places=0, msg="Should be about 0.712")
        self.assertAlmostEqual(propsys.elec_fan_dia, 0.712, places=0, msg="Should be about 0.712")
        self.assertAlmostEqual(propsys.mech_nac_dia, 0.879, places=0, msg="Should be about 0.879")
        self.assertAlmostEqual(propsys.elec_nac_dia, 0.879, places=0, msg="Should be about 0.879")
        self.assertAlmostEqual(propsys.areas_wetted_mech, 4.00, places=0, msg="Should be about 4.00")
        self.assertAlmostEqual(propsys.areas_wetted_elec, 2.00, places=0, msg="Should be about 2.00")

if __name__ == '__main__':
    unittest.main()
