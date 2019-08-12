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

        propsys = SUAVE.Components.Energy.Networks.Unified_Network()
        propsys.tag = 'unified_propsys'

        # Define propsys input variables
        propsys.number_of_engines_mech = 2
        propsys.number_of_engines_elec = 4

        vehicle.append_component(propsys)

        # Define vehicle input variables
        vehicle.mdottot_cruise = 500 * Units['kg/s']
        vehicle.fL_cruise = 0.5
        vehicle.cruise_mach = 0.23943836

        wing = SUAVE.Components.Wings.Main_Wing()
        wing.tag = 'main_wing'
        wing.spans.projected = 19.81 * Units.meter
        vehicle.append_component(wing)

        fuselage = SUAVE.Components.Fuselages.Fuselage()
        fuselage.tag = 'fuselage'
        fuselage.width = 1.829 * Units.meter
        fuselage.heights.maximum = 1.829 * Units.meter
        fuselage.effective_diameter = 0.5 * (fuselage.width + fuselage.heights.maximum)
        vehicle.append_component(fuselage)

        propsys = unified_network_sizing(vehicle.propulsors.unified_propsys, vehicle)

        self.assertAlmostEqual(propsys.mech_fan_dia, 0.712, places=0, msg="Should be about 0.712")
        self.assertAlmostEqual(propsys.elec_fan_dia, 0.712, places=0, msg="Should be about 0.712")
        self.assertAlmostEqual(propsys.mech_nac_dia, 0.879, places=0, msg="Should be about 0.879")
        self.assertAlmostEqual(propsys.elec_nac_dia, 0.879, places=0, msg="Should be about 0.879")
        self.assertAlmostEqual(propsys.areas_wetted_mech, 6.64, places=0, msg="Should be about 6.64")
        self.assertAlmostEqual(propsys.areas_wetted_elec, 1.66, places=0, msg="Should be about 1.66")

if __name__ == '__main__':
    unittest.main()
