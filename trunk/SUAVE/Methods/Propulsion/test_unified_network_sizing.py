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
        propsys.fBLIm = 0.0

        vehicle.append_component(propsys)

        # Define vehicle input variables
        vehicle.mdottot_cruise = 500 * Units['kg/s']
        vehicle.fL_cruise = 0.5
        vehicle.cruise_mach = 0.23943836
        vehicle.cruise_mach = 0.6

        vehicle.nr_engines_mech = propsys.number_of_engines_mech
        vehicle.nr_engines_elec = propsys.number_of_engines_elec

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

        propsys = unified_network_sizing(propsys, vehicle)

        print("propsys.mech_fan_dia: {}".format(propsys.mech_fan_dia))
        print("propsys.elec_fan_dia: {}".format(propsys.elec_fan_dia))
        print("propsys.mech_nac_dia: {}".format(propsys.mech_nac_dia))
        print("propsys.elec_nac_dia: {}".format(propsys.elec_nac_dia))
        print("propsys.areas_wetted_mech: {}".format(propsys.areas_wetted_mech))
        print("propsys.areas_wetted_elec: {}".format(propsys.areas_wetted_elec))

        self.assertAlmostEqual(propsys.mech_fan_dia, 0.986, places=3, msg="Should be about 0.986")
        self.assertAlmostEqual(propsys.elec_fan_dia, 0.697, places=3, msg="Should be about 0.697")
        self.assertAlmostEqual(propsys.mech_nac_dia, 1.233, places=3, msg="Should be about 1.232")
        self.assertAlmostEqual(propsys.elec_nac_dia, 0.872, places=3, msg="Should be about 0.872")
        self.assertAlmostEqual(propsys.areas_wetted_mech, 7.879, places=3, msg="Should be about 7.878")
        self.assertAlmostEqual(propsys.areas_wetted_elec, 1.970, places=3, msg="Should be about 1.96")

if __name__ == '__main__':
    unittest.main()
