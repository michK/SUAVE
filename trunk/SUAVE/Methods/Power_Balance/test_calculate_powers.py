import unittest
import numpy as np

from calculate_powers import calculate_powers

class TestCalculatePowers(unittest.TestCase):

    def test_calculate_powers(self):

        PKtot = 300e3
        fS = 0.8
        fL = 0.5
        eta_pe = 0.98 
        eta_mot = 0.95
        eta_fan = 0.9

        PKm, PKe, Pturb, Pbat, PfanM, PfanE, Pmot, Pinv, Plink = calculate_powers(PKtot, fS, fL, eta_pe, eta_mot, eta_fan)

        print("PKm: {}".format(PKm))
        print("PKe: {}".format(PKe))
        print("PfanM: {}".format(PfanM))
        print("PfanE: {}".format(PfanE))
        print("Pmot: {}".format(Pmot))
        print("Pinv: {}".format(Pinv))
        print("Pbat: {}".format(Pbat))
        print("Pturb: {}".format(Pturb))
        print("Plink: {}".format(Plink))

        # Check model results:
        # Assertion format:
        # self.assertAlmostEqual(variable, expected_value, places=0, msg="Should be about expected_value")

        self.assertAlmostEqual(PKm[0], (1-fL)*PKtot, places=0, msg="Should be {}".format((1-fL)*PKtot))
        self.assertAlmostEqual(PKe[0], fL*PKtot, places=0, msg="Should be {}".format(fL*PKtot))
        self.assertAlmostEqual(PfanM[0], PKm[0]/eta_fan, places=0, msg="Should be {}".format(PKm[0]/eta_fan))
        self.assertAlmostEqual(PfanE[0], PKe[0]/eta_fan, places=0, msg="Should be {}".format(PKe[0]/eta_fan))
        self.assertAlmostEqual(Pmot[0], PfanE[0]/eta_mot, places=0, msg="Should be {}".format(PfanE[0]/eta_mot))
        self.assertAlmostEqual(Pinv[0], Pmot[0]/eta_pe, places=0, msg="Should be {}".format(Pmot[0]/eta_pe))

if __name__ == '__main__':
    unittest.main()
