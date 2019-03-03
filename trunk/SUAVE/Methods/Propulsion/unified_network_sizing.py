# unified_network_sizing.py
#
# Created:  Feb 2018, M. Kruger
# Modified:

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
from SUAVE.Methods.Power_Balance.calculate_powers import calculate_powers, remove_negatives

# ----------------------------------------------------------------------
#   Sizing
# ----------------------------------------------------------------------


def unified_network_sizing(net, vec):
    """ size unified network
    """

    PK_tot = vec.PKtot
    fS = net.fS
    fL = net.fL
    eta_pe = net.inverter.efficiency
    eta_mot = net.motor.efficiency
    eta_fan = net.fan_elec.efficiency

    [PKe, PKm, PfanE, PfanM, Pmot, Pinv, Pbat, Pturb, Pmot_link, Pconv, Plink] = \
        calculate_powers(PK_tot, fS, fL, eta_pe, eta_mot, eta_fan)

    net.PKe_max = PKe
    net.PKm_max = PKm
    net.PfanE_max = PfanE
    net.PfanM_max = PfanM
    net.Pmot_max = Pmot
    net.Pinv_max = Pinv
    net.Pbat_max = Pbat
    net.Pturb_max = Pturb
    net.Pmot_link_max = Pmot_link
    net.Pconv_max = Pconv
    net.Plink_max = Plink

    return
