## @ingroup Components-Energy-Converters
# Propeller_Lo_Fid.py
#
# Created:  Jun 2014, E. Botero
# Modified: Jan 2016, T. MacDonald
#           Nov 2018, M. Kruger

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from warnings import warn

# ----------------------------------------------------------------------
#  Propeller Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class Propeller_Lo_Fid(Energy_Component):
    """This is a low-fidelity propeller component.

    Assumptions:
    None

    Source:
    None
    """
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        None
        """
        self.tip_radius            = 0.0
        self.propulsive_efficiency = 0.0


    def spin(self, conditions, power=0.0):
        """Analyzes a propeller given geometry and operating conditions.

        Assumptions:
        per source

        Source:
        Qprop theory document

        Inputs:
        self.inputs.omega            [radian/s]
        self.inputs.torque           [Nm]
        conditions.freestream.
          density                    [kg/m^3]
          dynamic_viscosity          [kg/(m-s)]
          velocity                   [m/s]
          speed_of_sound             [m/s]
          temperature                [K]

        Outputs:
        conditions.propulsion.etap   [-]  (propulsive efficiency)
        thrust                       [N]
        Qm                           [Nm] (torque)
        power                        [W]
        Cp                           [-]  (coefficient of power)

        Properties Used:
        self.tip_radius              [m]
        self.propulsive_efficiency   [-]
        """

        # Unpack
        R     = self.tip_radius
        etap  = self.propulsive_efficiency
        rho   = conditions.freestream.density[:,0,None]
        mu    = conditions.freestream.dynamic_viscosity[:,0,None]
        V     = conditions.freestream.velocity[:,0,None]
        a     = conditions.freestream.speed_of_sound[:,0,None]
        T     = conditions.freestream.temperature[:,0,None]

        try:
            omega = self.inputs.omega
            Qm    = self.inputs.torque
            power  = Qm*omega
            n      = omega/(2.*np.pi)
        except AttributeError:
            power = self.power

        # Do very little calculations
        D      = 2*R
        thrust = etap*power/V
        Cp     = power/(rho*(n*n*n)*(D*D*D*D*D))
        conditions.propulsion.etap = etap

        #pack the computed quantities into outputs
        self.outputs.thrust  = thrust
        self.outputs.torque  = Qm
        self.outputs.power  = power * etap
        self.outputs.power_coefficient  = Cp

        return thrust, Qm, power, Cp
