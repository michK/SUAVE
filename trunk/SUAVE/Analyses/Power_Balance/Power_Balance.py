## @ingroup Analyses-Power_Balance
# Power_Balance.py
#
# Created:  Oct 2018, Michael Kruger
# Modified: 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import SUAVE
from SUAVE.Core import Data
from SUAVE.Analyses import Analysis


# ----------------------------------------------------------------------
#  Analysis
# ----------------------------------------------------------------------

## @ingroup Analyses-Power_Balance
class Power_Balance(Analysis):
    """ SUAVE.Analyses.Power.Power_Balance()
    This class evaluates the aircraft power requirements based on the power balance method
    """
    def __defaults__(self):
        """This sets the default values and methods for the analysis.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            N/A
        """
        self.tag      = 'power_balance'
        self.features = Data()
        self.settings = Data()
        
        
    def evaluate(self, state_sizing):
        """Evaluate the power balance analysis.
    
        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        Results of the Power_Balance analysis

        Properties Used:
        N/A                
        """
        # unpack        
        vehicle  = self.vehicle

        power    = SUAVE.Methods.Power_Balance.Power_Balance

        # evaluate
        results = power(vehicle, state_sizing)

        # storing weight breakdown into vehicle
        vehicle.power_balance = results

        # done!
        return results

    def finalize(self):
        """Finalize the power balance analysis.
    
        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        N/A
        """
        
        return

    __call__ = evaluate
