## @ingroup Components-Landing_Gear
# Landing_Gear.py
# 
# Created:  Aug 2015, C. R. I. da Silva
# Modified: Feb 2016, T. MacDonald

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Data
from SUAVE.Components import Physical_Component

# ----------------------------------------------------------------------
#  A ttribute
# ----------------------------------------------------------------------
## @ingroup Components-Landing_Gear
class Landing_Gear(Physical_Component):
    """ SUAVE.Components.Landing_Gear.Landing_Gear()
        
        The Top Landing Gear Component Class
        
            Assumptions:
            None
            
            Source:
            N/A
    
    """

    def __defaults__(self):
        """ This sets the default values for the component attributes.
        
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
       
        self.tag = 'landing_gear'
        self.main = Data()
        self.nose = Data()
        self.main.strut_length = 0.0
        self.nose.strut_length = 0.0

        self.main.mass_properties = Data()
        self.nose.mass_properties = Data()
        self.main.mass_properties.mass = 0.0
        self.nose.mass_properties.mass = 0.0


# ----------------------------------------------------------------------
#   Unit Tests
# ----------------------------------------------------------------------
# this will run from command line, put simple tests for your code here
if __name__ == '__main__':
    raise RuntimeError('test failed, not implemented')