## @ingroup Input_Output-Results
# print_unified_system_weights.py 

# Created: M. Kruger
# Updated:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import datetime
from .identify_architecture import identify_arch

# ----------------------------------------------------------------------
#  Print output file with weight breakdown
# ----------------------------------------------------------------------
## @ingroup Input_Output-Results
def print_unified_system_info(vehicle, filename = 'unified_sys_info.dat'):
    """
    Based on print_weight_breakdown in print_weights.py, but prints breakdown
    of unified model component weights

    Assumptions:
    One propulsor (can be multiple engines) with 'unified' arch_tag.

    Source:
    N/A

    Inputs:
    vehicle
    filename (optional)  <string> Determines the name of the saved file

    Outputs:
    filename              Saved file with name as above

    Properties Used:
    N/A
    """     
    #unpack
    propsys = vehicle.propulsors.unified_propsys
    propsys_info = vehicle.propulsors.unified_propsys.info
    nr_fans_mech = propsys_info.nr_fans_mech
    nr_fans_elec = propsys_info.nr_fans_elec

    
    # start printing
    fid = open(filename,'w')   # Open output file
    fid.write('Output file with unified propulsion system information\n\n') #Start output printing
    fid.write(' AIRCRAFT PROPULSION SYSTEM ARCHITECTURE \n')
    fid.write(' Max fS = {}\n'.format(propsys.info.fS_max))
    fid.write(' Max fS segment = {}\n'.format(propsys.info.fS_max_segment))
    fid.write(' Max fL = {}\n'.format(propsys.info.fL_max))
    fid.write(' Max fL segment = {}\n'.format(propsys.info.fL_max_segment))
    fid.write(' ARCHITECTURE:................................... :{}\n\n'.format(identify_arch(propsys)))
    fid.write(' COMPONENT WEIGHTS \n')
    fid.write(' Core ........................................... : ' + str( '%8.2F' % (nr_fans_mech * propsys_info.m_core))        + ' kg\n')
    fid.write(' Mechanical fans ................................ : ' + str( '%8.2F' % (nr_fans_mech * propsys_info.m_fanm))        + ' kg\n')
    fid.write(' Mechanical nacelles ............................ : ' + str( '%8.2F' % (nr_fans_mech * propsys_info.m_nacm))        + ' kg\n')
    fid.write(' Link electrical machine ........................ : ' + str( '%8.2F' % (nr_fans_mech * propsys_info.m_gen))         + ' kg\n')
    fid.write(' Link power electronics ......................... : ' + str( '%8.2F' % (nr_fans_mech * propsys_info.m_pe_link))     + ' kg\n')
    fid.write(' Electrical fans ................................ : ' + str( '%8.2F' % (nr_fans_elec * propsys_info.m_fane))        + ' kg\n')
    fid.write(' Electrical nacelles ............................ : ' + str( '%8.2F' % (nr_fans_elec * propsys_info.m_nace))        + ' kg\n')
    fid.write(' Electrical propulsor motors .................... : ' + str( '%8.2F' % (nr_fans_elec * propsys_info.m_prop_mot))    + ' kg\n')
    fid.write(' Electrical propulsor power electronics ......... : ' + str( '%8.2F' % (nr_fans_elec * propsys_info.m_pe_prop_mot)) + ' kg\n')
    fid.write(' Thermal management system ...................... : ' + str( '%8.2F' % propsys_info.mass_tms)                       + ' kg\n')
    fid.write(' Additional system items ........................ : ' + str( '%8.2F' % propsys_info.m_add)                          + ' kg\n')
    fid.write('\n')
    fid.write(' System weight factor............................ : ' + str( '%8.2F' % propsys_info.weight_factor) + ' \n')
    fid.write('\n')
    fid.write(' Total propulsion system weight.................. : ' + str( '%8.2F' % propsys_info.weight_total)  + ' kg\n')

    fid.write(' \n')
    fid.write(' PROPULSOR COUNT, DIAMETERS AND WETTED AREAS \n')
    fid.write(' Number of mechanical fans................. : ' + str( '%8.0F' % propsys_info.nr_fans_mech)             + '\n')
    fid.write(' Number of electrical fans................. : ' + str( '%8.0F' % propsys_info.nr_fans_elec)             + '\n')
    fid.write(' Mechanical fan diameters.................. : ' + str( '%8.3F' % propsys_info.mech_fan_dia)             + ' m\n')
    fid.write(' Electrical fan diameters.................. : ' + str( '%8.3F' % propsys_info.elec_fan_dia)             + ' m\n')
    fid.write(' Mechanical nacelle wetted areas (total)... : ' + str( '%8.3F' % propsys_info.areas_wetted_mech_tot)    + ' m^2\n')
    fid.write(' Electrical nacelle wetted areas (total)... : ' + str( '%8.3F' % propsys_info.areas_wetted_elec_tot)    + ' m^2\n')
    fid.write(' Mechanical pylons wetted areas (total).... : ' + str( '%8.3F' % propsys_info.areas_wetted_mech_pylons) + ' m^2\n')
    fid.write(' Electrical pylons wetted areas (total).... : ' + str( '%8.3F' % propsys_info.areas_wetted_elec_pylons) + ' m^2\n')

    # Print timestamp
    fid.write('\n'+ 43*'-'+ '\n' + datetime.datetime.now().strftime(" %A, %d. %B %Y %I:%M:%S %p"))
    # done
    fid.close

# ----------------------------------------------------------------------
#   Module Test
# ----------------------------------------------------------------------
if __name__ == '__main__':
    print(' Error: No test defined ! ')    
