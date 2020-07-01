## @ingroup Methods-Geometry-Two_Dimensional-Cross_Section-Airfoil
# import_airfoil_geometry.py
# 
# Created:  Mar 2019, M. Clarke
#           Mar 2020, M. Clarke
#           Apr 2020, M. Clarke
#           Apr 2020, M. Clarke
#           May 2020, B. Dalman

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from SUAVE.Core import Data  
import numpy as np
import scipy.interpolate as interp

## @ingroup Methods-Geometry-Two_Dimensional-Cross_Section-Airfoil
def  import_airfoil_geometry(airfoil_geometry_files):
    """This imports an airfoil geometry from a text file  and stores
    the coordinates of upper and lower surfaces as well as the mean
    camberline
    
    Assumptions:
    Works for Selig and Lednicer airfoil formats. Automatically detects which format based off first line of data. Assumes it is one of those two.

    Source:
    airfoiltools.com/airfoil/index - method for determining format and basic error checking

    Inputs:
    airfoil_geometry_files   <list of strings>

    Outputs:
    airfoil_data.
        thickness_to_chord 
        x_coordinates 
        y_coordinates
        x_upper_surface
        x_lower_surface
        y_upper_surface
        y_lower_surface
        camber_coordinates  

    Properties Used:
    N/A
    """      
 
    num_airfoils = len(airfoil_geometry_files)
    # unpack      

    airfoil_data                    = Data()
    airfoil_data.x_coordinates      = []
    airfoil_data.y_coordinates      = []
    airfoil_data.thickness_to_chord = []
    airfoil_data.camber_coordinates = []
    airfoil_data.x_upper_surface    = []
    airfoil_data.x_lower_surface    = []
    airfoil_data.y_upper_surface    = []
    airfoil_data.y_lower_surface    = []
    
    for i in range(num_airfoils):  
        # Open file and read column names and data block
        f = open(airfoil_geometry_files[i]) 

        # Ignore header comment
        f.readline()

        # Check if it's a Selig or Lednicer file
        format_line = f.readline()
        format_flag = float(format_line.strip().split()[0])
        format_extra_data = float(format_line.strip().split()[1])

        if format_flag > 1.01: # Amount of wiggle room per airfoil tools
            lednicer_format = True
        else:
            lednicer_format = False

            

        if lednicer_format:

            # Ignore last line of header
            f.readline()    

            data_block = f.readlines()
            f.close() 
            
            x_up_surf = []
            y_up_surf = []
            x_lo_surf = []
            y_lo_surf = []
            
            # Loop through each value: append to each column
            upper_surface_flag = True
            for line_count , line in enumerate(data_block): 
                #check for blank line which signifies the upper/lower surface division 
                line_check = data_block[line_count].strip()
                if line_check == '':
                    upper_surface_flag = False
                    continue
                if upper_surface_flag:
                    x_up_surf.append(float(data_block[line_count].strip().split()[0])) 
                    y_up_surf.append(float(data_block[line_count].strip().split()[1])) 
                else:                              
                    x_lo_surf.append(float(data_block[line_count].strip().split()[0])) 
                    y_lo_surf.append(float(data_block[line_count].strip().split()[1]))   

        else:
            data_block = f.readlines()
            f.close()

            x_up_surf_rev = []
            y_up_surf_rev = []
            x_lo_surf = []
            y_lo_surf = []

            # Loop through each value: append to each column
            upper_surface_flag = True
            for line_count , line in enumerate(data_block): 
                #check for line which starts with 0., which should be the split between upper and lower in selig
                line_check = data_block[line_count].strip()
                if float(line_check.split()[0]) == 0.:
                    x_up_surf_rev.append(float(data_block[line_count].strip().split()[0])) 
                    y_up_surf_rev.append(float(data_block[line_count].strip().split()[1]))

                    x_lo_surf.append(float(data_block[line_count].strip().split()[0])) 
                    y_lo_surf.append(float(data_block[line_count].strip().split()[1])) 

                    upper_surface_flag = False
                    continue

                if upper_surface_flag:
                    x_up_surf_rev.append(float(data_block[line_count].strip().split()[0])) 
                    y_up_surf_rev.append(float(data_block[line_count].strip().split()[1])) 
                else:                              
                    x_lo_surf.append(float(data_block[line_count].strip().split()[0])) 
                    y_lo_surf.append(float(data_block[line_count].strip().split()[1]))

            # Upper surface values in Selig format are reversed from Lednicer format, so fix that

            x_up_surf_rev.reverse()
            y_up_surf_rev.reverse()

            x_up_surf = x_up_surf_rev
            y_up_surf = y_up_surf_rev

            # Add back data from first line, that was used to check format

            x_up_surf.append(float(format_line.strip().split()[0]))
            y_up_surf.append(float(format_line.strip().split()[1]))

        
        # determine the thickness to chord ratio - note that the upper and lower surface
        # may be of different lenghts so initial interpolation is required 
        # x coordinates
        x_up_surf_new = np.array(x_up_surf)     
        arrx          = np.array(x_lo_surf) 
        arrx_interp   = interp.interp1d(np.arange(arrx.size),arrx)
        x_lo_surf_new = arrx_interp(np.linspace(0,arrx.size-1,x_up_surf_new.size)) 
        
        # y coordinates 
        y_up_surf_new = np.array(y_up_surf)  
        arry          = np.array(y_lo_surf)
        arry_interp   = interp.interp1d(np.arange(arry.size),arry)
        y_lo_surf_new = arry_interp(np.linspace(0,arry.size-1,y_up_surf_new.size)) 
         
        # compute thickness, camber and concatenate coodinates 
        thickness     = y_up_surf_new - y_lo_surf_new
        camber        = y_lo_surf_new + thickness/2 
        x_data        = np.concatenate([x_up_surf_new[::-1],x_lo_surf_new])
        y_data        = np.concatenate([y_up_surf_new[::-1],y_lo_surf_new]) 
        
        airfoil_data.thickness_to_chord.append(np.max(thickness))    
        airfoil_data.x_coordinates.append(x_data)  
        airfoil_data.y_coordinates.append(y_data)     
        airfoil_data.x_upper_surface.append(x_up_surf_new)
        airfoil_data.x_lower_surface.append(x_lo_surf_new)
        airfoil_data.y_upper_surface.append(y_up_surf_new)
        airfoil_data.y_lower_surface.append(y_lo_surf_new)          
        airfoil_data.camber_coordinates.append(camber)

    return airfoil_data 