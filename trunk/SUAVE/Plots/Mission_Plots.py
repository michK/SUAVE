## @ingroup Plots
# Mission_Plots.py
# 
# Created:  Mar 2020, M. Clarke
#           Apr 2020, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units 
import matplotlib.pyplot as plt  
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Ellipse, Polygon
import colorsys 
import matplotlib.animation as animation
import matplotlib as mpl
import matplotlib.cm as cm
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker 
# ------------------------------------------------------------------
#   Altitude, SFC & Weight
# ------------------------------------------------------------------
## @ingroup Plots
def plot_altitude_sfc_weight(results, line_color = 'bo-', save_figure = False, save_filename = "Altitude_SFC_Weight" , file_type = ".png"):
    """This plots the altitude, speficic fuel comsumption and vehicle weight 

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.condtions.
        freestream.altitude
        weights.total_mass
        weights.vehicle_mass_rate
        frames.body.thrust_force_vector

    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	  
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(10, 8) 
    for segment in results.segments.values(): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min 
        mass     = segment.conditions.weights.total_mass[:,0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:,0] / Units.ft
        mdot     = segment.conditions.weights.vehicle_mass_rate[:,0]
        thrust   =  segment.conditions.frames.body.thrust_force_vector[:,0]
        sfc      = (mdot / Units.lb) / (thrust /Units.lbf) * Units.hr

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , altitude , line_color)
        axes.set_ylabel('Altitude (ft)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , sfc , line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('sfc (lb/lbf-hr)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , mass , 'ro-' )
        axes.set_ylabel('Weight (lb)',axis_font)
        set_axes(axes)
        
    if save_figure:
        plt.savefig(save_filename + file_type)  
        
    return

# ------------------------------------------------------------------
#   Aircraft Velocities
# ------------------------------------------------------------------
## @ingroup Plots
def plot_aircraft_velocities(results, line_color = 'bo-', save_figure = False, save_filename = "Aircraft_Velocities", file_type = ".png"):
    """This plots aircraft velocity, mach , true air speed 

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.condtions.freestream.
        velocity
        density
        mach_number 

    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	
    axis_font = {'size':'14'}  
    fig = plt.figure(save_filename)
    fig.set_size_inches(10, 8) 
    for segment in results.segments.values(): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min 
        velocity = segment.conditions.freestream.velocity[:,0]
        pressure = segment.conditions.freestream.pressure[:,0]
        density  = segment.conditions.freestream.density[:,0]
        EAS      = velocity * np.sqrt(density/1.225)
        mach     = segment.conditions.freestream.mach_number[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , velocity / Units.kts, line_color)
        axes.set_ylabel('velocity (kts)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , EAS / Units.kts, line_color)
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Equivalent Airspeed',axis_font)
        set_axes(axes)    
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach , line_color)
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Mach',axis_font)
        set_axes(axes)  
        
    if save_figure:
        plt.savefig(save_filename + file_type) 
        
    return

# ------------------------------------------------------------------
#   Disc and Power Loadings
# ------------------------------------------------------------------
## @ingroup Plots
def plot_disc_power_loading(results, line_color = 'bo-', save_figure = False, save_filename = "Disc_Power_Loading", file_type = ".png"):
    """This plots the propeller disc and power loadings

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.condtions.propulsion.
        disc_loadings
        power_loading 
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	   
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10) 
    
    for i in range(len(results.segments)): 
        time  = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        DL    = results.segments[i].conditions.propulsion.disc_loading
        PL    = results.segments[i].conditions.propulsion.power_loading   
   
        axes = fig.add_subplot(2,1,1)
        axes.plot(time, DL, line_color)
        axes.set_ylabel('lift disc power N/m^2',axis_font)
        set_axes(axes)      
  
        axes = fig.add_subplot(2,1,2)
        axes.plot(time, PL, line_color )       
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('lift power loading (N/W)',axis_font)
        set_axes(axes)       

    if save_figure:
        plt.savefig(save_filename + file_type)          
        
    return


# ------------------------------------------------------------------
#   Aerodynamic Coefficients
# ------------------------------------------------------------------
## @ingroup Plots
def plot_aerodynamic_coefficients(results, line_color = 'bo-', save_figure = False, save_filename = "Aerodynamic_Coefficients", file_type = ".png"):
    """This plots the aerodynamic coefficients 

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.condtions.aerodynamics.
        lift_coefficient
        drag_coefficient
        angle_of_attack
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	    
    axis_font = {'size':'14'}  
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)
    
    for segment in results.segments.values(): 
        time = segment.conditions.frames.inertial.time[:,0] / Units.min
        cl   = segment.conditions.aerodynamics.lift_coefficient[:,0,None] 
        cd   = segment.conditions.aerodynamics.drag_coefficient[:,0,None] 
        aoa  = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d  = cl/cd

        axes = fig.add_subplot(2,2,1)
        axes.plot( time , aoa , line_color )
        axes.set_ylabel('Angle of Attack (deg)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , cl, line_color )
        axes.set_ylabel('CL',axis_font)
        set_axes(axes)   
        
        axes = fig.add_subplot(2,2,3)
        axes.plot( time , cd, line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('CD',axis_font)
        set_axes(axes)   
        
        axes = fig.add_subplot(2,2,4)
        axes.plot( time , l_d, line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('L/D',axis_font)
        set_axes(axes)            
                
    if save_figure:
        plt.savefig(save_filename + file_type) 
        
    return

# ------------------------------------------------------------------
#   Aerodynamic Forces
# ------------------------------------------------------------------
## @ingroup Plots
def plot_aerodynamic_forces(results, line_color = 'bo-', save_figure = False, save_filename = "Aerodynamic_Forces", file_type = ".png"):
    """This plots the aerodynamic forces

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.condtions.frames
         body.thrust_force_vector 
         wind.lift_force_vector        
         wind.drag_force_vector   
         
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	   
    axis_font = {'size':'14'}  
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)
    
    for segment in results.segments.values():
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]  
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]          
        eta    = segment.conditions.propulsion.throttle[:,0]

        axes = fig.add_subplot(2,2,1)
        axes.plot( time , eta , line_color )
        axes.set_ylabel('Throttle',axis_font)
        set_axes(axes)	 

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , Lift , line_color)
        axes.set_ylabel('Lift (N)',axis_font)
        set_axes(axes)
        
        axes = fig.add_subplot(2,2,3)
        axes.plot( time , Thrust , line_color)
        axes.set_ylabel('Thrust (N)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)
        
        axes = fig.add_subplot(2,2,4)
        axes.plot( time , Drag , line_color)
        axes.set_ylabel('Drag (N)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)       
    
    if save_figure:
        plt.savefig(save_filename + file_type) 
            
    return

# ------------------------------------------------------------------
#   Drag Components
# ------------------------------------------------------------------
## @ingroup Plots
def plot_drag_components(results, line_color = 'bo-', save_figure = False, save_filename = "Drag_Components", file_type = ".png"):
    """This plots the drag components of the aircraft

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.condtions.aerodynamics.drag_breakdown
          parasite.total 
          induced.total 
          compressible.total    
          miscellaneous.total     
    
    Outputs: 
    Plots
    
    Properties Used:
    N/A	
    """	  
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename,figsize=(8,10))
    axes = plt.gca()
    
    for i, segment in enumerate(results.segments.values()):
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]
        
        if i == 0:
            axes.plot( time , cdp , 'ko-', label='CD parasite' )
            axes.plot( time , cdi , line_color, label='CD induced' )
            axes.plot( time , cdc , 'go-', label='CD compressibility' )
            axes.plot( time , cdm , 'yo-', label='CD miscellaneous' )
            axes.plot( time , cd  , 'ro-', label='CD total'   )    
            axes.legend(loc='upper center')   
        else:
            axes.plot( time , cdp , 'ko-')
            axes.plot( time , cdi , line_color)
            axes.plot( time , cdc , 'go-')
            axes.plot( time , cdm , 'yo-')
            axes.plot( time , cd  , 'ro-') 
            
    axes.set_xlabel('Time (min)',axis_font)
    axes.set_ylabel('CD',axis_font)
    axes.grid(True)         
    
    if save_figure:
        plt.savefig(save_filename + file_type) 
        
    return


# ------------------------------------------------------------------
#   Electronic Conditions
# ------------------------------------------------------------------
## @ingroup Plots
def plot_electronic_conditions(results, line_color = 'bo-', save_figure = False, save_filename = "Electronic_Conditions", file_type = ".png"):
    """This plots the electronic conditions of the network

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.conditions.propulsion
         battery_draw 
         battery_energy    
         voltage_under_load    
         voltage_open_circuit    
         current        
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	  
    
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)
    
    for i in range(len(results.segments)):     
        time           = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        power          = results.segments[i].conditions.propulsion.battery_draw[:,0] 
        energy         = results.segments[i].conditions.propulsion.battery_energy[:,0] 
        volts          = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc       = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]     
        current        = results.segments[i].conditions.propulsion.current[:,0]      
        battery_amp_hr = (energy/ Units.Wh )/volts  
        C_rating       = current/battery_amp_hr
        
        axes = fig.add_subplot(2,2,1)
        axes.plot(time, -power, line_color)
        axes.set_ylabel('Battery Power (Watts)',axis_font)
        set_axes(axes)       
    
        axes = fig.add_subplot(2,2,2)
        axes.plot(time, energy/ Units.Wh, line_color)
        axes.set_ylabel('Battery Energy (W-hr)',axis_font)
        set_axes(axes)  
    
        axes = fig.add_subplot(2,2,3)
        axes.plot(time, volts, 'bo-',label='Under Load')
        axes.plot(time,volts_oc, 'ks--',label='Open Circuit')
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('Battery Voltage (Volts)',axis_font)  
        set_axes(axes) 
        if i == 0:
            axes.legend(loc='upper right')          
                
        
        axes = fig.add_subplot(2,2,4)
        axes.plot(time, C_rating, line_color)
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('C-Rating (C)',axis_font)  
        set_axes(axes)
 
    if save_figure:
        plt.savefig(save_filename + file_type)       
        
    return


# ------------------------------------------------------------------
#   Flight Conditions
# ------------------------------------------------------------------
## @ingroup Plots
def plot_flight_conditions(results, line_color = 'bo-', save_figure = False, save_filename = "Flight_Conditions", file_type = ".png"):
    """This plots the flights the conditions 

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.conditions.
         frames 
             body.inertial_rotations
             inertial.position_vector 
         freestream.velocity
         aerodynamics.
             lift_coefficient
             drag_coefficient
             angle_of_attack
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	    
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)
    for segment in results.segments.values(): 
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        airspeed = segment.conditions.freestream.velocity[:,0] 
        theta    = segment.conditions.frames.body.inertial_rotations[:,1,None] / Units.deg
        cl       = segment.conditions.aerodynamics.lift_coefficient[:,0,None] 
        cd       = segment.conditions.aerodynamics.drag_coefficient[:,0,None] 
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        
        x        = segment.conditions.frames.inertial.position_vector[:,0]
        y        = segment.conditions.frames.inertial.position_vector[:,1]
        z        = segment.conditions.frames.inertial.position_vector[:,2]
        altitude = segment.conditions.freestream.altitude[:,0]
        
        axes = fig.add_subplot(2,2,1)
        axes.plot(time, altitude, line_color)
        axes.set_ylabel('Altitude (m)',axis_font)
        set_axes(axes)            

        axes = fig.add_subplot(2,2,2)
        axes.plot( time , airspeed , line_color )
        axes.set_ylabel('Airspeed (m/s)',axis_font)
        set_axes(axes)

        axes = fig.add_subplot(2,2,3)
        axes.plot( time , theta, line_color )
        axes.set_ylabel('Pitch Angle (deg)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)   
        
        axes = fig.add_subplot(2,2,4)
        axes.plot( time , x, 'bo-', time , y, 'go-' , time , z, 'ro-')
        axes.set_ylabel('Range (m)',axis_font)
        axes.set_xlabel('Time (min)',axis_font)
        set_axes(axes)         
        
    if save_figure:
        plt.savefig(save_filename + file_type)
        
    return

# ------------------------------------------------------------------
#   Propulsion Conditions
# ------------------------------------------------------------------
## @ingroup Plots
def plot_propeller_conditions(results, line_color = 'bo-', save_figure = False, save_filename = "Propeller", file_type = ".png"):
    """This plots the propeller performance

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.conditions. 
        frames.inertial.time 
        propulsion.rpm 
        frames.body.thrust_force_vector 
        propulsion.motor_torque 
        propulsion.propeller_tip_mach 
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	 
    
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)  
    
    for segment in results.segments.values():  
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        rpm    = segment.conditions.propulsion.rpm[:,0] 
        thrust = segment.conditions.frames.body.thrust_force_vector[:,2]
        torque = segment.conditions.propulsion.motor_torque[:,0] 
        tm     = segment.conditions.propulsion.propeller_tip_mach[:,0]
 
        axes = fig.add_subplot(2,2,1)
        axes.plot(time, -thrust, line_color)
        axes.set_ylabel('Thrust (N)',axis_font)
        set_axes(axes)
        
        axes = fig.add_subplot(2,2,2)
        axes.plot(time, rpm, line_color)
        axes.set_ylabel('RPM',axis_font)
        set_axes(axes)
        
        axes = fig.add_subplot(2,2,3)
        axes.plot(time, torque, line_color )
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('Torque (N-m)',axis_font)
        set_axes(axes)  
        
        axes = fig.add_subplot(2,2,4)
        axes.plot(time, tm, line_color )
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('Tip Mach',axis_font)
        set_axes(axes)     
        
    if save_figure:
        plt.savefig(save_filename + file_type)  
            
    return

# ------------------------------------------------------------------
#   Electric Propulsion efficiencies
# ------------------------------------------------------------------
## @ingroup Plots
def plot_eMotor_Prop_efficiencies(results, line_color = 'bo-', save_figure = False, save_filename = "eMotor_Prop_Propulsor", file_type = ".png"):
    """This plots the electric driven network propeller efficiencies 

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.conditions.propulsion. 
         etap
         etam
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	   
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)  
    for segment in results.segments.values():
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        effp   = segment.conditions.propulsion.etap[:,0]
        effm   = segment.conditions.propulsion.etam[:,0]
        
        axes = fig.add_subplot(1,2,1)
        axes.plot(time, effp, line_color )
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('Propeller Efficiency (N-m)',axis_font)
        set_axes(axes)         
        plt.ylim((0,1))
        
        axes = fig.add_subplot(1,2,2)
        axes.plot(time, effm, line_color )
        axes.set_xlabel('Time (mins)',axis_font)
        axes.set_ylabel('Motor Efficiency (N-m)',axis_font)
        set_axes(axes)
        
    if save_figure:
        plt.savefig(save_filename + file_type)  
            
    return

# ------------------------------------------------------------------
#   Stability Coefficients
# ------------------------------------------------------------------
## @ingroup Plots
def plot_stability_coefficients(results, line_color = 'bo-', save_figure = False, save_filename = "Stability_Coefficients", file_type = ".png"):
    """This plots the static stability characteristics of an aircraft

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.conditions.stability.
       static
           CM 
           Cm_alpha 
           static_margin 
       aerodynamics.
           angle_of_attack
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """	    
    axis_font = {'size':'14'} 
    fig = plt.figure(save_filename)
    fig.set_size_inches(12, 10)
    
    for segment in results.segments.values():
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        cm       = segment.conditions.stability.static.CM[:,0]
        cm_alpha = segment.conditions.stability.static.Cm_alpha[:,0]
        SM       = segment.conditions.stability.static.static_margin[:,0]
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        
        axes = fig.add_subplot(2,2,1)
        axes.plot( time , aoa, line_color )
        axes.set_ylabel(r'$AoA$',axis_font)
        set_axes(axes)   
         
        axes = fig.add_subplot(2,2,2)
        axes.plot( time , cm, line_color )
        axes.set_ylabel(r'$C_M$',axis_font)
        set_axes(axes)  
        
        axes = fig.add_subplot(2,2,3)
        axes.plot( time , cm_alpha, line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel(r'$C_M\alpha$',axis_font)
        set_axes(axes)  
        
        axes = fig.add_subplot(2,2,4)
        axes.plot( time , SM, line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Static Margin (%)',axis_font)
        set_axes(axes)
    
    if save_figure:
        plt.savefig(save_filename + file_type)
        
    return

# ------------------------------------------------------------------    
#   Solar Flux
# ------------------------------------------------------------------
## @ingroup Plots
def plot_solar_flux(results, line_color = 'bo-', save_figure = False, save_filename = "Solar_Flux", file_type = ".png"):
    """This plots the solar flux and power train performance of an solar powered aircraft 

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.conditions.propulsion
        solar_flux 
        battery_draw 
        battery_energy 
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """
    
    axis_font = {'size':'14'} 
    fig       = plt.figure(save_filename) 
    fig.set_size_inches(8, 14)
    
    for segment in results.segments.values():               
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        flux   = segment.conditions.propulsion.solar_flux[:,0] 
        charge = segment.conditions.propulsion.battery_draw[:,0] 
        energy = segment.conditions.propulsion.battery_energy[:,0] / Units.MJ
    
        axes = fig.add_subplot(3,1,1)
        axes.plot( time , flux , line_color )
        axes.set_ylabel('Solar Flux (W/m$^2$)',axis_font)
        set_axes(axes)
    
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , charge , line_color )
        axes.set_ylabel('Charging Power (W)',axis_font)
        set_axes(axes)
    
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , energy , line_color )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Battery Energy (MJ)',axis_font)
        set_axes(axes)            
    
    if save_figure:
        plt.savefig(save_filename + file_type)
        
    return

# ------------------------------------------------------------------
#   Lift-Cruise Network
# ------------------------------------------------------------------
## @ingroup Plots
def plot_lift_cruise_network(results, line_color = 'bo-', save_figure = False, save_filename = "Lift_Cruise_Network", file_type = ".png"):
    """This plots the electronic and propulsor performance of a vehicle with a lift cruise network

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.conditions.propulsion
         throttle 
         rotor_throttle 
         battery_energy
         battery_specfic_power 
         voltage_under_load  
         voltage_open_circuit   
        
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """   
    axis_font = {'size':'14'} 
    # ------------------------------------------------------------------
    #   Electronic Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Lift_Cruise_Electric_Conditions")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time           = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        eta            = results.segments[i].conditions.propulsion.throttle[:,0]
        eta_l          = results.segments[i].conditions.propulsion.throttle_lift[:,0]
        energy         = results.segments[i].conditions.propulsion.battery_energy[:,0]/ Units.Wh
        specific_power = results.segments[i].conditions.propulsion.battery_specfic_power[:,0]
        volts          = results.segments[i].conditions.propulsion.voltage_under_load[:,0] 
        volts_oc       = results.segments[i].conditions.propulsion.voltage_open_circuit[:,0]  
                    
        axes = fig.add_subplot(2,2,1)
        axes.plot(time, eta, 'bo-',label='Forward Motor')
        axes.plot(time, eta_l, 'r^-',label='Lift Motors')
        axes.set_ylabel('Throttle')
        set_axes(axes)     
        plt.ylim((0,1))
        if i == 0:
            axes.legend(loc='upper center')         
    
        axes = fig.add_subplot(2,2,2)
        axes.plot(time, energy, 'bo-')
        axes.set_ylabel('Battery Energy (W-hr)')
        set_axes(axes)
    
        axes = fig.add_subplot(2,2,3)
        axes.plot(time, volts, 'bo-',label='Under Load')
        axes.plot(time,volts_oc, 'ks--',label='Open Circuit')
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Battery Voltage (Volts)')  
        set_axes(axes)
        
        if i == 0:
            axes.legend(loc='upper center')                
        
        axes = fig.add_subplot(2,2,4)
        axes.plot(time, specific_power, 'bo-') 
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Specific Power')  
        set_axes(axes)
        
    if save_figure:
        plt.savefig("Lift_Cruise_Electric_Conditions" + file_type)
    
   
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Prop-Rotor Network")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time         = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        prop_rpm     = results.segments[i].conditions.propulsion.rpm_forward[:,0] 
        prop_thrust  = results.segments[i].conditions.frames.body.thrust_force_vector[:,0]
        prop_torque  = results.segments[i].conditions.propulsion.motor_torque_forward
        prop_effp    = results.segments[i].conditions.propulsion.propeller_efficiency_forward[:,0]
        prop_effm    = results.segments[i].conditions.propulsion.motor_efficiency_forward[:,0]
        prop_Cp      = results.segments[i].conditions.propulsion.propeller_power_coefficient[:,0]
        rotor_rpm    = results.segments[i].conditions.propulsion.rpm_lift[:,0] 
        rotor_thrust = -results.segments[i].conditions.frames.body.thrust_force_vector[:,2]
        rotor_torque = results.segments[i].conditions.propulsion.motor_torque_lift[:,0]
        rotor_effp   = results.segments[i].conditions.propulsion.propeller_efficiency_lift[:,0]
        rotor_effm   = results.segments[i].conditions.propulsion.motor_efficiency_lift[:,0] 
        rotor_Cp     = results.segments[i].conditions.propulsion.propeller_power_coefficient_lift[:,0]        
    
        axes = fig.add_subplot(2,3,1)
        axes.plot(time, prop_rpm, 'bo-')
        axes.plot(time, rotor_rpm, 'r^-')
        axes.set_ylabel('RPM')
        set_axes(axes)      
    
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, prop_thrust, 'bo-')
        axes.plot(time, rotor_thrust, 'r^-')
        axes.set_ylabel('Thrust (N)')
        set_axes(axes)  
    
        axes = fig.add_subplot(2,3,3)
        axes.plot(time, prop_torque, 'bo-' )
        axes.plot(time, rotor_torque, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Torque (N-m)')
        set_axes(axes)
    
        axes = fig.add_subplot(2,3,4)
        axes.plot(time, prop_effp, 'bo-' )
        axes.plot(time, rotor_effp, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Propeller Efficiency, $\eta_{propeller}$')
        set_axes(axes)      
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,5)
        axes.plot(time, prop_effm, 'bo-' )
        axes.plot(time, rotor_effm, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Motor Efficiency, $\eta_{motor}$')
        set_axes(axes)       
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,6)
        axes.plot(time, prop_Cp, 'bo-' )
        axes.plot(time, rotor_Cp, 'r^-'  )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Power Coefficient')
        set_axes(axes)
        
    if save_figure:
        plt.savefig("Propulsor_Network" + file_type)
            
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Rotor")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time   = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        rpm    = results.segments[i].conditions.propulsion.rpm_lift [:,0] 
        thrust = results.segments[i].conditions.frames.body.thrust_force_vector[:,2]
        torque = results.segments[i].conditions.propulsion.motor_torque_lift
        effp   = results.segments[i].conditions.propulsion.propeller_efficiency_lift[:,0]
        effm   = results.segments[i].conditions.propulsion.motor_efficiency_lift[:,0] 
        Cp     = results.segments[i].conditions.propulsion.propeller_power_coefficient_lift[:,0]
    
        axes = fig.add_subplot(2,3,1)
        axes.plot(time, rpm, 'r^-')
        axes.set_ylabel('RPM')
        set_axes(axes)      
    
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, -thrust, 'r^-')
        axes.set_ylabel('Thrust (N)')
        set_axes(axes)
    
        axes = fig.add_subplot(2,3,3)
        axes.plot(time, torque, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Torque (N-m)')
        set_axes(axes)
    
        axes = fig.add_subplot(2,3,4)
        axes.plot(time, effp, 'r^-',label= r'$\eta_{rotor}$' ) 
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Propeller Efficiency $\eta_{rotor}$')
        set_axes(axes)    
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,5)
        axes.plot(time, effm, 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Motor Efficiency $\eta_{mot}$')
        set_axes(axes)
        plt.ylim((0,1))  
    
        axes = fig.add_subplot(2,3,6)
        axes.plot(time, Cp , 'r^-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Power Coefficient')
        set_axes(axes)            
    
    if save_figure:
        plt.savefig("Rotor" + file_type)  
        
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Propeller")
    fig.set_size_inches(16, 8)
    for i in range(len(results.segments)):          
        time   = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        rpm    = results.segments[i].conditions.propulsion.rpm_forward [:,0] 
        thrust = results.segments[i].conditions.frames.body.thrust_force_vector[:,0]
        torque = results.segments[i].conditions.propulsion.motor_torque_forward[:,0]
        effp   = results.segments[i].conditions.propulsion.propeller_efficiency_forward[:,0]
        effm   = results.segments[i].conditions.propulsion.motor_efficiency_forward[:,0]
        Cp     = results.segments[i].conditions.propulsion.propeller_power_coefficient[:,0]
    
        axes = fig.add_subplot(2,3,1)
        axes.plot(time, rpm, 'bo-')
        axes.set_ylabel('RPM')
        set_axes(axes)       
    
        axes = fig.add_subplot(2,3,2)
        axes.plot(time, thrust, 'bo-')
        axes.set_ylabel('Thrust (N)')
        set_axes(axes)   
    
        axes = fig.add_subplot(2,3,3)
        axes.plot(time, torque, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Torque (N-m)')
        set_axes(axes)  
    
        axes = fig.add_subplot(2,3,4)
        axes.plot(time, effp, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Propeller Efficiency $\eta_{propeller}$')
        set_axes(axes)            
        plt.ylim((0,1))
    
        axes = fig.add_subplot(2,3,5)
        axes.plot(time, effm, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel(r'Motor Efficiency $\eta_{motor}$')
        set_axes(axes) 
    
        axes = fig.add_subplot(2,3,6)
        axes.plot(time, Cp, 'bo-' )
        axes.set_xlabel('Time (mins)')
        axes.set_ylabel('Power Coefficient')
        set_axes(axes)  
        
    if save_figure:
        plt.savefig("Cruise_Propulsor" + file_type)
       
    # ------------------------------------------------------------------
    #   Propulsion Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Tip_Mach") 
    for i in range(len(results.segments)):          
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min 
        rtm  = results.segments[i].conditions.propulsion.propeller_tip_mach_lift[:,0]
        ptm  = results.segments[i].conditions.propulsion.propeller_tip_mach_forward[:,0] 
        
        axes = fig.add_subplot(1,1,1)
        axes.plot(time, ptm, 'bo-',label='Propeller')
        axes.plot(time, rtm, 'r^-',label='Rotor')
        axes.set_ylabel('Mach')
        set_axes(axes)   
        if i == 0:
            axes.legend(loc='upper center')     
    
    if save_figure:
        plt.savefig("Tip_Mach" + file_type) 
        
        
    return

# ------------------------------------------------------------------
#   Pressure Coefficient
# ------------------------------------------------------------------
def plot_surface_pressure_contours(results,vehicle, save_figure = False, save_filename = "Surface_Pressure", file_type = ".png"):
    """This plots the surface pressure distrubtion at all control points
    on all lifting surfaces of the aircraft

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.aerodynamics.
        pressure_coefficient
    vehicle.vortex_distribution.
       n_cw
       n_sw
       n_w
       
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """   
    VD         = vehicle.vortex_distribution	 
    n_cw       = VD.n_cw 	
    n_cw       = VD.n_cw 
    n_sw       = VD.n_sw 
    n_w        = VD.n_w 
    
    # Create a boolean for not plotting vertical wings
    idx        = 0
    plot_flag  = np.ones(n_w)
    for wing in vehicle.wings: 
        if wing.vertical: 
            plot_flag[idx] = 0 
            idx += 1    
        else:
            idx += 1 
        if wing.vertical and wing.symmetric:             
            plot_flag[idx] = 0 
            idx += 1
        else:
            idx += 1  
        
    img_idx    = 1	
    seg_idx    = 1	
    for segment in results.segments.values():   	
        num_ctrl_pts = len(segment.conditions.frames.inertial.time)	
        for ti in range(num_ctrl_pts):  
            CP         = segment.conditions.aerodynamics.pressure_coefficient[ti]
            
            fig        = plt.figure()	
            axes       = fig.add_subplot(1, 1, 1)  
            x_max      = max(VD.XC) + 2
            y_max      = max(VD.YC) + 2
            axes.set_ylim(x_max, 0)
            axes.set_xlim(-y_max, y_max)            
            fig.set_size_inches(12, 12)         	 
            for i in range(n_w):
                n_pts     = (n_sw + 1) * (n_cw + 1) 
                xc_pts    = VD.X[i*(n_pts):(i+1)*(n_pts)]
                x_pts     = np.reshape(np.atleast_2d(VD.XC[i*(n_sw*n_cw):(i+1)*(n_sw*n_cw)]).T, (n_sw,-1))
                y_pts     = np.reshape(np.atleast_2d(VD.YC[i*(n_sw*n_cw):(i+1)*(n_sw*n_cw)]).T, (n_sw,-1))
                z_pts     = np.reshape(np.atleast_2d(CP[i*(n_sw*n_cw):(i+1)*(n_sw*n_cw)]).T, (n_sw,-1))
                x_pts_p   = x_pts*((n_cw+1)/n_cw) - x_pts[0,0]*((n_cw+1)/n_cw)  +  xc_pts[0] 
                points    = np.linspace(0.001,1,50)
                A         = np.cumsum(np.sin(np.pi/2*points))
                levals    = -(np.concatenate([-A[::-1],A[1:]])/(2*A[-1])  + A[-1]/(2*A[-1]) )[::-1]*0.015  
                color_map = plt.cm.get_cmap('jet')
                rev_cm    = color_map.reversed()
                if plot_flag[i] == 1:
                    CS  = axes.contourf(y_pts,x_pts_p, z_pts, cmap = rev_cm,levels=levals,extend='both')    
                
            # Set Color bar	
            cbar = fig.colorbar(CS, ax=axes)
            cbar.ax.set_ylabel('$C_{P}$', rotation =  0)  
            plt.axis('off')	
            plt.grid(None)            
            
            if save_figure: 
                plt.savefig( save_filename + '_' + str(img_idx) + file_type) 	
            img_idx += 1	
        seg_idx +=1
        
    return   


# ------------------------------------------------------------------
#   Sectional Lift Distribution
# ------------------------------------------------------------------
def plot_lift_distribution(results,vehicle, save_figure = False, save_filename = "Sectional_Lift", file_type = ".png"):
    """This plots the sectional lift distrubtion at all control points
    on all lifting surfaces of the aircraft

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.aerodynamics.
        inviscid_wings_sectional_lift
    vehicle.vortex_distribution.
       n_sw
       n_w
       
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """   
    VD         = vehicle.vortex_distribution	 	
    n_sw       = VD.n_sw 
    n_w        = VD.n_w
    
    axis_font  = {'size':'12'}  	
    img_idx    = 1	
    seg_idx    = 1	
    for segment in results.segments.values():   	
        num_ctrl_pts = len(segment.conditions.frames.inertial.time)	
        for ti in range(num_ctrl_pts):  
            cl_y = segment.conditions.aerodynamics.lift_breakdown.inviscid_wings_sectional[ti] 
            line = ['-b','-b','-r','-r','-k']
            fig  = plt.figure()
            fig.set_size_inches(12, 12)       
            axes = fig.add_subplot(1,1,1)
            for i in range(n_w): 
                y_pts = VD.Y_SW[i*(n_sw):(i+1)*(n_sw)]
                z_pts = cl_y[i*(n_sw):(i+1)*(n_sw)]
                axes.plot(y_pts, z_pts, line[i] ) 
            axes.set_xlabel("Spanwise Location (m)",axis_font)
            axes.set_title('$C_{Ly}$',axis_font)  
            
            if save_figure: 
                plt.savefig( save_filename + '_' + str(img_idx) + file_type) 	
            img_idx += 1	
        seg_idx +=1
        
    return      
 
# ------------------------------------------------------------------
#   VLM Video 
# ------------------------------------------------------------------
def create_video_frames(results,vehicle, save_figure = True ,flight_profile = True,  save_filename = "Flight_Mission_Frame", file_type = ".png"):
    """This creates video frames of the aerodynamic conditions of the vehicle as well as the 
    surface pressure coefficient throughout a mission

    Assumptions:
    None

    Source:
    None

    Inputs:
    results.segments.
       aerodynamics.         
          lift_coefficient 	
          drag_coefficient 	
       conditions.
           freestream.altitude 
           weights.total_mass 
                   
    vehicle.vortex_distribution.
       n_cp
       n_cw 
       n_sw 
       n_w
       n_fus
       
    Outputs: 
    Plots

    Properties Used:
    N/A	
    """      
    VD         = vehicle.vortex_distribution	 
    n_cw       = VD.n_cw 	
    n_sw       = VD.n_sw 
    n_w        = VD.n_w
    n_fus      = VD.n_fus
    
    # Create a boolean for not plotting vertical wings
    idx        = 0
    plot_flag  = np.ones(n_w)
    for wing in vehicle.wings: 
        if wing.vertical: 
            plot_flag[idx] = 0 
            idx += 1    
        else:
            idx += 1 
        if wing.vertical and wing.symmetric:             
            plot_flag[idx] = 0 
            idx += 1
        else:
            idx += 1  
            
    axis_font  = {'size':'16'}  	
    img_idx    = 1	
    seg_idx    = 1	
    for segment in results.segments.values():   	
        num_ctrl_pts = len(segment.conditions.frames.inertial.time)	
        for ti in range(num_ctrl_pts):  
            CP         = segment.conditions.aerodynamics.pressure_coefficient[ti] 
            fig        = plt.figure(constrained_layout=True)
            fig.set_size_inches(12, 6.75)         
            gs         = fig.add_gridspec(4, 4) 
            axes       = fig.add_subplot(gs[:, :-1])
            
            x_max = max(VD.XC) + 2
            y_max = max(VD.YC) + 2
            axes.set_ylim(x_max, -2)
            axes.set_xlim(-y_max, y_max)    
            
            # plot wing CP distribution   
            for i in range(n_w):
                n_pts     = (n_sw + 1) * (n_cw + 1) 
                xc_pts    = VD.X[i*(n_pts):(i+1)*(n_pts)]
                x_pts     = np.reshape(np.atleast_2d(VD.XC[i*(n_sw*n_cw):(i+1)*(n_sw*n_cw)]).T, (n_sw,-1))
                y_pts     = np.reshape(np.atleast_2d(VD.YC[i*(n_sw*n_cw):(i+1)*(n_sw*n_cw)]).T, (n_sw,-1))
                z_pts     = np.reshape(np.atleast_2d(CP[i*(n_sw*n_cw):(i+1)*(n_sw*n_cw)]).T, (n_sw,-1))  
                x_pts_p   = x_pts*((n_cw+1)/n_cw) - x_pts[0,0]*((n_cw+1)/n_cw)  +  xc_pts[0]  
                points    = np.linspace(0.001,1,50)
                A         = np.cumsum(np.sin(np.pi/2*points))
                levals    = -(np.concatenate([-A[::-1],A[1:]])/(2*A[-1])  + A[-1]/(2*A[-1]) )[::-1]*0.015  
                color_map = plt.cm.get_cmap('jet')
                rev_cm    = color_map.reversed()
                if plot_flag[i] == 1:
                    CS    = axes.contourf( y_pts,x_pts_p, z_pts, cmap = rev_cm,levels=levals,extend='both')   
                
            # Set Color bar	
            sfmt = ticker.ScalarFormatter(useMathText=True) 
            sfmt = ticker.FormatStrFormatter('%.3f')  
            cbar = fig.colorbar(CS, ax=axes , format= sfmt ) 
            cbar.ax.set_ylabel('$C_{P}$', labelpad  = 20, rotation =  0, fontsize =16)  
            
            # plot fuselage 
            for i in range(n_fus):
                n_pts  = (n_sw + 1) * (n_cw + 1)
                j      = n_w + i
                x_pts  = np.reshape(np.atleast_2d(VD.X[j*(n_pts):(j+1)*(n_pts)]).T, (n_sw+1,n_cw+1))
                y_pts  = np.reshape(np.atleast_2d(VD.Y[j*(n_pts):(j+1)*(n_pts)]).T, (n_sw+1,n_cw+1))
                z_pts  = np.reshape(np.atleast_2d(VD.Z[j*(n_pts):(j+1)*(n_pts)]).T, (n_sw+1,n_cw+1))   
                CS_fus = axes.contourf( y_pts,x_pts, z_pts,cmap=plt.cm.bone) 
                
            plt.axis('off')	
            plt.grid(None)   
            
            if flight_profile: 
                time_vec      = np.empty(shape=[0,1])	
                cl_vec        = np.empty(shape=[0,1])	
                cd_vec        = np.empty(shape=[0,1])	
                l_d_vec       = np.empty(shape=[0,1])	
                altitude_vec  = np.empty(shape=[0,1])	
                mass_vec      = np.empty(shape=[0,1])          	
                for seg_i in range(seg_idx):	
                    if seg_i == seg_idx-1:	
                        t_vals   = results.segments[seg_i].conditions.frames.inertial.time[0:ti+1] / Units.min	
                        cl_vals  = results.segments[seg_i].conditions.aerodynamics.lift_coefficient[0:ti+1]	
                        cd_vals  = results.segments[seg_i].conditions.aerodynamics.drag_coefficient[0:ti+1]	
                        l_d_vals = cl_vals/cd_vals	
                        alt_vals = results.segments[seg_i].conditions.freestream.altitude[0:ti+1] / Units.ft	
                        m_vals   = results.segments[seg_i].conditions.weights.total_mass[0:ti+1] * 0.001              	
                
                    else:                    	
                        t_vals   = results.segments[seg_i].conditions.frames.inertial.time / Units.min	
                        cl_vals  = results.segments[seg_i].conditions.aerodynamics.lift_coefficient	
                        cd_vals  = results.segments[seg_i].conditions.aerodynamics.drag_coefficient	
                        l_d_vals = cl_vals/cd_vals 	
                        alt_vals = results.segments[seg_i].conditions.freestream.altitude / Units.ft	
                        m_vals   = results.segments[seg_i].conditions.weights.total_mass * 0.001  	
                
                    time_vec      = np.append(time_vec     ,t_vals[:,0])	
                    cl_vec        = np.append(cl_vec       ,cl_vals[:,0])	
                    cd_vec        = np.append(cd_vec       ,cd_vals[:,0])	
                    l_d_vec       = np.append(l_d_vec      , l_d_vals[:,0])	
                    altitude_vec  = np.append(altitude_vec ,alt_vals[:,0])	
                    mass_vec      = np.append(mass_vec     ,m_vals[:,0]) 	
                
                mini_axes1 = fig.add_subplot(gs[0:1, -1]) 
                mini_axes1.plot(time_vec, altitude_vec , 'ko-')	
                mini_axes1.set_ylabel('Altitude (ft)',axis_font)	
                mini_axes1.set_xlim(-10,420)	
                mini_axes1.set_ylim(0,36000)        	
                mini_axes1.grid(False)	
                
                mini_axes2 = fig.add_subplot(gs[1:2, -1])
                mini_axes2.plot(time_vec, mass_vec , 'ro-' )	
                mini_axes2.set_ylabel('Weight (tons)',axis_font)       	
                mini_axes2.grid(False)            	
                mini_axes2.set_xlim(-10,420)	
                mini_axes2.set_ylim(60,80)   	
                
                mini_axes3 = fig.add_subplot(gs[2:3, -1])
                mini_axes3.plot( time_vec, cl_vec, 'bo-'  )	
                mini_axes3.set_ylabel('$C_{L}$',axis_font)	
                mini_axes3.set_xlim(-10,420)	
                mini_axes3.set_ylim(0.3,0.9)  	
                mini_axes3.grid(False) 	
                
                mini_axes4 = fig.add_subplot(gs[3:4, -1])
                mini_axes4.plot(time_vec , l_d_vec ,'go-'  )	
                mini_axes4.set_ylabel('L/D',axis_font)	
                mini_axes4.set_xlabel('Time (mins)',axis_font)	
                mini_axes4.set_xlim(-10,420)	
                mini_axes4.set_ylim(15,20)  	
                mini_axes4.grid(False)             	
            
            if save_figure:
                plt.savefig(save_filename + '_' + str(img_idx) + file_type) 	
            img_idx += 1	
        seg_idx +=1 

# ------------------------------------------------------------------
#   Set Axis Parameters 
# ------------------------------------------------------------------
## @ingroup Plots
def set_axes(axes):
    """This sets the axis parameters for all plots

    Assumptions:
    None

    Source:
    None

    Inputs
    axes
        
    Outputs: 
    axes

    Properties Used:
    N/A	
    """   
    
    axes.minorticks_on()
    axes.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
    axes.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')      
    axes.grid(True)   

    return  