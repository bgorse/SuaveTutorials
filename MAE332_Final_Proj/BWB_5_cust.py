######################################################################################
#                                                                                    #
#             SUAVE Analysis Code for MAE 332 Final Project Spring 2023              #
#      Inspired from SUAVE's 'tut_mission_B737.py' and 'tut_C172.py' Tutorials       # 
#                                                                                    #
######################################################################################

import numpy as np
import matplotlib.pyplot as plt
import math 

import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units, Data

from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties
from SUAVE.Plots.Performance import *

from SUAVE.Methods.Propulsion.turbofan_sizing import turbofan_sizing

from SUAVE.Input_Output.OpenVSP import write
from SUAVE.Input_Output.OpenVSP.vsp_read import vsp_read

from SUAVE.Plots.Performance.Mission_Plots import *

from copy import deepcopy

ft_2_m = 0.3048

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    """This function gets the vehicle configuration, analysis settings, and then runs the mission.
    Once the mission is complete, the results are plotted."""

    ### ******Stuff from full_setup() START *********

    # Collect baseline vehicle data and changes when using different configuration settings
    vehicle  = vehicle_setup(write_output=False)
    configs  = configs_setup(vehicle)

    # Get the analyses to be used when different configurations are evaluated
    configs_analyses = analyses_setup(configs)

    # Create the mission that will be flown
    mission  = cruise_mission_setup(configs_analyses)              # Can change between cruise and climb missions
    # mission  = climb_mission_setup(configs_analyses)
    missions_analyses = missions_setup(mission)

    # Add the analyses to the proper containers
    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    ### ******Stuff from full_setup() END *********

    # Perform operations needed to make the configurations and analyses usable in the mission
    configs.finalize()
    analyses.finalize()

    # Determine the vehicle weight breakdown (independent of mission fuel usage)
    # These may not be necessary
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()

    # Perform a mission analysis
    mission = analyses.missions.base
    results = mission.evaluate(mission)

    # Plot all mission results, including items such as altitude profile and L/D
    plot_mission(results)

    return

def vehicle_setup(write_output: bool=False):

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'PLNE'    
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff                 = 11200000.0 * Units.kilogram                  # NOT SET
    vehicle.mass_properties.takeoff                     = 11200000.0 * Units.kilogram                  # Same as TO: NOT SET
    # Assumes a jet fuel density of 800 kg/m^3 with our fuel vol of 52,000ft^3 ~= 1472.476
    vehicle.mass_properties.max_zero_fuel               = vehicle.mass_properties.max_takeoff - (1177981.0 * Units.kilogram) 
    vehicle.mass_properties.operating_empty             = vehicle.mass_properties.max_zero_fuel        # NOT SET: Max op empty should be same as ~max no fuel
    vehicle.mass_properties.cargo                       = 1524000.0  * Units.kilogram                  # Set from slides 
    vehicle.mass_properties.center_of_gravity           = np.array([[33.528, 0.0, 0.0] * Units.meter])   # FROM SLIDES
    vehicle.mass_properties.zero_fuel_center_of_gravity = np.array([[0.0,0.0,0.0]])                    # Not set
    
    # Envelope properties
    vehicle.envelope.ultimate_load = 3.0
    vehicle.envelope.limit_load    = 2.5

    # basic parameters
    # Main wing reference area
    vehicle.reference_area         = 3994.83 * Units['meters**2']  # ACCURATE
    vehicle.passengers             = 1
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "long range"

    # Costs
    vehicle.costs.industrial       = 0.0 # NOT SET: @ SULLY
    vehicle.costs.operating        = 0.0

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------ 

    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    # # Adds the airfoil to the main wing
    # mw_airfoil = SUAVE.Components.Airfoils.Airfoil()
    # mw_airfoil.naca_4_series_airfoil = '4412'
    # wing.append_airfoil(mw_airfoil)

    # Defining the main components of the wing
           
    wing.sweeps.quarter_chord    = 26.259 * Units.deg                  # Taken as average quarter chord throughout variable sweep
    wing.thickness_to_chord      = 0.12                                # Accurate
    wing.spans.projected         = 151.16 * Units.meter                # Accurate
    wing.chords.root             = 64.425 * Units.meter                # Accurate 
    wing.chords.tip              = 8.522 * Units.meter                 # Accurate
    wing.chords.mean_aerodynamic = 45.596 * Units.meter                # Accurate
    wing.areas.reference         = 3994.83 * Units['meters**2']        # ACCURATE
    wing.twists.root             = 0.0 * Units.degrees                 # Accurate
    wing.twists.tip              = 0.0 * Units.degrees                 # Accurate
    wing.origin                  = [[0.0, 0.0, 0.0]] * Units.meter     # ACCURATE
    wing.vertical                = False                               
    wing.symmetric               = True                                
    wing.dynamic_pressure_ratio  = 1.0                                 # Default
    wing.aspect_ratio            = wing.spans.projected**2. / wing.areas.reference 
    wing.taper                   = wing.chords.tip/wing.chords.root
    wing.dihedral                = 0.0                                 # Accurate
    wing.aerodynamic_center      = [31.7,0.0,0.0] * Units.meter        # Accurate in x, the rest assumed to be symmetric [x,y,z]
    wing.areas.wetted            = 8491.4 * Units['meters**2']         # Accurate  
    

    # ------------------------------------------------------------------
    #   Main Wing Segments
    # ------------------------------------------------------------------

    # # First segment
    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'Root Segment'
    # segment.percent_span_location         = 0.0                            # Accurate
    # segment.twist                         = 0. * Units.deg                 # Accurate
    # segment.root_chord_percent            = 1.                             # Accurate
    # segment.thickness_to_chord            = 0.12                           # Accurate
    # segment.dihedral_outboard             = 0.0 * Units.degrees            # Accurate
    # segment.sweeps.quarter_chord          = 37.6 * Units.degrees                           # Accurate
    # segment.areas.reference               = 360.62 * Units['meters**2']    # Accurate
    # segment.chords.mean_aerodynamic       = 61.62 * Units.meter            # Accurate
    # wing.append_segment(segment)

    # # Second segment
    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = '2nd Segment'
    # segment.percent_span_location         = 0.0774                         # Accurate
    # segment.twist                         = 0. * Units.deg                 # Accurate
    # segment.root_chord_percent            = 0.913                          # Accurate
    # segment.thickness_to_chord            = 0.12                           # Accurate
    # segment.dihedral_outboard             = 0.0 * Units.degrees            # Accurate
    # segment.sweeps.quarter_chord          = 34.7262 * Units.degrees        # Accurate
    # segment.areas.reference               = 193.96 * Units['meters**2']    # Accurate
    # segment.chords.mean_aerodynamic       = 56.722 * Units.meter           # Accurate 
    # wing.append_segment(segment)

    # # Third segment
    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = '3rd Segment'
    # segment.percent_span_location         = 0.1226                         # Accurate
    # segment.twist                         = 0. * Units.deg                 # Accurate
    # segment.root_chord_percent            = 0.847826                       # Accurate
    # segment.thickness_to_chord            = 0.12                           # Accurate
    # segment.dihedral_outboard             = 0.0 * Units.degrees            # Accurate
    # segment.sweeps.quarter_chord          = 29.7695 * Units.degrees        # Accurate
    # segment.areas.reference               = 151.35 * Units['meters**2']    # Accurate
    # segment.chords.mean_aerodynamic       = 52.137 * Units.meter           # Accurate
    # wing.append_segment(segment)

    # # Fourth segment
    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = '4th Segment'
    # segment.percent_span_location         = 0.1610                         # Accurate
    # segment.twist                         = 0. * Units.deg                 # Accurate
    # segment.root_chord_percent            = 0.77072                        # Accurate 
    # segment.thickness_to_chord            = 0.12                           # Accurate
    # segment.dihedral_outboard             = 0.0 * Units.degrees            # Accurate
    # segment.sweeps.quarter_chord          = 5.4388 * Units.degrees         # Accurate 
    # segment.areas.reference               = 229.11 * Units['meters**2']    # Accurate
    # segment.chords.mean_aerodynamic       = 39.030 * Units.meter           # Accurate
    # wing.append_segment(segment)

    # # Tip segment
    # segment                               = SUAVE.Components.Wings.Segment()
    # segment.tag                           = 'Tip Segment'
    # segment.percent_span_location         = 0.2387                         # Accurate 
    # segment.twist                         = 0. * Units.deg                 # Accurate
    # segment.root_chord_percent            = 0.440937                       # Accurate
    # segment.thickness_to_chord            = 0.12                           # Accurate
    # segment.dihedral_outboard             = 0.0 * Units.degrees            # Accurate
    # segment.sweeps.quarter_chord          = 26.1485 * Units.degrees        # Accurate
    # segment.areas.reference               = 1062.37 * Units['meters**2']   # Accurate
    # segment.chords.mean_aerodynamic       = 18.4647 * Units.meter          # Accurate
    # wing.append_segment(segment)
    

    # ------------------------------------------------------------------
    #   Main Wing Control Surfaces
    # ------------------------------------------------------------------

    # ## HIGH LIFT CONTROL SURFACES
    wing.high_lift               = False # change to true if adding the flaps/slats
    # flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    # flap.tag                   = 'flap' 
    # flap.span_fraction_start   = 0.20 
    # flap.span_fraction_end     = 0.70   
    # flap.deflection            = 0.0 * Units.degrees
    # # Flap configuration types are used in computing maximum CL and noise
    # flap.configuration_type    = 'double_slotted'
    # flap.chord_fraction        = 0.30   
    # wing.append_control_surface(flap)   
        
    # slat                       = SUAVE.Components.Wings.Control_Surfaces.Slat() 
    # slat.tag                   = 'slat' 
    # slat.span_fraction_start   = 0.324 
    # slat.span_fraction_end     = 0.963     
    # slat.deflection            = 0.0 * Units.degrees
    # slat.chord_fraction        = 0.1  	 
    # wing.append_control_surface(slat) 

    ## CAUSES ERROR WHEN RUNNING VORTEX LATTICE METHOD
    aileron1                       = SUAVE.Components.Wings.Control_Surfaces.Aileron() 
    aileron1.tag                   = 'aileron 1' 
    aileron1.span_fraction_start   = 0.0 
    aileron1.span_fraction_end     = 0.27 
    aileron1.deflection            = 0.0 * Units.degrees
    aileron1.chord_fraction        = 0.15    
    wing.append_control_surface(aileron1)

    # ## CAUSES IT TO BREAK ON WRITE FOR UNKNOWN REASON
    # aileron2                       = SUAVE.Components.Wings.Control_Surfaces.Aileron() 
    # aileron2.tag                   = 'aileron 2'  
    # aileron2.span_fraction_start   = 0.67 
    # aileron2.span_fraction_end     = 0.74 
    # aileron2.deflection            = 0.0 * Units.degrees
    # aileron2.chord_fraction        = 0.25    
    # wing.append_control_surface(aileron2)

    # ## CAUSES IT TO BREAK ON WRITE FOR UNKNOWN REASON
    # aileron3                       = SUAVE.Components.Wings.Control_Surfaces.Aileron() 
    # aileron3.tag                   = 'aileron 3' 
    # aileron3.span_fraction_start   = 0.7425 
    # aileron3.span_fraction_end     = 0.79 
    # aileron3.deflection            = 0.0 * Units.degrees
    # aileron3.chord_fraction        = 0.25    
    # wing.append_control_surface(aileron3)

    # ## CAUSES IT TO BREAK ON WRITE FOR UNKNOWN REASON
    # aileron4                       = SUAVE.Components.Wings.Control_Surfaces.Aileron() 
    # aileron4.tag                   = 'aileron 4' 
    # aileron4.span_fraction_start   = 0.7925 
    # aileron4.span_fraction_end     = 0.825 
    # aileron4.deflection            = 0.0 * Units.degrees
    # aileron4.chord_fraction        = 0.25    
    # wing.append_control_surface(aileron4)    

    # flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    # flap.tag                   = 'flap' 
    # flap.span_fraction_start   = 0.20 
    # flap.span_fraction_end     = 0.70   
    # flap.deflection            = 0.0 * Units.degrees
    # # Flap configuration types are used in computing maximum CL and noise
    # flap.configuration_type    = 'double_slotted'
    # flap.chord_fraction        = 0.30   
    # wing.append_control_surface(flap)   
        
    # slat                       = SUAVE.Components.Wings.Control_Surfaces.Slat() 
    # slat.tag                   = 'slat' 
    # slat.span_fraction_start   = 0.324 
    # slat.span_fraction_end     = 0.963     
    # slat.deflection            = 0.0 * Units.degrees
    # slat.chord_fraction        = 0.1  	 
    # wing.append_control_surface(slat) 

    # DO NOT Fill out more segment properties automatically
    # wing = segment_properties(wing)        
    # wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    # Add main wing to the vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal / Vertical Stabilizers
    # ------------------------------------------------------------------     
                           
    wing                                        = SUAVE.Components.Wings.Wing()
    wing.tag                                    = 'stabilizer' 
    wing.sweeps.quarter_chord                   = 34.8473 * Units.deg                 # Accurate
    wing.thickness_to_chord                     = 0.1                                 # Accurate
    wing.spans.projected                        = 17.1851*2.0                         # btot*cos(angle from vertical)
    wing.spans.total                            = 18.288*2.0                          # Accurate
    wing.areas.reference                        = 445.9346 * Units['meters**2']       # Accurate
    wing.chords.root                            = 17.417 * Units.meters               # Accurate
    wing.chords.tip                             = 6.967 * Units.meters                # Accurate
    wing.chords.mean_aerodynamic                = 12.192 * Units.meter                # Accurate
    wing.taper                                  = wing.chords.tip/wing.chords.root    # Accurate
    wing.aspect_ratio                           = (wing.spans.projected**2)/ wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees                 # Accurate 
    wing.twists.tip                             = 0.0 * Units.degrees                 # Accurate
    wing.origin                                 = [[51.816,9.144,0.0]* Units.meters ]    # Accurate
    wing.vertical                               = True
    wing.symmetric                              = True
    # wing.high_lift                              = False 
    wing.dynamic_pressure_ratio                 = 0.9                                 # Default assumption
    wing.areas.wetted                           = 824.06 * Units['meters**2']         # Accurate
    wing.dihedral                               = 20.0 * Units.deg                    # Accurate
    # wing.aerodynamic_center                   = [31.7,0.0,0.0] * Units.meter        # NOT SET [x,y,z]

    # Adds the airfoil to the stabilizer
    # Was being glitchy and NACA 0010 is the default anyway
    # stab_airfoil = SUAVE.Components.Airfoils.Airfoil()
    # stab_airfoil.naca_4_series_airfoil = '0010'
    # wing.append_airfoil(stab_airfoil)
    

    # DO NOT Fill out more segment properties automatically        
    # wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    # Adds both Stabilizers to the vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------ 

    engine_OD = 13.0 * ft_2_m
    engine_ID = 13.0 * 0.9 * ft_2_m
    engine_l =  24.0 * ft_2_m
    count_per_row = 5
    count_per_col = 2
    horizontal_spacing = 17.0
    vert_spacing = 14.0

    for k in range(count_per_col):
        for i in range(count_per_row):
            nacelle                       = SUAVE.Components.Nacelles.Nacelle()
            nacelle.tag                   = 'Turbofan_{}'.format(i + (count_per_row*k))
            nacelle.length                = engine_l
            nacelle.inlet_diameter        = engine_ID
            nacelle.diameter              = engine_OD                                  
            nacelle.areas.wetted          = 1.1*np.pi*nacelle.diameter*nacelle.length # Default
            if k % 2 == 1:
                nacelle.origin                = [[50.0* ft_2_m, (60.0 + i*horizontal_spacing  + 0.5*horizontal_spacing)* ft_2_m, (20.0 + k*vert_spacing)* ft_2_m]]
            else:
                nacelle.origin                = [[50.0* ft_2_m, (60.0 + i*horizontal_spacing)* ft_2_m, (20.0 + k*vert_spacing)* ft_2_m]]                     
            nacelle.flow_through          = True                                      
            nacelle_airfoil               = SUAVE.Components.Airfoils.Airfoil()       
            nacelle_airfoil.naca_4_series_airfoil = '2410'                            # Default
            nacelle.append_airfoil(nacelle_airfoil)

            nacelle_2                     = deepcopy(nacelle)
            nacelle_2.tag                 = nacelle.tag + "_mirror"
            nacelle_2.origin[0][1]       *= -1 
            
            vehicle.append_component(nacelle)  
            vehicle.append_component(nacelle_2)   


    # ------------------------------------------------------------------
    #   Turbofan Network
    # ------------------------------------------------------------------    
    
    turbofan = SUAVE.Components.Energy.Networks.Turbofan()
    # For some methods, the 'turbofan' tag is still necessary. This will be changed in the
    # future to allow arbitrary tags.
    turbofan.tag = 'turbofan'
    
    # High-level setup
    turbofan.number_of_engines = 2
    turbofan.bypass_ratio      = 5.4
    turbofan.origin            = [[13.72, 4.86,-1.9],[13.72, -4.86,-1.9]] * Units.meter

    # Establish the correct working fluid
    turbofan.working_fluid = SUAVE.Attributes.Gases.Air()
    
    # Components use estimated efficiencies. Estimates by technology level can be
    # found in textbooks such as those by J.D. Mattingly
    
    # ------------------------------------------------------------------
    #   Component 1 - Ram
    
    # Converts freestream static to stagnation quantities
    ram = SUAVE.Components.Energy.Converters.Ram()
    ram.tag = 'ram'
    
    # add to the network
    turbofan.append(ram)

    # ------------------------------------------------------------------
    #  Component 2 - Inlet Nozzle
    
    # Create component
    inlet_nozzle = SUAVE.Components.Energy.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet_nozzle'
    
    # Specify performance
    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 0.98
    
    # Add to network
    turbofan.append(inlet_nozzle)
    
    # ------------------------------------------------------------------
    #  Component 3 - Low Pressure Compressor
    
    # Create component
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'low_pressure_compressor'

    # Specify performance
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 1.14    
    
    # Add to network
    turbofan.append(compressor)
    
    # ------------------------------------------------------------------
    #  Component 4 - High Pressure Compressor
    
    # Create component
    compressor = SUAVE.Components.Energy.Converters.Compressor()    
    compressor.tag = 'high_pressure_compressor'
    
    # Specify performance
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 13.415    
    
    # Add to network
    turbofan.append(compressor)

    # ------------------------------------------------------------------
    #  Component 5 - Low Pressure Turbine
    
    # Create component
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='low_pressure_turbine'
    
    # Specify performance
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    
    # Add to network
    turbofan.append(turbine)
      
    # ------------------------------------------------------------------
    #  Component 6 - High Pressure Turbine
    
    # Create component
    turbine = SUAVE.Components.Energy.Converters.Turbine()   
    turbine.tag='high_pressure_turbine'

    # Specify performance
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    
    # Add to network
    turbofan.append(turbine)  
    
    # ------------------------------------------------------------------
    #  Component 7 - Combustor
    
    # Create component    
    combustor = SUAVE.Components.Energy.Converters.Combustor()   
    combustor.tag = 'combustor'
    
    # Specify performance
    combustor.efficiency                = 0.99 
    combustor.alphac                    = 1.0   
    combustor.turbine_inlet_temperature = 1450 # K
    combustor.pressure_ratio            = 0.95
    combustor.fuel_data                 = SUAVE.Attributes.Propellants.Jet_A()    
    
    # Add to network
    turbofan.append(combustor)

    # ------------------------------------------------------------------
    #  Component 8 - Core Nozzle
    
    # Create component
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    nozzle.tag = 'core_nozzle'
    
    # Specify performance
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    
    
    # Add to network
    turbofan.append(nozzle)

    # ------------------------------------------------------------------
    #  Component 9 - Fan Nozzle
    
    # Create component
    nozzle = SUAVE.Components.Energy.Converters.Expansion_Nozzle()   
    nozzle.tag = 'fan_nozzle'

    # Specify performance
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    
    
    # Add to network
    turbofan.append(nozzle)
    
    # ------------------------------------------------------------------
    #  Component 10 - Fan
    
    # Create component
    fan = SUAVE.Components.Energy.Converters.Fan()   
    fan.tag = 'fan'

    # Specify performance
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.7    
    
    # Add to network
    turbofan.append(fan)
    
    # ------------------------------------------------------------------
    #  Component 11 - thrust (to compute the thrust)
    
    thrust = SUAVE.Components.Energy.Processes.Thrust()       
    thrust.tag ='compute_thrust'
 
    # Design thrust is used to determine mass flow at full throttle
    thrust.total_design             = 2*24000. * Units.N #Newtons
    
    # Add to network
    turbofan.thrust = thrust
    
    # Design sizing conditions are also used to determine mass flow
    altitude      = 35000.0*Units.ft
    mach_number   = 0.78     

    # Determine turbofan behavior at the design condition
    turbofan_sizing(turbofan,mach_number,altitude)   
    
    # Add turbofan network to the vehicle 
    vehicle.append_component(turbofan)



    if write_output:
        write(vehicle, 'homebrewed_BWB5')

    return vehicle


# ----------------------------------------------------------------------
#   Cruise Mission definition    
# ----------------------------------------------------------------------

def cruise_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    #airport: At Sea Level
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    
    
    M_cruise = 0.6
    T_sl = 293.15 # K
    P_sl = 101325. # Pa
    rho_sl = 1.225 # kg/m^3
    gamma_air = 1.4 # 
    R_air = 287. # J/kg-K
    V_cruise = M_cruise * np.sqrt(gamma_air*R_air*T_sl)
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"
    segment.analyses.extend( analyses.base)
    segment.air_speed  = V_cruise * Units['m/s']
    segment.distance   = 6500. * Units.nautical_miles
    segment.altitude   = 30.0 * Units.meters


    # ## IS the state necessary????
    # ones_row                                        = segment.state.ones_row   
    # segment.state.numerics.number_control_points    = 16
    # segment.state.unknowns.throttle                 = 1.0 * ones_row(1)

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

# ----------------------------------------------------------------------
#   Cruise + Climb Mission Definition    
# ----------------------------------------------------------------------

def climb_mission_setup(analyses, max_alt, M_at_alt):
    """
    Parameter: max_alt = maximum altitude in km
    Assumes a linear linear regime in the troposphere since low flight
    Temp scales at -6.5K/km
    """

    M_cruise = 0.6
    T_sl = 293.15 # K
    P_sl = 101325. # Pa
    rho_sl = 1.225 # kg/m^3
    gamma_air = 1.4 # 
    R_air = 287. # J/kg-K

    # Each pressure value represents the pressure in Pa at 0-10km
    # Pressures from https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
    P_at_alts = [101325., 89880., 79500., 70120., 61660., 54050.,
                47220., 41110., 35650., 30800., 26500.]
    P_at_high_alts = [26500., 12110., 55290.]
    ceil = math.ceil(max_alt)
    floor = math.floor(max_alt)
    if ceil == floor and ceil > 0 and ceil < 11:
        P_at_alt = P_at_alts[max_alt]
    elif max_alt > 10:
        if max_alt < 15:
            P_at_alt = ((max_alt - 10)/(15-20))*(P_at_high_alts[2]-P_at_high_alts[1]) + P_at_high_alts[1]
        elif max_alt < 20:
            P_at_alt = ((max_alt - 15)/(20-15))*(P_at_high_alts[3]-P_at_high_alts[2]) + P_at_high_alts[2]
        else:
            raise RuntimeError("Altitude Outside alloted altitude range")
    else:
        P_at_alt = ((max_alt - floor)/(ceil-floor))*(P_at_alts[ceil]-P_at_alts[floor]) + P_at_alts[floor]

    T_at_alt = 293.15 - (6.5*max_alt) # K
    rho_at_alt = P_at_alt/(R_air*T_at_alt) # kg/m^3

    V_at_alt = M_at_alt*np.sqrt(gamma_air*T_at_alt*R_air)

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    V_cruise = M_cruise * np.sqrt(gamma_air*R_air*T_sl)
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"
    segment.analyses.extend( analyses.base)
    segment.air_speed  = V_cruise
    segment.distance   = 5500. * Units.nautical_miles
    segment.altitude   = 30.0 * Units.meters

    # ## IS the state necessary????
    # ones_row                                        = segment.state.ones_row   
    # segment.state.numerics.number_control_points    = 16
    # segment.state.unknowns.throttle                 = 1.0 * ones_row(1)

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"
    segment.analyses.extend( analyses.base)  
    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 7. * Units.deg    
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = max_alt   * Units.km
    segment.air_speed      = 0.9*V_cruise * Units['m/s']                 ### We're coming off of flying in cruise
    segment.climb_rate     = 5   * Units['m/s']                          ### ~ 2.5% climb gradient

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    V_cruise = M_cruise * np.sqrt(gamma_air*R_air*T_sl)
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise at alt"
    segment.analyses.extend( analyses.base)
    segment.air_speed  = V_at_alt
    segment.distance   = 250. * Units.nautical_miles
    segment.altitude   = max_alt * Units.km

    # ## IS the state necessary????
    # ones_row                                        = segment.state.ones_row   
    # segment.state.numerics.number_control_points    = 16
    # segment.state.unknowns.throttle                 = 1.0 * ones_row(1)

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"
    segment.analyses.extend( analyses.base)
    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 5. * Units.deg      
    segment.altitude_end = max_alt*0.8 * Units.km
    segment.air_speed    = 0.95*V_at_alt * Units['m/s']
    segment.descent_rate = 1.5   * Units['m/s']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"
    segment.analyses.extend( analyses.base)
    segment.altitude_end = 0.6*max_alt   * Units.km
    segment.air_speed    = 0.85*V_at_alt * Units['m/s']
    segment.descent_rate = 1.5  * Units['m/s']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"
    segment.analyses.extend( analyses.base)
    segment.altitude_end = 0.4*max_alt  * Units.km
    segment.air_speed    = 0.75*V_at_alt * Units['m/s']
    segment.descent_rate = 1.5   * Units['m/s']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4"
    segment.analyses.extend( analyses.base)
    segment.altitude_end = 0.2*max_alt   * Units.km
    segment.air_speed    = 0.65*V_at_alt * Units['m/s']
    segment.descent_rate = 1.5   * Units['m/s']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fifth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_5"
    segment.analyses.extend( analyses.base)
    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 5. * Units.deg       
    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 0.6*V_at_alt * Units['m/s']
    segment.descent_rate = 0.75   * Units['m/s']

    # append to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def missions_setup(base_mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    # Setup the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    # Only one mission (the base mission) is defined in this case
    missions.base = base_mission

    return missions

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):
    """This function plots the results of the mission analysis and saves those results to 
    png files."""

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)      
        
    return

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = SUAVE.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

# Probably needs future tweaking
def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    # config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    # config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    # A max lift coefficient factor of 1 is the default, but it is highlighted here as an option
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    # config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    # config.wings['main_wing'].control_surfaces.slat.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)    

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    # config.wings['main_wing'].control_surfaces.flap.deflection = 30. * Units.deg
    # config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg  
    config.max_lift_coefficient_factor    = 1. 

    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    # config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    # config.wings['main_wing'].control_surfaces.slat.deflection = 20. * Units.deg
    config.max_lift_coefficient_factor    = 1. 
  
    configs.append(config)

    return configs

def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses 

# This section is needed to actually run the various functions in the file
if __name__ == '__main__': 
    main()    
    # The show commands makes the plots actually appear
    plt.show()