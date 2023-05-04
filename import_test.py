### FIXME: Made changes in 



## Copied Imports from tut_C172.py
import numpy as np
import matplotlib.pyplot as plt
import math 

import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units, Data

from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties
from SUAVE.Plots.Performance import *

from SUAVE.Input_Output.OpenVSP import write
from SUAVE.Input_Output.OpenVSP.vsp_read import vsp_read
# Unit type can be 'SI' or 'Imperial'

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # wing areas
    for wing in base.wings:                                      # Set these manually FIXME
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # diff the new data
    base.store_diff()

    return 

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions

def plot_mission(results,line_style='bo-'):

    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)    
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)          
        
    return

def main():
    print("NOTE: there seems to be some randomness in SUAVE. Sometimes it runs with errors. Re-running usually works")
    ## Specifically, some errors for missing geometries like this: "KeyError: 'key "fuselagegeom_11182" already exists'"
    vehicle = read_vehicle()
    # Extracting some of the relevant components of a vehicle: see SUAVE > trunk > SUAVE > Vehicle.py for more
    wings = vehicle.wings
    fuses = vehicle.fuselages
    mass_prop = vehicle.mass_properties
    vehicle_envelope = vehicle.envelope
    if vehicle.reference_area == 0.0:
        print("Model Reference Area is " + str(vehicle.reference_area) + "; set it to the wetted area as calculated in VSP for accurate analysis")
    vehicle.reference_area = 0.0 ## << SET THE WETTED AREA HERE

    # Returns a list of the wings
    wings_list = []
    print("These are the wings that exist in your model. Each will need to be adjusted independently")
    for w in wings:
        print(type(w))
        wings_list.append(w)
    
    fuse_list = []
    print("These are the fuselages that exist in your model. Each will need to be adjusted independently")
    for f in fuses:
        # print(type(f))
        fuse_list.append(f)

    # In this case there is first the main wing and second the vertical tails. 
    main_wing = wings_list[0]
    vertical_tail = wings_list[1]
    print("mw span: " + str(main_wing.spans.total))
    main_wing.spans.total = 1000
    main_wing.tag = 'main_wing'
    print("main wing tag: " + str(main_wing.tag))
    print("vert tail tag: " + str(vertical_tail.tag))
    # main_wing.mass_properties.center_of_gravity = np.array([[100.0, 0.0, 15.0]]) # << successfully changes the mw's COM
    # print("******MAIN WING******\n" + str(main_wing))           # These lines produce a lot of output so they are
    # print("******VERTICAL TAIL******\n" + str(vertical_tail))   # commented for the moment
    
    engines_list = []
    engine_mounts = []
    for f in fuse_list:
        # Important parameters: f.lengths.total, 
        print(f.nose_curvature)
        if f.effective_diameter > 3: 
            engines_list.append(f)
        else:
            engine_mounts.append(f)

    cs = main_wing.control_surfaces
    for i, c in enumerate(cs):
        # Printing Relevant Parameters for all of the control surfaces
        # Control Surfaces Generally seem to be a bit wonky
        # print("tag: " + str(c.tag))
        # print("span: " + str(c.span))
        # print("span_fraction_start: " + str(c.span_fraction_start))
        # print("span_fraction_end: " + str(c.span_fraction_end))
        # print("chord_fraction: " + str(c.chord_fraction))

        # This only works in Ben Gorse's SUAVE (Modded vsp_wing.py)
        if i == 0: 
            c.span_fraction_start = 0.0
            c.span_fraction_end   = 0.27
    
    for k, e in enumerate(engines_list):
        # Creating values to have a flat front of a fuselage
        vals                    = Data()
        vals.nose               = Data()
        vals.nose.top           = Data()
        vals.nose.side          = Data()
        vals.nose.TB_Sym        = Data()
        vals.nose.z_pos         = Data()
        vals.tail               = Data()

        vals.nose.top.angle     = 0.0
        vals.nose.top.strength  = 0.0
        vals.nose.side.angle    = 0.0
        vals.nose.side.strength = 0.0
        vals.nose.TB_Sym        = True    # Top/Bottom Symmetry
        vals.nose.z_pos         = 0.0 

        e.nose_curvature = 0.01
        e.OpenVSP_values = vals
        if 'OpenVSP_values' in e: 
            vals = e.OpenVSP_values
            # Printing relevant aircraft parameters
            # print("engine num: " + str(k))
            # print("vals.nose.top.angle: " + str(vals.nose.top.angle))
            # print("vals.nose.top.strength: " + str(vals.nose.top.strength))
            # print("vals.nose.side.angle: " + str(vals.nose.side.angle))
            # print("vals.nose.side.strength: " + str(vals.nose.side.strength))
            # print("vals.nose.TB_Sym: " + str(vals.nose.TB_Sym))
            # print("vals.nose.z_pos: " + str(vals.nose.z_pos))

        
        for i, seg in enumerate(e.Segments):
            # seg.vsp_data.shape = 'circle'
            seg.width = 3.9624
            seg.height = 4
            # seg.effective_diameter = 50
            # Printing relevant engine parameters
            # print("engine num: " + str(k) + "   seg: " + str(i))
            # print("tag: " + str(seg.tag))
            # print("percent_x_location: " + str(seg.percent_x_location))
            # print("height: " + str(seg.height))
            # print("width: " + str(seg.width))
            # print("length: " + str(seg.length))
            # print("effective_diameter: " + str(seg.effective_diameter))
            # print("vsp_data.xsec_id: " + str(seg.vsp_data.xsec_id))
            # print("vsp_data.shape: " + str(seg.vsp_data.shape))

    for m in engine_mounts: 
        m.heights.at_quarter_length = 1.2192
        m.nose_curvature = 0

    for q in vehicle.fuselages:
        print("q: " + str(q.effective_diameter))

    # write(vehicle, 'mod_parsed_BWB5')

    # vehicle data pre-defined from above
    configs  = configs_setup(vehicle)

    # vehicle analyses
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis
    
    configs_analyses = analyses

    # mission analyses
    mission  = cruise_mission_setup(configs_analyses)             # FIXME Set to Cruise or Climb
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # plt the old results
    plot_mission(results)
    return

def read_vehicle():
    # Can set units to 'SI' or 'Imperial': If set to Imperial, exports in metric.
    vehicle = vsp_read('NathanBWB_5_simplified.vsp3',units_type = 'Imperial')

    # mass properties
    vehicle.mass_properties.takeoff           = 11000000. * Units.lb   # Set TO Mass
    vehicle.mass_properties.operating_empty   =  9000000. * Units.lb   # Set OP EW
    vehicle.mass_properties.max_takeoff       = 12000000. * Units.lb   # Max TOW
    vehicle.mass_properties.max_payload       =  2000000. * Units.lb   # Max Payload
    vehicle.mass_properties.center_of_gravity = [[25.0,   0.  ,  0. ]] # Set COM in m

    # basic parameters
    vehicle.reference_area         = 4010 * Units['meters**2']  
    vehicle.passengers             = 0
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "long range"

    # write(vehicle, 'parsed_BWB5')
    return vehicle

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

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

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

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"
    segment.analyses.extend( analyses.base)  
    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 7. * Units.deg    
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 0.3*max_alt   * Units.km
    segment.air_speed      = V_cruise * Units['m/s']                       ### We're coming off of flying in cruise
    segment.climb_rate     = 2.0   * Units['m/s']                          ### CHANGE ME

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"
    segment.analyses.extend( analyses.base)
    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 5. * Units.deg  
    segment.altitude_start = 0.3*max_alt   * Units.km
    segment.altitude_end   = 0.8*max_alt * Units.km
    segment.air_speed      = (V_cruise + (V_at_alt-V_cruise)*0.5) * Units['m/s']
    segment.climb_rate     = 2.0   * Units['m/s']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3"
    segment.analyses.extend( analyses.base)
    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 5. * Units.deg  
    segment.altitude_start = 0.8*max_alt * Units.km
    segment.altitude_end = 1*max_alt * Units.km
    segment.air_speed    = (V_cruise + (V_at_alt-V_cruise)*0.9)  * Units['m/s']
    segment.climb_rate   = 1.0    * Units['m/s']

    # add to mission
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

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.AVL()
    aerodynamics.process.compute.lift.inviscid.settings.filenames.avl_bin_name = 'C:\\Users\\bengo\\Documents\\Github\\SuaveTutorials\\BWB_5_Analysis' # CHANGE ME TO YOUR DIRECTORY
    #aerodynamics.settings.number_spanwise_vortices  = 5
    #aerodynamics.settings.number_chordwise_vortices = 3
    aerodynamics.geometry = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.AVL()
    stability.settings.filenames.avl_bin_name = 'C:\\Users\\bengo\\Documents\\Github\\SuaveTutorials\\BWB_5_Analysis'  #  CHANGE ME TO YOUR DIRECTORY
    
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
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

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    return configs


if __name__ == '__main__':
    main()
    plt.show()
