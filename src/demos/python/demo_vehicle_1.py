"""
// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Main driver function for the HMMWV full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================
"""

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math as m


#// =============================================================================

#int main(int argc, char* argv[]) {
#    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

def main():


    # Create systems


    #  Create the HMMWV vehicle, set parameters, and initialize
    my_hmmwv = veh.HMMWV_Full()
    my_hmmwv.SetContactMethod(contact_method)
    my_hmmwv.SetChassisCollisionType(chassis_collision_type)
    my_hmmwv.SetChassisFixed(False) 
    my_hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    my_hmmwv.SetPowertrainType(powertrain_model)
    my_hmmwv.SetDriveType(drive_type)
    my_hmmwv.SetSteeringType(steering_type)
    my_hmmwv.SetTireType(tire_model)
    my_hmmwv.SetTireStepSize(tire_step_size)
    my_hmmwv.SetVehicleStepSize(step_size)
    my_hmmwv.Initialize()

    #VisualizationType tire_vis_type =
     #   (tire_model == TireModelType::RIGID_MESH) ? VisualizationType::MESH : VisualizationType::NONE;

    tire_vis_type = veh.VisualizationType_MESH # if tire_model == veh.TireModelType_RIGID_MESH else tire_vis_type = veh.VisualizationType_NONE

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type)
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type)
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type)
    my_hmmwv.SetTireVisualizationType(tire_vis_type)

    # Create the terrain

    terrain = veh.RigidTerrain(my_hmmwv.GetSystem())
    #patch = veh.Patch()
    """
    switch (terrain_model) {
        case RigidTerrain::BOX:
            patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                     ChVector<>(terrainLength, terrainWidth, 10));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::HEIGHT_MAP:
            patch = terrain.AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 128,
                                     128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::MESH:
            patch = terrain.AddPatch(CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    """
    patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight - 5), chrono.QUNIT), chrono.ChVectorD(terrainLength, terrainWidth, 10))

    patch.SetContactFrictionCoefficient(0.9)
    patch.SetContactRestitutionCoefficient(0.01)
    patch.SetContactMaterialProperties(2e7, 0.3)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    # please note that wchar_t conversion requres some workaround
    app = veh.ChWheeledVehicleIrrApp(my_hmmwv.GetVehicle(), my_hmmwv.GetPowertrain())#, "HMMWV Demo")

    app.SetSkyBox()
    app.AddTypicalLights(chronoirr.vector3df(30, -30, 100), chronoirr.vector3df(30, 50, 100), 250, 130)

    app.SetChaseCamera(trackPoint, 6.0, 0.5)
    app.SetTimestep(step_size)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Initialize output

    try:
           os.mkdir(out_dir)
    except:
           print("Error creating directory " )


    if (povray_output):
        try: 
            os.mkdir(pov_dir)
        except:
            print("Error creating POV directory ")
        terrain.ExportMeshPovray(out_dir)

    # Initialize output file for driver inputs
    #driver_file = out_dir + "/driver_inputs.txt"
    # no RECORD so far
    #utils::CSV_writer driver_csv(" ");

    # Set up vehicle output
    my_hmmwv.GetVehicle().SetChassisOutput(True);
    my_hmmwv.GetVehicle().SetSuspensionOutput(0, True);
    my_hmmwv.GetVehicle().SetSteeringOutput(0, True);
    my_hmmwv.GetVehicle().SetOutput(veh.ChVehicleOutput.ASCII , out_dir, "output", 0.1);

    # Generate JSON information with available output channels
    my_hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    # Create the driver system

    # Create the interactive driver system
    driver = veh.ChIrrGuiDriver(app)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    # If in playback mode, attach the data file to the driver system and
    # force it to playback the driver inputs.
    if (driver_mode == "PLAYBACK"):
        #driver.SetInputDataFile(driver_file)
        driver.SetInputMode(veh.ChIrrGuiDriver.DATAFILE)

    driver.Initialize()


    # Simulation loop

    """
    if (debug_output) :
        GetLog() << "\n\n============ System Configuration ============\n"
        my_hmmwv.LogHardpointLocations()"""

    # Number of simulation steps between miscellaneous events
    render_steps = m.ceil(render_step_size / step_size)
    debug_steps = m.ceil(debug_step_size / step_size)

    # Initialize simulation frame counter and simulation time
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    render_frame = 0
    time = 0

    if (contact_vis):
        app.SetSymbolscale(1e-4);
        #app.SetContactsDrawMode(chronoirr.eCh_ContactsDrawMode::CONTACT_FORCES);

    while (app.GetDevice().run()):
        time = my_hmmwv.GetSystem().GetChTime()

        #End simulation
        if (time >= t_end):
            break

        app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')

        # Output POV-Ray data
        if (povray_output and step_number % render_steps == 0) :
            #char filename[100];
            print('filename', "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
            #utils::WriteShapesPovray(my_hmmwv.GetSystem(), filename);
            #render_frame++;

        #Debug logging
        if (debug_output and step_number % debug_steps == 0) :
            print ("\n\n============ System Information ============\n")
            print( "Time = " << time << "\n\n")
            #my_hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS)

            marker_driver = my_hmmwv.GetChassis().GetMarkers()[0].GetAbsCoord().pos
            marker_com = my_hmmwv.GetChassis().GetMarkers()[1].GetAbsCoord().pos
            print ( "Markers\n")
            print ( "  Driver loc:      " , marker_driver.x , " " , marker_driver.y , " " , marker_driver.z)
            print( "  Chassis COM loc: " , marker_com.x, " ", marker_com.y, " ",marker_com.z)


        # Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle()
        steering_input = driver.GetSteering()
        braking_input = driver.GetBraking()

        # Driver output
        """
        if (driver_mode == RECORD) {
            driver_csv << time << steering_input << throttle_input << braking_input << std::endl;
        }"""

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        my_hmmwv.Synchronize(time, steering_input, braking_input, throttle_input, terrain)
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input)

        # Advance simulation for one timestep for all modules
        step = realtime_timer.SuggestSimulationStep(step_size)
        driver.Advance(step)
        terrain.Advance(step)
        my_hmmwv.Advance(step)
        app.Advance(step)

        # Increment frame number
        step_number += 1

        app.EndScene()
    """    
    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }"""
    return 0

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath('../../../../Library/data/')
veh.SetDataPath('../../../../Library/data/Vehicle/')
#  Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 1.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

#enum DriverMode { DEFAULT, RECORD, PLAYBACK };
driver_mode = 'DEFAULT'

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.ChassisCollisionType_NONE

# Type of powertrain model (SHAFTS, SIMPLE)
powertrain_model = veh.PowertrainModelType_SHAFTS

# Drive type (FWD, RWD, or AWD)
drive_type = veh.DrivelineType_AWD

# Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
steering_type = veh.SteeringType_PITMAN_ARM

# Type of tire model (RIGID, RIGID_MESH, PACEJKA, LUGRE, FIALA, PAC89)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0;      # terrain height (FLAT terrain only)
terrainLength = 100.0;  # size in X direction
terrainWidth = 100.0;   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChMaterialSurface.SMC
contact_vis = False;

# Simulation step sizes
step_size = 3e-3;
tire_step_size = 1e-3;

# Simulation end time
t_end = 1000;

# Time interval between two render frames
render_step_size = 1.0 / 50;  # FPS = 50

# Output directories
out_dir = os.path.join(os.path.dirname(__file__), "HMMWV_demo")
pov_dir = os.path.join(os.path.dirname(__file__),  "POVRAY")

# Debug logging
debug_output = False
debug_step_size = 1.0 / 1  # FPS = 1

# POV-Ray output
povray_output = False

main() 