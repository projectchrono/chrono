# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Simone Benatti, Radu Serban
# =============================================================================
#
# Main driver function for the HMMWV full model.
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import errno
import os
import math as m


#// =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create systems

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(chassis_collision_type)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chassis_vis_type)
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    hmmwv.SetSteeringVisualizationType(steering_vis_type)
    hmmwv.SetWheelVisualizationType(wheel_vis_type)
    hmmwv.SetTireVisualizationType(tire_vis_type)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain

    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Initialize output

    try:
        os.mkdir(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
           print("Error creating output directory " )

    # Set up vehicle output
    hmmwv.GetVehicle().SetChassisOutput(True);
    hmmwv.GetVehicle().SetSuspensionOutput(0, True);
    hmmwv.GetVehicle().SetSteeringOutput(0, True);
    hmmwv.GetVehicle().SetOutput(veh.ChVehicleOutput.ASCII , out_dir, "output", 0.1);

    # Generate JSON information with available output channels
    hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()


    # Simulation loop


    # Number of simulation steps between miscellaneous events
    render_steps = m.ceil(render_step_size / step_size)
    debug_steps = m.ceil(debug_step_size / step_size)

    # Initialize simulation frame counter and simulation time
    step_number = 0
    render_frame = 0

    if (contact_vis):
        vis.SetSymbolscale(1e-4);
        # vis.EnableContactDrawing(irr.IrrContactsDrawMode_CONTACT_FORCES);

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        #End simulation
        if (time >= t_end):
            break

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        #Debug logging
        if (debug_output and step_number % debug_steps == 0) :
            print("\n\n============ System Information ============\n")
            print( "Time = ", time, "\n")
            #hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS)

            marker_driver = hmmwv.GetChassis().GetMarkers()[0].GetAbsCoordsys().pos
            marker_com = hmmwv.GetChassis().GetMarkers()[1].GetAbsCoordsys().pos
            print( "\nMarkers\n")
            print( "  Driver loc:      " , marker_driver.x , " " , marker_driver.y , " " , marker_driver.z)
            print( "  Chassis COM loc: " , marker_com.x, " ", marker_com.y, " ",marker_com.z)

            print("\nTire forces\n")
            tf_FL = hmmwv.GetVehicle().GetTire(0, veh.LEFT).ReportTireForce(terrain)
            tf_FR = hmmwv.GetVehicle().GetTire(0, veh.RIGHT).ReportTireForce(terrain)
            tf_RL = hmmwv.GetVehicle().GetTire(1, veh.LEFT).ReportTireForce(terrain)
            tf_RR = hmmwv.GetVehicle().GetTire(1, veh.RIGHT).ReportTireForce(terrain)
            print("   Front left:  ", tf_FL.force.x, " ", tf_FL.force.y, " ", tf_FL.force.z)
            print("   Front right: ", tf_FR.force.x, " ", tf_FR.force.y, " ", tf_FR.force.z)
            print("   Rear left:   ", tf_RL.force.x, " ", tf_RL.force.y, " ", tf_RL.force.z)
            print("   Rear right:  ", tf_RR.force.x, " ", tf_RR.force.y, " ", tf_RR.force.z)

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        # Increment frame number
        step_number += 1

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of powertrain models (SHAFTS, SIMPLE)
engine_model = veh.EngineModelType_SHAFTS
transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS

# Drive type (FWD, RWD, or AWD)
drive_type = veh.DrivelineTypeWV_AWD

# Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0;      # terrain height (FLAT terrain only)
terrainLength = 100.0;  # size in X direction
terrainWidth = 100.0;   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChContactMethod_SMC
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

# Debug logging
debug_output = True
debug_step_size = 1.0 / 1  # FPS = 1

# POV-Ray output
povray_output = False

main()
