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
# Chrono demonstration of sensors attached to a HMMWV
# A camera, gps and an imu are attached to the HMMWV
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math as m
import errno
import os
import math


# // =============================================================================

def main():
    print("Copyright (c) 2017 projectchrono.org" + "\n\n")

    # Create systems

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(chassis_collision_type)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
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
        patch_mat = chrono.ChMaterialSurfaceNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChMaterialSurfaceSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM,
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
            print("Error creating output directory ")

    # Set up vehicle output
    hmmwv.GetVehicle().SetChassisOutput(True)
    hmmwv.GetVehicle().SetSuspensionOutput(0, True)
    hmmwv.GetVehicle().SetSteeringOutput(0, True)
    hmmwv.GetVehicle().SetOutput(veh.ChVehicleOutput.ASCII, out_dir, "output", 0.1)

    # Generate JSON information with available output channels
    hmmwv.GetVehicle().ExportComponentList(out_dir + "/component_list.json")

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
        vis.SetSymbolscale(1e-4)
        # vis.EnableContactDrawing(irr.IrrContactsDrawMode_CONTACT_FORCES);

    # ---------------------------------------------
    # Create a sensor manager and add a point light
    # ---------------------------------------------
    manager = sens.ChSensorManager(hmmwv.GetSystem())
    manager.scene.AddPointLight(chrono.ChVectorF(
        0, 0, 100), chrono.ChColor(2, 2, 2), 5000)

    # ------------------------------------------------
    # Create a camera and add it to the sensor manager
    # ------------------------------------------------
    fov = 1.408
    lag = 0
    update_rate = 5
    exposure_time = 1/update_rate
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-5, 0, 2))
    cam = sens.ChCameraSensor(
        hmmwv.GetChassisBody(),              # body camera is attached to
        update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                    # camera's horizontal field of view
    )
    cam.SetName("Camera Sensor")
    # cam.SetLag(0);
    # cam.SetCollectionWindow(0);

    # Visualizes the image
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            image_width, image_height, "HMMWV Camera"))

    # Save the current image to a png file at the specified path
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "cam/"))

    # Add a camera to a sensor manager
    manager.AddSensor(cam)

    # ----------------------------------------------
    # Create an IMU sensor and add it to the manager
    # ----------------------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-8, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    imu = sens.ChAccelerometerSensor(hmmwv.GetChassisBody(),                     # body imu is attached to
                                     imu_update_rate,         # update rate in Hz
                                     offset_pose,             # offset pose
                                     imu_noise_none          # noise model
                                     )
    imu.SetName("IMU Sensor")
    imu.SetLag(imu_lag)
    imu.SetCollectionWindow(imu_collection_time)

    # Provides the host access to the imu data
    imu.PushFilter(sens.ChFilterAccelAccess())

    # Add the imu to the sensor manager
    manager.AddSensor(imu)

    # ----------------------------------------------
    # Create an GPS sensor and add it to the manager
    # ----------------------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-8, 0, 1), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    gps = sens.ChGPSSensor(hmmwv.GetChassisBody(),                     # body imu is attached to
                           gps_update_rate,       # update rate in Hz
                           offset_pose,             # offset pose
                           gps_reference,
                           gps_noise_none          # noise model
                           )
    gps.SetName("GPS Sensor")
    gps.SetLag(gps_lag)
    gps.SetCollectionWindow(gps_collection_time)

    # Provides the host access to the gps data
    gps.PushFilter(sens.ChFilterGPSAccess())

    # Add the gps to the sensor manager
    manager.AddSensor(gps)

    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        # End simulation
        if (time >= t_end):
            break

        if (step_number % render_steps == 0):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Debug logging
        if (debug_output and step_number % debug_steps == 0):
            print("\n\n============ System Information ============\n")
            print("Time = " << time << "\n\n")
            # hmmwv.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS)

            marker_driver = hmmwv.GetChassis().GetMarkers()[
                0].GetAbsCoord().pos
            marker_com = hmmwv.GetChassis().GetMarkers()[
                1].GetAbsCoord().pos
            print("Markers\n")
            print("  Driver loc:      ", marker_driver.x,
                  " ", marker_driver.y, " ", marker_driver.z)
            print("  Chassis COM loc: ", marker_com.x,
                  " ", marker_com.y, " ", marker_com.z)

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

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Increment frame number
        step_number += 1

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

    return 0


# -----------------
# Sensor parameters
# -----------------
# Update rate of each sensor in Hz
cam_update_rate = 5
imu_update_rate = 200
gps_update_rate = 2

# Image width and height
image_width = 1280
image_height = 720

# Lag time for each sensor
cam_lag = 0
imu_lag = 0
gps_lag = 0

# Collection window for each sensor
# Typically 1 / update rate
cam_collection_time = 0  # instant
imu_collection_time = 0  # instant
gps_collection_time = 1. / float(gps_update_rate)

# GPS reference point
# Located in Madison, WI
gps_reference = chrono.ChVectorD(-89.400, 43.070, 260.0)

# IMU and GPS noise models
# Setting to none (does not affect the data)
imu_noise_none = sens.ChNoiseNone()
gps_noise_none = sens.ChNoiseNone()

# ------------------
# Vehicle parameters
# ------------------

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 1.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of Engine (SHAFTS, SIMPLE)
engine_model = veh.EngineModelType_SHAFTS
# Type of transmission (SHAFTS, SIMPLE)
transmission_model = veh.TransmissionModelType_SHAFTS

# Drive type (FWD, RWD, or AWD)
drive_type = veh.DrivelineTypeWV_AWD

# Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height (FLAT terrain only)
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
t_end = 1000

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directory
out_dir = "SENSOR_OUTPUT/"

# Debug logging
debug_output = False
debug_step_size = 1.0 / 1  # FPS = 1
vis = True
save = False
# POV-Ray output
povray_output = False

main()
