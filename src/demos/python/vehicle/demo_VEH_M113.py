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
# Authors: Radu Serban
# =============================================================================
#
# Demonstration program for M113 vehicle on rigid terrain.
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math as m


# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the M113 vehicle
    # ------------------------

    vehicle = veh.M113_Vehicle(False, 
                               veh.TrackShoeType_SINGLE_PIN, 
                               veh.BrakeType_SIMPLE, 
                               chrono.ChContactMethod_SMC,
                               veh.CollisionType_NONE)

    vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))

    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSprocketVisualizationType(veh.VisualizationType_MESH);
    vehicle.SetIdlerVisualizationType(veh.VisualizationType_MESH);
    vehicle.SetRoadWheelAssemblyVisualizationType(veh.VisualizationType_MESH);
    vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_MESH);
    vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH);

    # Create the powertrain system
    # ----------------------------

    powertrain = veh.M113_SimpleCVTPowertrain("Powertrain")
    vehicle.InitializePowertrain(powertrain)

    # Create the terrain
    # ------------------

    terrain = veh.RigidTerrain(vehicle.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChMaterialSurfaceNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChMaterialSurfaceSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 
                             terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    # -------------------------------------

    app = veh.ChTrackedVehicleIrrApp(vehicle, 'M113', irr.dimension2du(1000,800))

    app.SetSkyBox()
    app.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
    app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    app.SetChaseCamera(trackPoint, 6.0, 0.5)
    app.SetTimestep(step_size)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Create the interactive driver system
    # ------------------------------------

    driver = veh.ChIrrGuiDriver(app)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 0.5  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # Simulation loop
    # ---------------

    # Inter-module communication data
    shoe_forces_left = veh.TerrainForces(vehicle.GetNumTrackShoes(veh.LEFT))
    shoe_forces_right = veh.TerrainForces(vehicle.GetNumTrackShoes(veh.RIGHT))

    # Number of simulation steps between miscellaneous events
    render_steps = m.ceil(render_step_size / step_size)

    # Initialize simulation frame counter and simulation time
    step_number = 0

    realtime_timer = chrono.ChRealtimeStepTimer()
    while (app.GetDevice().run()):
        time = vehicle.GetSystem().GetChTime()

        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right)
        app.Synchronize("", driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        app.Advance(step_size)

        # Increment frame number
        step_number += 1

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 1.1)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Rigid terrain
terrainHeight = 0;      # terrain height (FLAT terrain only)
terrainLength = 100.0;  # size in X direction
terrainWidth = 100.0;   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 0.0)

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size = 5e-4;

# Time interval between two render frames
render_step_size = 1.0 / 60;  # FPS = 60

main()
