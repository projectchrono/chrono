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

    m113 = veh.M113()
    m113.SetContactMethod(chrono.ChContactMethod_SMC)
    m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
    m113.SetEngineType(veh.EngineModelType_SHAFTS)
    m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    m113.SetBrakeType(veh.BrakeType_SIMPLE)

    m113.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    m113.Initialize()

    m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    m113.SetSprocketVisualizationType(veh.VisualizationType_MESH);
    m113.SetIdlerVisualizationType(veh.VisualizationType_MESH);
    m113.SetIdlerWheelVisualizationType(veh.VisualizationType_MESH);
    m113.SetSuspensionVisualizationType(veh.VisualizationType_MESH);
    m113.SetRoadWheelVisualizationType(veh.VisualizationType_MESH);
    m113.SetTrackShoeVisualizationType(veh.VisualizationType_MESH);

    m113.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    # ------------------

    terrain = veh.RigidTerrain(m113.GetSystem())
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
    patch.SetColor(chrono.ChColor(0.5, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    # -------------------------------------

    vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('M113')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(m113.GetVehicle())

    # Create the interactive driver system
    # ------------------------------------

    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 0.5  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # Solver and integrator settings
    # ------------------------------

    m113.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

    # Simulation loop
    # ---------------

    # Number of simulation steps between miscellaneous events
    render_steps = m.ceil(render_step_size / step_size)

    # Initialize simulation frame counter and simulation time
    step_number = 0

    m113.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = m113.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        m113.Synchronize(time, driver_inputs)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        m113.Advance(step_size)
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
initLoc = chrono.ChVector3d(0, 0, 1.1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Rigid terrain
terrainHeight = 0;      # terrain height (FLAT terrain only)
terrainLength = 100.0;  # size in X direction
terrainWidth = 100.0;   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 0.0)

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size = 5e-4;

# Time interval between two render frames
render_step_size = 1.0 / 60;  # FPS = 60

main()
