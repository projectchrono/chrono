# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
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
# Main driver function for the FED-alpha full model.
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m


#// =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create systems

    #  Create the FEDA vehicle, set parameters, and initialize
    my_feda = veh.FEDA()
    my_feda.SetContactMethod(contact_method)
    my_feda.SetChassisCollisionType(chassis_collision_type)
    my_feda.SetChassisFixed(False) 
    my_feda.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    my_feda.SetPowertrainType(powertrain_model)
    my_feda.SetTireType(tire_model)
    my_feda.SetTireStepSize(tire_step_size)
    my_feda.Initialize()

    my_feda.SetChassisVisualizationType(chassis_vis_type)
    my_feda.SetSuspensionVisualizationType(suspension_vis_type)
    my_feda.SetSteeringVisualizationType(steering_vis_type)
    my_feda.SetWheelVisualizationType(wheel_vis_type)
    my_feda.SetTireVisualizationType(tire_vis_type)

    # Create the terrain

    terrain = veh.RigidTerrain(my_feda.GetSystem())
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
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    app = veh.ChWheeledVehicleIrrApp(my_feda.GetVehicle(), 'HMMWV', irr.dimension2du(1000,800))

    app.SetSkyBox()
    app.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
    app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    app.SetChaseCamera(trackPoint, 6.0, 0.5)
    app.SetTimestep(step_size)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Create the interactive driver system
    driver = veh.ChIrrGuiDriver(app)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(10 * step_size / steering_time)
    driver.SetThrottleDelta(10 * step_size / throttle_time)
    driver.SetBrakingDelta(10 * step_size / braking_time)

    driver.Initialize()

    # Simulation loop
    realtime_timer = chrono.ChRealtimeStepTimer()
    while (app.GetDevice().run()):
        time = my_feda.GetSystem().GetChTime()

        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        my_feda.Synchronize(time, driver_inputs, terrain)
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        my_feda.Advance(step_size)
        app.Advance(step_size)

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of powertrain model (SHAFTS, SIMPLE_MAP)
powertrain_model = veh.PowertrainModelType_SIMPLE_MAP

# Type of tire model (RIGID, PAC02)
tire_model = veh.TireModelType_PAC02

# Rigid terrain
terrainHeight = 0;      # terrain height (FLAT terrain only)
terrainLength = 100.0;  # size in X direction
terrainWidth = 100.0;   # size in Y direction

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Simulation step sizes
step_size = 1e-3;
tire_step_size = 1e-3;

main()
