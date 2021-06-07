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
# Demonstration of a Chrono::Vehicle simulation in a non-ISO frame.
# The world frame has Y up, X forward, and Z pointing to the right.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math as m


#// =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    veh.ChWorldFrame.SetYUP()

    #  Create the HMMWV vehicle, set parameters, and initialize
    my_hmmwv = veh.HMMWV_Reduced()
    my_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    my_hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    my_hmmwv.SetChassisFixed(False) 
    my_hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, chrono.Q_from_AngY(initYaw)))
    my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
    my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    my_hmmwv.SetTireType(tire_model)
    my_hmmwv.SetTireStepSize(tire_step_size)
    my_hmmwv.Initialize()

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type)
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type)
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type)
    my_hmmwv.SetTireVisualizationType(tire_vis_type)

    # Create the terrain

    terrain = veh.RigidTerrain(my_hmmwv.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), 
                             200.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    app = veh.ChWheeledVehicleIrrApp(my_hmmwv.GetVehicle(), 'HMMWV-9 YUP world frame', irr.dimension2du(1000,800))
    app.SetSkyBox()
    app.AddTypicalLights()
    app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    app.SetTimestep(step_size)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Create the interactive driver system
    driver = veh.ChIrrGuiDriver(app)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Simulation loop

    realtime_timer = chrono.ChRealtimeStepTimer()
    while (app.GetDevice().run()):
        time = my_hmmwv.GetSystem().GetChTime()

        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.RenderFrame(chrono.ChVectorD(0, 0, 0), 10)
        app.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        my_hmmwv.Synchronize(time, driver_inputs, terrain)
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        my_hmmwv.Advance(step_size)
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
initLoc = chrono.ChVectorD(0, 1, 10)
initYaw = 0

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 

# Type of tire model (RIGID, RIGID_MESH, PACEJKA, LUGRE, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Simulation step sizes
step_size = 3e-3;
tire_step_size = 1e-3;

main()
