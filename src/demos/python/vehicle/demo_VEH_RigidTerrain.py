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
# Demonstration of using a RigidTerrain constructed from different patches.
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    my_hmmwv = veh.HMMWV_Full()
    my_hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    my_hmmwv.SetChassisFixed(False);
    my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-10, -2, 0.6), chrono.ChQuaternionD(1, 0, 0, 0)))
    my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
    my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    my_hmmwv.SetTireType(veh.TireModelType_TMEASY)
    my_hmmwv.SetTireStepSize(tire_step_size)
    my_hmmwv.Initialize()

    my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)
    my_hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    my_hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the terrain with multiple patches
    terrain = veh.RigidTerrain(my_hmmwv.GetSystem())

    patch1_mat = chrono.ChMaterialSurfaceNSC()
    patch1_mat.SetFriction(0.9)
    patch1_mat.SetRestitution(0.01)
    patch1 = terrain.AddPatch(patch1_mat, chrono.ChVectorD(-16, 0, 0), chrono.ChVectorD(0, 0, 1), 32, 20)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

    patch2_mat = chrono.ChMaterialSurfaceNSC()
    patch2_mat.SetFriction(0.9)
    patch2_mat.SetRestitution(0.01)
    patch2 = terrain.AddPatch(patch2_mat, chrono.ChVectorD(16, 0, 0.15), chrono.ChVectorD(0, 0, 1), 32, 30);
    patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
    patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

    patch3_mat = chrono.ChMaterialSurfaceNSC()
    patch3_mat.SetFriction(0.9)
    patch3_mat.SetRestitution(0.01)
    patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, -42, 0), chrono.QUNIT),
                              veh.GetDataFile("terrain/meshes/bump.obj"))
    patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
    patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

    patch4_mat = chrono.ChMaterialSurfaceNSC()
    patch4_mat.SetFriction(0.9)
    patch4_mat.SetRestitution(0.01)
    patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 42, 0), chrono.QUNIT),
                              veh.GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0)
    patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    my_hmmwv.GetVehicle().SetVisualSystem(vis)
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddTypicalLights()
    vis.AddSkyBox()

    # Create the interactive driver system
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run() :
        time = my_hmmwv.GetSystem().GetChTime()

        # Draw scene
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        my_hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        my_hmmwv.Advance(step_size)
        vis.Advance(step_size)

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

    return 0



# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step sizes
step_size = 2e-3;
tire_step_size = 1e-3;


main()