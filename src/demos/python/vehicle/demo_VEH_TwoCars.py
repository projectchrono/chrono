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
# Demonstration of simulating two vehicles simultaneously.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    step_size = 0.005

    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    # Create the terrain
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create and initialize the first vehicle
    hmmwv_1 = veh.HMMWV_Reduced(sys)
    hmmwv_1.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -1.5, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv_1.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv_1.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv_1.SetDriveType(veh.DrivelineTypeWV_RWD)
    hmmwv_1.SetTireType(veh.TireModelType_RIGID)
    hmmwv_1.Initialize()
    hmmwv_1.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_1.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_1.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_1.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv_1.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    driver_data_1 = veh.vector_Entry([veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0), 
                                      veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0, 0.0),
                                      veh.DataDriverEntry(0.7, 0.3, 0.7, 0.0, 0.0),
                                      veh.DataDriverEntry(1.0, 0.3, 0.7, 0.0, 0.0),
                                      veh.DataDriverEntry(3.0, 0.5, 0.1, 0.0, 0.0)
                                     ])
    driver_1 = veh.ChDataDriver(hmmwv_1.GetVehicle(), driver_data_1)
    driver_1.Initialize()

    # Create and initialize the second vehicle
    hmmwv_2 = veh.HMMWV_Reduced(sys)
    hmmwv_2.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(7, 1.5, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv_2.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv_2.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv_2.SetDriveType(veh.DrivelineTypeWV_RWD)
    hmmwv_2.SetTireType(veh.TireModelType_RIGID)
    hmmwv_2.Initialize()
    hmmwv_2.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_2.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_2.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv_2.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv_2.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    driver_data_2 = veh.vector_Entry([veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0), 
                                      veh.DataDriverEntry(0.5, 0.0, 0.0, 0.0, 0.0),
                                      veh.DataDriverEntry(0.7, -0.3, 0.7, 0.0, 0.0),
                                      veh.DataDriverEntry(1.0, -0.3, 0.7, 0.0, 0.0),
                                      veh.DataDriverEntry(3.0, -0.5, 0.1, 0.0, 0.0)
                                     ])
    driver_2 = veh.ChDataDriver(hmmwv_2.GetVehicle(), driver_data_2)
    driver_2.Initialize()


    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Two Car Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.SetChaseCameraState(veh.ChChaseCamera.Track)
    vis.SetChaseCameraPosition(chrono.ChVector3d(-10, 0, 2.0))
    vis.Initialize()
    vis.AddLightDirectional()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv_1.GetVehicle())


    # Simulation loop
    hmmwv_1.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = hmmwv_1.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs_1 = driver_1.GetInputs()
        driver_inputs_2 = driver_2.GetInputs()

        # Update modules (process inputs from other modules)
        driver_1.Synchronize(time)
        driver_2.Synchronize(time)
        hmmwv_1.Synchronize(time, driver_inputs_1, terrain)
        hmmwv_2.Synchronize(time, driver_inputs_2, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs_1)

        # Advance simulation for one timestep for all modules
        driver_1.Advance(step_size)
        driver_2.Advance(step_size)
        hmmwv_1.Advance(step_size)
        hmmwv_2.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

        # Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size)

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

main()