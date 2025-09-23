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
# Authors: Harry Zhang, Aaron Young, Radu Serban
# =============================================================================
#
# Demonstration of simulating two vehicles simultaneously.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


def main():
    # print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

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
    hmmwv_1.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(0, -1.5, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)
        )
    )
    hmmwv_1.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv_1.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv_1.SetDriveType(veh.DrivelineTypeWV_RWD)
    hmmwv_1.SetTireType(veh.TireModelType_RIGID)
    hmmwv_1.Initialize()
    hmmwv_1.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv_1.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv_1.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv_1.SetWheelVisualizationType(chrono.VisualizationType_NONE)
    hmmwv_1.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

    # Create the driver system for vehicle 1
    driver_1 = veh.ChDriver(hmmwv_1.GetVehicle())
    driver_1.Initialize()

    # Create ROS manager for vehicle 1
    ros_manager_1 = chros.ChROSPythonManager("hmmwv_1")
    ros_manager_1.RegisterHandler(chros.ChROSClockHandler())
    ros_manager_1.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver_1, "~/input/driver_inputs")
    )
    ros_manager_1.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv_1.GetChassisBody(), "~/output/hmmwv/state")
    )
    ros_manager_1.Initialize()

    # Create and initialize the second vehicle
    hmmwv_2 = veh.HMMWV_Reduced(sys)
    hmmwv_2.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(7, 1.5, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)
        )
    )
    hmmwv_2.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv_2.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv_2.SetDriveType(veh.DrivelineTypeWV_RWD)
    hmmwv_2.SetTireType(veh.TireModelType_RIGID)
    hmmwv_2.Initialize()
    hmmwv_2.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv_2.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv_2.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    hmmwv_2.SetWheelVisualizationType(chrono.VisualizationType_NONE)
    hmmwv_2.SetTireVisualizationType(chrono.VisualizationType_PRIMITIVES)

    # Create the driver system for vehicle 2
    driver_2 = veh.ChDriver(hmmwv_2.GetVehicle())
    driver_2.Initialize()

    # Create ROS manager for vehicle 2
    ros_manager_2 = chros.ChROSPythonManager("hmmwv_2")
    ros_manager_2.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver_2, "~/input/driver_inputs")
    )
    ros_manager_2.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv_2.GetChassisBody(), "~/output/hmmwv/state")
    )
    ros_manager_2.Initialize()

    # Simulation loop
    hmmwv_1.GetVehicle().EnableRealtime(True)
    hmmwv_2.GetVehicle().EnableRealtime(True)

    time = 0
    time_end = 300

    while time < time_end:
        time = hmmwv_1.GetSystem().GetChTime()

        # Get driver inputs
        driver_inputs_1 = driver_1.GetInputs()
        driver_inputs_2 = driver_2.GetInputs()

        # Update modules (process inputs from other modules)
        driver_1.Synchronize(time)
        driver_2.Synchronize(time)
        hmmwv_1.Synchronize(time, driver_inputs_1, terrain)
        hmmwv_2.Synchronize(time, driver_inputs_2, terrain)
        terrain.Synchronize(time)

        # Advance simulation for one timestep for all modules
        driver_1.Advance(step_size)
        driver_2.Advance(step_size)
        hmmwv_1.Advance(step_size)
        hmmwv_2.Advance(step_size)
        terrain.Advance(step_size)

        # Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size)

        if not ros_manager_1.Update(time, step_size) or not ros_manager_2.Update(
            time, step_size
        ):
            break

    return 0


if __name__ == "__main__":
    main()
