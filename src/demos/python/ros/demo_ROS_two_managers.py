# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Harry Zhang, Aaron Young, Radu Serban, Patrick Chen
# =============================================================================
#
# Two vehicles simulated simultaneously, each served by its own ChROSManager
# (node namespaces "hmmwv_1" and "hmmwv_2"), in Python.
#
#   ros2 topic echo /hmmwv_1/output/vehicle/state/pose
#   ros2 topic pub  /hmmwv_2/input/driver_inputs \
#       chrono_ros_interfaces/msg/DriverInputs "{steering: 0.4, throttle: 0.4}"
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


def main():
    step_size = 0.005

    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    # Terrain.
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # ROS handler rates / topics (shared, scoped per node).
    driver_inputs_rate = 25
    vehicle_state_rate = 25
    driver_inputs_topic = "~/input/driver_inputs"
    vehicle_state_topic = "~/output/vehicle/state"

    # First vehicle.
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

    driver_1 = veh.ChDriver(hmmwv_1.GetVehicle())
    driver_1.Initialize()

    ros_manager_1 = chros.ChROSManager("hmmwv_1")
    ros_manager_1.RegisterHandler(chros.ChROSClockHandler())
    ros_manager_1.RegisterHandler(chros.ChROSDriverInputsHandler(driver_inputs_rate, driver_1, driver_inputs_topic))
    ros_manager_1.RegisterHandler(chros.ChROSBodyHandler(vehicle_state_rate, hmmwv_1.GetChassisBody(), vehicle_state_topic))
    ros_manager_1.Initialize()

    # Second vehicle.
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

    driver_2 = veh.ChDriver(hmmwv_2.GetVehicle())
    driver_2.Initialize()

    ros_manager_2 = chros.ChROSManager("hmmwv_2")
    ros_manager_2.RegisterHandler(chros.ChROSDriverInputsHandler(driver_inputs_rate, driver_2, driver_inputs_topic))
    ros_manager_2.RegisterHandler(chros.ChROSBodyHandler(vehicle_state_rate, hmmwv_2.GetChassisBody(), vehicle_state_topic))
    ros_manager_2.Initialize()

    # Simulation loop.
    hmmwv_1.GetVehicle().EnableRealtime(True)
    hmmwv_2.GetVehicle().EnableRealtime(True)

    time = 0
    time_end = 30

    while time < time_end:
        time = hmmwv_1.GetSystem().GetChTime()

        driver_inputs_1 = driver_1.GetInputs()
        driver_inputs_2 = driver_2.GetInputs()

        driver_1.Synchronize(time)
        driver_2.Synchronize(time)
        hmmwv_1.Synchronize(time, driver_inputs_1, terrain)
        hmmwv_2.Synchronize(time, driver_inputs_2, terrain)
        terrain.Synchronize(time)

        driver_1.Advance(step_size)
        driver_2.Advance(step_size)
        hmmwv_1.Advance(step_size)
        hmmwv_2.Advance(step_size)
        terrain.Advance(step_size)

        sys.DoStepDynamics(step_size)

        if not ros_manager_1.Update(time, step_size) or not ros_manager_2.Update(time, step_size):
            break


if __name__ == "__main__":
    main()
