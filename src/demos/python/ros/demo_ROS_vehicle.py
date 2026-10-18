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
# Authors: Aaron Young, Patrick Chen
# =============================================================================
#
# Chrono::Vehicle + ROS, in Python. An HMMWV is driven from ROS
# (chrono_ros_interfaces/msg/DriverInputs) and its chassis state is published.
# The ChROSDriverInputsHandler / ChROSBodyHandler are the same C++ built-ins used
# by the C++ demo, here taking a pychrono.vehicle ChDriver / ChBody.
#
#   ros2 topic echo /chrono_ros_node/output/vehicle/state/pose
#   ros2 topic pub  /chrono_ros_node/input/driver_inputs \
#       chrono_ros_interfaces/msg/DriverInputs "{throttle: 0.6}"
#
# =============================================================================

import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros


def main():
    # Create the HMMWV vehicle, set parameters, and initialize.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()

    # Create the terrain.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    terrain.Initialize()

    # Create the basic driver (commanded from ROS).
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # ROS: clock + DriverInputs subscriber + chassis state publisher.
    ros_manager = chros.ChROSManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/vehicle/state"))
    ros_manager.Initialize()

    # Simulation loop (real-time paced).
    time = 0
    time_step = 1e-3
    time_end = 30

    hmmwv.GetVehicle().EnableRealtime(True)
    while time < time_end:
        time = hmmwv.GetSystem().GetChTime()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        if not ros_manager.Update(time, time_step):
            break


if __name__ == "__main__":
    main()
