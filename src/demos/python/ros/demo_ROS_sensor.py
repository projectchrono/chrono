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
# Demo: publishing Chrono::Sensor data to ROS from PYTHON.
#
# Mirrors demo_ROS_sensor.cpp: a body spins with a constant angular velocity and
# IMU/GPS sensors attached to it are published by the built-in sensor handlers -
# all from Python, no bridge code. Try:
#
#   ros2 topic echo /chrono_ros_node/output/gyroscope/data   # angular_velocity.z ~ 0.1
#   ros2 topic echo /chrono_ros_node/output/imu/data
#   ros2 topic echo /chrono_ros_node/output/gps/data
#
# NOTE: camera/lidar return with their handlers in the next Phase-5 batch. These
# IMU/GPS sensors are CPU-side, so no GPU/OptiX is required.
#
# =============================================================================

import pychrono as chrono
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    sys = chrono.ChSystemNSC()

    # The body the sensors are attached to (matches the C++ demo).
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.SetMass(0)
    sys.Add(ground_body)

    noise_none = sens.ChNoiseNone()
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2),
                                  chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
    gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)

    # Sensor manager + the (CPU-side) IMU/GPS sensors. No ray-traced sensors, so
    # no GPU/OptiX is needed.
    sensor_manager = sens.ChSensorManager(sys)

    acc = sens.ChAccelerometerSensor(ground_body, 100, offset_pose, noise_none)
    acc.PushFilter(sens.ChFilterAccelAccess())
    sensor_manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(ground_body, 100, offset_pose, noise_none)
    gyro.PushFilter(sens.ChFilterGyroAccess())
    sensor_manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(ground_body, 100, offset_pose, noise_none, gps_reference)
    mag.PushFilter(sens.ChFilterMagnetAccess())
    sensor_manager.AddSensor(mag)

    gps = sens.ChGPSSensor(ground_body, 5, offset_pose, gps_reference, noise_none)
    gps.PushFilter(sens.ChFilterGPSAccess())
    sensor_manager.AddSensor(gps)

    sensor_manager.Update()

    # ROS manager + built-in sensor handlers (call interfaces match 9.0).
    ros_manager = chros.ChROSManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    acc_handler = chros.ChROSAccelerometerHandler(acc.GetUpdateRate() / 2, acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)

    gyro_handler = chros.ChROSGyroscopeHandler(gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)

    mag_handler = chros.ChROSMagnetometerHandler(mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)

    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)

    gps_handler = chros.ChROSGPSHandler(gps, "~/output/gps/data")
    ros_manager.RegisterHandler(gps_handler)

    ros_manager.Initialize()

    # Simulation loop. A constant angular velocity makes the gyroscope read ~0.1
    # on its z-axis. Paced to wall-clock so it's observable (see the C++ demo).
    time = 0.0
    step_size = 2e-3
    time_end = 1000.0

    ground_body.SetAngVelParent(chrono.ChVector3d(0, 0, 0.1))

    realtime_timer = chrono.ChRealtimeStepTimer()
    while time < time_end:
        time = sys.GetChTime()

        sensor_manager.Update()
        if not ros_manager.Update(time, step_size):
            print("Chrono::ROS bridge node stopped; ending simulation.")
            break

        sys.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)


if __name__ == "__main__":
    main()
