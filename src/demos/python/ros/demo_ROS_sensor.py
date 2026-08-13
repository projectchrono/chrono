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
# a camera, 3D lidar, 2D lidar, and IMU/GPS suite attached to it are published by
# the built-in sensor handlers - all from Python, no bridge code. Try:
#
#   ros2 topic echo /chrono_ros_node/output/gyroscope/data        # angular_velocity.z ~ 0.1
#   ros2 topic hz   /chrono_ros_node/output/camera/data/image
#   ros2 topic echo /chrono_ros_node/output/lidar/data/pointcloud --once
#
# Like the C++ demo this also pushes Chrono's own ChFilterVisualize preview
# windows; headless they log "XDG_RUNTIME_DIR is invalid" but publishing
# continues fine (the windows just don't open).
#
# =============================================================================

import pychrono as chrono
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    sys = chrono.ChSystemNSC()

    # A mesh body so the camera/lidar have something to see (scene dressing).
    mmesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(
        chrono.GetChronoDataFile("vehicle/audi/audi_chassis.obj"), False, True)
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Audi Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    mesh_body.SetFixed(True)
    sys.Add(mesh_body)

    # The body the sensors are attached to.
    ground_body = chrono.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.SetMass(0)
    sys.Add(ground_body)

    noise_none = sens.ChNoiseNone()
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2),
                                  chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
    gps_reference = chrono.ChVector3d(-89.400, 43.070, 260.0)

    # Sensor manager + scene (lights + environment map are render inputs).
    sensor_manager = sens.ChSensorManager(sys)
    sensor_manager.scene.AddPointLight(chrono.ChVector3f(100, 100, 100), chrono.ChColor(2, 2, 2), 500.0)
    sensor_manager.scene.SetAmbientLight(chrono.ChVector3f(0.1, 0.1, 0.1))
    b = sens.Background()
    b.mode = sens.BackgroundMode_ENVIRONMENT_MAP
    b.env_tex = chrono.GetChronoDataFile("sensor/textures/quarry_01_4k.hdr")
    sensor_manager.scene.SetBackground(b)

    # Camera (bump to 3840x2160 for a 4K throughput check).
    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280, 720, chrono.CH_PI / 3)
    cam.PushFilter(sens.ChFilterRGBA8Access())
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))
    sensor_manager.AddSensor(cam)

    # 3D lidar -> PointCloud2.
    lidar = sens.ChLidarSensor(ground_body, 5.0, offset_pose, 900, 30, 2 * chrono.CH_PI,
                               chrono.CH_PI / 12, -chrono.CH_PI / 6, 100.0)
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 0.50, "3D Lidar"))
    sensor_manager.AddSensor(lidar)

    # 2D lidar -> LaserScan.
    lidar_2d = sens.ChLidarSensor(ground_body, 5.0, offset_pose, 480, 1, 2 * chrono.CH_PI, 0.0, 0.0, 100.0)
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterVisualize(640, 480, "2D Lidar"))
    sensor_manager.AddSensor(lidar_2d)

    # IMU/GPS suite.
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

    # ROS manager + built-in handlers.
    ros_manager = chros.ChROSManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    ros_manager.RegisterHandler(
        chros.ChROSCameraHandler(cam.GetUpdateRate() / 2, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(lidar, "~/output/lidar/data/pointcloud"))
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(lidar_2d, "~/output/lidar_2d/data/laser_scan",
                                chros.ChROSLidarHandlerMessageType_LASER_SCAN))

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

    ros_manager.RegisterHandler(chros.ChROSGPSHandler(gps, "~/output/gps/data"))

    ros_manager.Initialize()

    # Simulation loop, paced to wall-clock.
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
