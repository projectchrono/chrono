# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Aaron Young
# =============================================================================
#
# Demo to show the use of Chrono::Sensor with ROS in python
#
# =============================================================================

import pychrono as ch
import pychrono.sensor as sens
import pychrono.ros as chros


def main():
    # Create the system
    sys = ch.ChSystemNSC()

    # Add a mesh object to make the scene interesting
    mmesh = ch.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile(
        "vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(1))

    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)
    mesh_body.SetMass(0)
    sys.Add(mesh_body)

    # This is the body we'll attach the sensors to
    ground_body = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground_body.SetPos(ch.ChVector3d(0, 0, 0))
    ground_body.SetFixed(False)
    ground_body.SetMass(0)
    sys.Add(ground_body)

    # Create the sensor manager
    sens_manager = sens.ChSensorManager(sys)

    intensity = 1.0
    sens_manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100), ch.ChColor(
        intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100), ch.ChColor(
        intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100), ch.ChColor(
        intensity, intensity, intensity), 500.0)
    sens_manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100), ch.ChColor(
        intensity, intensity, intensity), 500.0)

    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))

    cam = sens.ChCameraSensor(ground_body, 30, offset_pose, 1280,  720, 1.408)
    cam.PushFilter(sens.ChFilterVisualize(1280, 720))
    cam.PushFilter(sens.ChFilterRGBA8Access())
    cam.SetName("camera")
    sens_manager.AddSensor(cam)

    lidar = sens.ChLidarSensor(ground_body, 5., offset_pose, 90, 300,
                               2*ch.CH_PI, ch.CH_PI / 12, -ch.CH_PI / 6, 100., 0)
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(1280, 720, 1))
    lidar.SetName("lidar")
    sens_manager.AddSensor(lidar)

    noise_model_none = sens.ChNoiseNone()
    gps_reference = ch.ChVector3d(-89.4, 433.07, 260.)
    gps = sens.ChGPSSensor(ground_body, 10, offset_pose,
                           gps_reference, noise_model_none)
    gps.PushFilter(sens.ChFilterGPSAccess())
    gps.SetName("gps")
    sens_manager.AddSensor(gps)

    acc = sens.ChAccelerometerSensor(
        ground_body, 100, offset_pose, noise_model_none)
    acc.PushFilter(sens.ChFilterAccelAccess())
    acc.SetName("accelerometer")
    sens_manager.AddSensor(acc)

    gyro = sens.ChGyroscopeSensor(
        ground_body, 100, offset_pose, noise_model_none)
    gyro.PushFilter(sens.ChFilterGyroAccess())
    gyro.SetName("gyroscope")
    sens_manager.AddSensor(gyro)

    mag = sens.ChMagnetometerSensor(
        ground_body, 100, offset_pose, noise_model_none, gps_reference)
    mag.PushFilter(sens.ChFilterMagnetAccess())
    mag.SetName("magnetometer")
    sens_manager.AddSensor(mag)

    sens_manager.Update()

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSCameraHandler(
        cam.GetUpdateRate() / 4, cam, "~/output/camera/data/image"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(
        lidar, "~/output/lidar/data/pointcloud"))
    ros_manager.RegisterHandler(
        chros.ChROSGPSHandler(gps, "~/output/gps/data"))
    acc_handler = chros.ChROSAccelerometerHandler(
        acc, "~/output/accelerometer/data")
    ros_manager.RegisterHandler(acc_handler)
    gyro_handler = chros.ChROSGyroscopeHandler(
        gyro, "~/output/gyroscope/data")
    ros_manager.RegisterHandler(gyro_handler)
    mag_handler = chros.ChROSMagnetometerHandler(
        mag, "~/output/magnetometer/data")
    ros_manager.RegisterHandler(mag_handler)
    imu_handler = chros.ChROSIMUHandler(100, "~/output/imu/data")
    imu_handler.SetAccelerometerHandler(acc_handler)
    imu_handler.SetGyroscopeHandler(gyro_handler)
    imu_handler.SetMagnetometerHandler(mag_handler)
    ros_manager.RegisterHandler(imu_handler)
    ros_manager.Initialize()

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 100

    # Give the ground body some rotational velocity so that the sensors attached to it appear to be moving
    # Note how the gyroscopes angular velocity in ROS will read 0.1 on the z-axis
    ground_body.SetAngVelParent(ch.ChVector3d(0, 0, 0.1))
    while time < time_end:
        time = sys.GetChTime()

        # Updates
        sens_manager.Update()
        if not ros_manager.Update(time, time_step):
            break

        sys.DoStepDynamics(time_step)


if __name__ == "__main__":
    main()
