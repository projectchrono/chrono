// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Demo: publishing Chrono::Sensor data to ROS through the schema-driven bridge.
//
// A body spins with a constant angular velocity; a camera, 3D lidar, 2D lidar,
// and an IMU/GPS suite are attached to it and published by the built-in sensor
// handlers - no bridge code, no message-type wrapping. Try:
//
//   ros2 topic echo /chrono_ros_node/output/gyroscope/data    # angular_velocity.z ~ 0.1
//   ros2 topic hz   /chrono_ros_node/output/camera/data/image
//   ros2 topic echo /chrono_ros_node/output/lidar/data/pointcloud --field width
//
// NOTE on visualization: this pushes Chrono's own ChFilterVisualize (camera) and
// ChFilterVisualizePointCloud (lidar) preview windows alongside the data-access
// filters the ROS handlers read. On a headless host (e.g. inside Docker) these
// log "XDG_RUNTIME_DIR is invalid" but the simulation and ROS publishing continue
// normally - the preview windows simply don't open. Drop those two PushFilter
// calls if you want to silence the warning.
//
// =============================================================================

#include "chrono/core/ChTypes.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

#include <iostream>

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::sensor;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    ChSystemNSC sys;

    // A mesh body so the camera/lidar have something to see (scene dressing).
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
        GetChronoDataFile("vehicle/audi/audi_chassis.obj"), false, true);
    mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Audi Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(ChVector3d(0, 0, 0)));
    mesh_body->SetFixed(true);
    sys.Add(mesh_body);

    // The body the sensors are attached to.
    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetFixed(false);
    ground_body->SetMass(0);
    sys.Add(ground_body);

    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    ChFrame<double> offset_pose({-8, 0, 2}, QuatFromAngleAxis(0.2, {0, 1, 0}));
    ChVector3d gps_reference(-89.400, 43.070, 260.0);

    // Sensor manager + scene (lights + environment map are render inputs; no display needed).
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(&sys);
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    sensor_manager->scene->SetBackground(b);

    // Camera (bump to 3840x2160 for a 4K throughput check). The
    // ChFilterRGBA8Access feeds the ROS handler; ChFilterVisualize is Chrono's
    // own preview window.
    auto cam = chrono_types::make_shared<ChCameraSensor>(ground_body, 30, offset_pose, 1280, 720, CH_PI / 3.);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));
    sensor_manager->AddSensor(cam);

    // 3D lidar -> PointCloud2 (DI raycast -> point cloud -> XYZI access).
    auto lidar = chrono_types::make_shared<ChLidarSensor>(ground_body, 5.f, offset_pose, 900, 30, 2 * CH_PI,
                                                          CH_PI / 12, -CH_PI / 6, 100.0f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 0.50, "3D Lidar"));
    sensor_manager->AddSensor(lidar);

    // 2D lidar -> LaserScan (single ring; depth/intensity access).
    auto lidar_2d =
        chrono_types::make_shared<ChLidarSensor>(ground_body, 5.f, offset_pose, 480, 1, 2 * CH_PI, 0.0, 0.0, 100.0f);
    lidar_2d->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar_2d->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 480, "2D Lidar"));
    sensor_manager->AddSensor(lidar_2d);

    // IMU/GPS suite.
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(ground_body, 100.f, offset_pose, noise_none);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    sensor_manager->AddSensor(acc);
    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(ground_body, 100.f, offset_pose, noise_none);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    sensor_manager->AddSensor(gyro);
    auto mag = chrono_types::make_shared<ChMagnetometerSensor>(ground_body, 100.f, offset_pose, noise_none, gps_reference);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    sensor_manager->AddSensor(mag);
    auto gps = chrono_types::make_shared<ChGPSSensor>(ground_body, 5.f, offset_pose, gps_reference, noise_none);
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    sensor_manager->AddSensor(gps);

    sensor_manager->Update();

    // ------------

    // ROS manager + built-in handlers. Per-sensor TF frames are published below
    // via the ChROSTFHandler AddSensor overload.
    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());

    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSCameraHandler>(
        cam->GetUpdateRate() / 2, cam, "~/output/camera/data/image"));

    ros_manager->RegisterHandler(
        chrono_types::make_shared<ChROSLidarHandler>(lidar, "~/output/lidar/data/pointcloud"));

    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSLidarHandler>(
        lidar_2d, "~/output/lidar_2d/data/laser_scan", ChROSLidarHandlerMessageType::LASER_SCAN));

    auto acc_handler =
        chrono_types::make_shared<ChROSAccelerometerHandler>(acc->GetUpdateRate() / 2, acc, "~/output/accelerometer/data");
    ros_manager->RegisterHandler(acc_handler);
    auto gyro_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyro, "~/output/gyroscope/data");
    ros_manager->RegisterHandler(gyro_handler);
    auto mag_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(mag, "~/output/magnetometer/data");
    ros_manager->RegisterHandler(mag_handler);

    auto imu_handler = chrono_types::make_shared<ChROSIMUHandler>(100, "~/output/imu/data");
    imu_handler->SetAccelerometerHandler(acc_handler);
    imu_handler->SetGyroscopeHandler(gyro_handler);
    imu_handler->SetMagnetometerHandler(mag_handler);
    ros_manager->RegisterHandler(imu_handler);

    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSGPSHandler>(gps, "~/output/gps/data"));

    // One TF handler broadcasting each sensor's frame (its offset pose) relative
    // to the body it's mounted on.
    auto tf_handler = chrono_types::make_shared<ChROSTFHandler>(100);
    tf_handler->AddSensor(cam, ground_body->GetName(), "camera");
    tf_handler->AddSensor(lidar, ground_body->GetName(), "lidar");
    tf_handler->AddSensor(lidar_2d, ground_body->GetName(), "lidar_2d");
    tf_handler->AddSensor(acc, ground_body->GetName(), "accelerometer");
    tf_handler->AddSensor(gyro, ground_body->GetName(), "gyroscope");
    tf_handler->AddSensor(mag, ground_body->GetName(), "magnetometer");
    tf_handler->AddSensor(gps, ground_body->GetName(), "gps");
    ros_manager->RegisterHandler(tf_handler);

    ros_manager->Initialize();

    // ------------

    double time = 0;
    double step_size = 2e-3;
    double time_end = 1000;

    ground_body->SetAngVelParent({0, 0, 0.1});

    ChRealtimeStepTimer realtime_timer;
    while (time < time_end) {
        time = sys.GetChTime();

        sensor_manager->Update();
        if (!ros_manager->Update(time, step_size)) {
            std::cerr << "Chrono::ROS bridge node stopped; ending simulation." << std::endl;
            break;
        }

        sys.DoStepDynamics(step_size);
        realtime_timer.Spin(step_size);
    }

    return 0;
}
