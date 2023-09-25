// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Demo to show the use of Chrono::Sensor with ROS
//
// =============================================================================

#include "chrono/core/ChTypes.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

#include <chrono>

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::sensor;
using namespace chrono::geometry;

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2023 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the system
    ChSystemNSC sys;

    // Add a mesh object to make the scene interesting
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                                                                  false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Audi Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(ChVector<>(0, 0, 0)));
    mesh_body->SetBodyFixed(true);
    sys.Add(mesh_body);

    // This is the body we'll attach the sensors to
    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetBodyFixed(false);
    ground_body->SetMass(0);
    sys.Add(ground_body);

    // -----------------------
    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    chrono::ChFrame<double> offset_pose({-8, 0, 2}, Q_from_AngAxis(.2, {0, 1, 0}));

    // Create the sensor system
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(&sys);
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});

    // Set the background to an environment map
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    sensor_manager->scene->SetBackground(b);

    // Create a camera that's placed on the hood
    auto cam = chrono_types::make_shared<ChCameraSensor>(ground_body, 30, offset_pose, 1280, 720, CH_C_PI / 3.);
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720));
    sensor_manager->AddSensor(cam);

    // Create a lidar and add it to the sensor manager
    auto lidar = chrono_types::make_shared<ChLidarSensor>(ground_body, 5.f, offset_pose, 900, 30, 2 * CH_C_PI,
                                                          CH_C_PI / 12, -CH_C_PI / 6, 100.0f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 1));
    sensor_manager->AddSensor(lidar);

    // add an accelerometer, gyroscope, and magnetometer
    ChVector<> gps_reference(-89.400, 43.070, 260.0);
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(ground_body, 100.f, offset_pose, noise_none);
    acc->PushFilter(chrono_types::make_shared<ChFilterAccelAccess>());
    sensor_manager->AddSensor(acc);

    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(ground_body, 100.f, offset_pose, noise_none);
    gyro->PushFilter(chrono_types::make_shared<ChFilterGyroAccess>());
    sensor_manager->AddSensor(gyro);

    auto mag =
        chrono_types::make_shared<ChMagnetometerSensor>(ground_body, 100.f, offset_pose, noise_none, gps_reference);
    mag->PushFilter(chrono_types::make_shared<ChFilterMagnetAccess>());
    sensor_manager->AddSensor(mag);

    // add a GPS sensor
    auto gps = chrono_types::make_shared<ChGPSSensor>(ground_body, 5.f, offset_pose, gps_reference, noise_none);
    gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    sensor_manager->AddSensor(gps);
    sensor_manager->Update();

    // ------------

    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create the publisher for the camera
    // Let's say I want to only publish the camera data at half the rate specified in the simulation
    // Override that with the constructor that takes a rate
    // All sensor handlers accept a rate, otherwise it's just the sensor's update rate
    auto camera_rate = cam->GetUpdateRate() / 2;
    auto camera_topic_name = "~/output/camera/data/image";
    auto camera_handler = chrono_types::make_shared<ChROSCameraHandler>(camera_rate, cam, camera_topic_name);
    ros_manager->RegisterHandler(camera_handler);

    // Create the publisher for the lidar
    auto lidar_topic_name = "~/output/lidar/data/pointcloud";
    auto lidar_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_topic_name);
    ros_manager->RegisterHandler(lidar_handler);

    // Create the publisher for the accelerometer
    // Let's say I want to only publish the imu data at half the rate specified in the simulation
    // Override that with the constructor that takes a rate
    // All sensor handlers accept a rate, otherwise it's just the sensor's update rate
    auto acc_rate = acc->GetUpdateRate() / 2;
    auto acc_topic_name = "~/output/accelerometer/data";
    auto acc_handler = chrono_types::make_shared<ChROSAccelerometerHandler>(acc_rate, acc, acc_topic_name);
    ros_manager->RegisterHandler(acc_handler);

    // Create the publisher for the gyroscope
    auto gyro_topic_name = "~/output/gyroscope/data";
    auto gyro_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyro, gyro_topic_name);
    ros_manager->RegisterHandler(gyro_handler);

    // Create the publisher for the magnetometer
    auto mag_topic_name = "~/output/magnetometer/data";
    auto mag_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(mag, mag_topic_name);
    ros_manager->RegisterHandler(mag_handler);

    // Create the publisher for the GPS
    auto gps_topic_name = "~/output/gps/data";
    auto gps_handler = chrono_types::make_shared<ChROSGPSHandler>(gps, gps_topic_name);
    ros_manager->RegisterHandler(gps_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    // ------------

    // Simulation
    double time = 0;
    double step_size = 2e-3;
    double time_end = 30;

    // Give the ground body some rotational velocity so that the sensors attached to it appear to be moving
    // Note how the gyroscopes angular velocity in ROS will read 0.1 on the z-axis
    ground_body->SetWvel_par({0, 0, 0.1});

    // Simulation loop
    while (time < time_end) {
        time = sys.GetChTime();

        // Updates
        sensor_manager->Update();
        if (!ros_manager->Update(time, step_size))
            break;

        sys.DoStepDynamics(step_size);
    }

    return 0;
}
