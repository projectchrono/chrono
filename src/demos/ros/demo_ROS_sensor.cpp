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
// A body is given a constant angular velocity and several sensors are attached
// to it. The built-in sensor handlers publish each sensor on the new API - no
// bridge code, no message-type wrapping. Try:
//
//   ros2 topic echo /chrono_ros_node/output/gyroscope/data    # angular_velocity.z ~ 0.1
//   ros2 topic echo /chrono_ros_node/output/imu/data
//   ros2 topic echo /chrono_ros_node/output/gps/data
//
// NOTE: this mirrors the Chrono 9.0 demo_ROS_sensor's IMU/GPS setup. The camera
// and lidar sensors (and the tf AddSensor transforms) return in the next Phase-5
// batch together with the camera/lidar handlers; the sim setup here is otherwise
// identical to 9.0 so behavior is comparable.
//
// =============================================================================

#include "chrono/core/ChTypes.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include <iostream>

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::sensor;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    // Create the system
    ChSystemNSC sys;

    // The body the sensors are attached to (matches the 9.0 demo).
    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetFixed(false);
    ground_body->SetMass(0);
    sys.Add(ground_body);

    auto noise_none = chrono_types::make_shared<ChNoiseNone>();
    ChFrame<double> offset_pose({-8, 0, 2}, QuatFromAngleAxis(0.2, {0, 1, 0}));
    ChVector3d gps_reference(-89.400, 43.070, 260.0);

    // Create the sensor manager and attach the (CPU-side) IMU/GPS sensors. No
    // ray-traced sensors here, so no GPU/OptiX is required.
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(&sys);

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

    // Create the ROS manager and register the built-in handlers (call interfaces
    // match 9.0). Sensor handlers default to the sensor's own update rate; pass a
    // rate to override (shown for the accelerometer).
    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());

    auto acc_handler =
        chrono_types::make_shared<ChROSAccelerometerHandler>(acc->GetUpdateRate() / 2, acc, "~/output/accelerometer/data");
    ros_manager->RegisterHandler(acc_handler);

    auto gyro_handler = chrono_types::make_shared<ChROSGyroscopeHandler>(gyro, "~/output/gyroscope/data");
    ros_manager->RegisterHandler(gyro_handler);

    auto mag_handler = chrono_types::make_shared<ChROSMagnetometerHandler>(mag, "~/output/magnetometer/data");
    ros_manager->RegisterHandler(mag_handler);

    // The IMU handler aggregates the accelerometer + gyroscope (+ magnetometer)
    // handlers into one sensor_msgs/Imu. The sub-handlers above tick on their own.
    auto imu_handler = chrono_types::make_shared<ChROSIMUHandler>(100, "~/output/imu/data");
    imu_handler->SetAccelerometerHandler(acc_handler);
    imu_handler->SetGyroscopeHandler(gyro_handler);
    imu_handler->SetMagnetometerHandler(mag_handler);
    ros_manager->RegisterHandler(imu_handler);

    auto gps_handler = chrono_types::make_shared<ChROSGPSHandler>(gps, "~/output/gps/data");
    ros_manager->RegisterHandler(gps_handler);

    ros_manager->Initialize();

    // ------------

    // Simulation loop (same physics setup as 9.0). A constant angular velocity
    // on the body makes the gyroscope read ~0.1 on its z-axis.
    //
    // The 9.0 demo had no realtime timer because the camera/lidar GPU rendering
    // paced each step. With those sensors deferred (batch 3b), this CPU-only sim
    // would otherwise run thousands of x faster than wall time and finish almost
    // instantly, so a ChRealtimeStepTimer paces it to wall-clock - keeping it
    // observable and publishing at the sensors' real rates. The timer stays
    // correct once camera/lidar return.
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
