// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Chrono demonstration of a camera sensor
// Load in sensor settings and filter list using JSON
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iomanip>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 20.0f;
bool save_data = false;
bool vis = true;

float ranf() {
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
                                                                  false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(2));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("HMMWV Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape,ChFrame<>());
    mesh_body->SetBodyFixed(true);
    sys.Add(mesh_body);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    float intensity = .5;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({2, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    manager->scene->AddPointLight({9, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    manager->scene->AddPointLight({16, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    manager->scene->AddPointLight({23, 2.5, 100}, {intensity, intensity, intensity}, 5000);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto cam = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Camera.json"), mesh_body,
                                      ChFrame<>({-5, 0, 0}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(cam);

    // -----------------------------------------------
    // Create a lidar and add it to the sensor manager
    // -----------------------------------------------
    auto lidar = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Lidar.json"), mesh_body,
                                        ChFrame<>({-5, 0, .5}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(lidar);

    // ---------------------------------------------
    // Create a gps and add it to the sensor manager
    // ---------------------------------------------
    auto gps = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/GPS.json"), mesh_body,
                                      ChFrame<>({0, 0, 0}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(gps);

    // ---------------------------------------------
    // Create a imu and add it to the sensor manager
    // ---------------------------------------------
    auto acc = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Accelerometer.json"), mesh_body,
                                      ChFrame<>({0, 0, 0}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(acc);

    auto gyro = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Gyroscope.json"), mesh_body,
                                       ChFrame<>({0, 0, 0}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(gyro);

    auto mag = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Magnetometer.json"), mesh_body,
                                      ChFrame<>({0, 0, 0}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(mag);

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 10.f;
    float orbit_rate = 0.5f;
    float ch_time = 0.0f;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    int num_camera_updates = 0;
    int num_lidar_updates = 0;
    int num_gps_updates = 0;
    int num_imu_updates = 0;

    while (ch_time < end_time) {
        sys.DoStepDynamics(0.001);
        manager->Update();

        cam->SetOffsetPose(chrono::ChFrame<double>(
            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 3},
            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

        UserR8BufferPtr camera_data = cam->GetMostRecentBuffer<UserR8BufferPtr>();
        if (camera_data->Buffer) {
            num_camera_updates++;
            // std::cout << "Data recieved from camera. Frame: " << num_camera_updates << std::endl;
        }

        UserXYZIBufferPtr lidar_data = lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
        if (lidar_data->Buffer) {
            num_lidar_updates++;
            // std::cout << "Data recieved from lidar. Frame: " << num_lidar_updates << std::endl;
        }

        UserGPSBufferPtr gps_data = gps->GetMostRecentBuffer<UserGPSBufferPtr>();
        if (gps_data->Buffer) {
            num_gps_updates++;
            // std::cout << "Data recieved from gps. Frame: " << num_gps_updates << std::endl;
        }

        UserAccelBufferPtr acc_data = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        if (acc_data->Buffer) {
            num_imu_updates++;
            // std::cout << "Data recieved from imu. Frame: " << num_imu_updates << std::endl;
        }

        ch_time = (float)sys.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
