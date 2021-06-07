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
// Benchmark for testing changes to rendering algorimths
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 100.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------

    auto floor = std::make_shared<ChBodyEasyBox>(100, 100, .1, 1000, true, true);
    floor->SetPos({0, 0, -1});
    floor->SetBodyFixed(true);
    mphysicalSystem.Add(floor);

    auto first_wall = std::make_shared<ChBodyEasyBox>(.1, 1, 1, 1000, true, true);
    first_wall->SetPos({50.05, 0, 0});
    first_wall->SetBodyFixed(true);
    mphysicalSystem.Add(first_wall);

    auto second_wall = std::make_shared<ChBodyEasyBox>(.1, 10, 10, 1000, true, true);
    second_wall->SetPos({60.05, 0, 0});
    second_wall->SetBodyFixed(true);
    mphysicalSystem.Add(second_wall);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = std::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 1000);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------

    auto lidar1 = std::make_shared<ChLidarSensor>(
        floor,                                                             // body lidar is attached to
        10.0f,                                                             // scanning rate in Hz
        chrono::ChFrame<double>({0, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1,                                                                 // number of horizontal samples
        1,                                                                 // number of vertical channels
        1.0f,                                                              // horizontal field of view
        0.0f, 0.0f, 100.0f                                                 // vertical field of view
    );
    lidar1->SetName("Lidar Sensor");
    lidar1->PushFilter(std::make_shared<ChFilterDIAccess>());
    manager->AddSensor(lidar1);

    auto lidar2 = chrono_types::make_shared<ChLidarSensor>(
        floor,                                                             // body lidar is attached to
        10.0f,                                                             // scanning rate in Hz
        chrono::ChFrame<double>({0, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1,                                                                 // number of horizontal samples
        1,                                                                 // number of vertical channels
        1.0f,                                                              // horizontal field of view
        0.0f, 0.0f, 100.0f,                                                // vertical field of view
        10,                                 // radius of samples to use, 1->1 sample,2->9 samples, 3->25 samples...
        .003f,                              // 3 mradius cited by velodyne
        LidarReturnMode::STRONGEST_RETURN,  // return mode for the lidar
        LidarModelType::RAYCAST             // method/model to use for generating data
    );
    lidar2->SetName("Lidar Sensor");
    // lidar2->PushFilter(std::make_shared<ChFilterLidarNoiseXYZI>(.01f, .001f, .001f, .01f));
    // lidar2->PushFilter(std::make_shared<ChFilterVisualize>(1000,100,"Raw Lidar Depth Data - reduced "));
    // lidar2->PushFilter(std::make_shared<ChFilterDIAccess>());
    manager->AddSensor(lidar2);

    auto lidar3 = chrono_types::make_shared<ChLidarSensor>(
        floor,                                                             // body lidar is attached to
        10.0f,                                                             // scanning rate in Hz
        chrono::ChFrame<double>({0, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1,                                                                 // number of horizontal samples
        1,                                                                 // number of vertical channels
        1.0f,                                                              // horizontal field of view
        0.0f, 0.0f, 100.0f,                                                // vertical field of view
        5,                                  // radius of samples to use, 1->1 sample,2->9 samples, 3->25 samples...
        0.003f,                             // 3 mradius cited by velodyne
        LidarReturnMode::STRONGEST_RETURN,  // return mode for the lidar
        LidarModelType::RAYCAST             // method/model to use for generating data
    );
    lidar3->SetName("Lidar Sensor");
    // lidar2->PushFilter(std::make_shared<ChFilterLidarNoiseXYZI>(.01f, .001f, .001f, .01f));
    // lidar2->PushFilter(std::make_shared<ChFilterVisualize>(1000,100,"Raw Lidar Depth Data - reduced "));
    // lidar3->PushFilter(std::make_shared<ChFilterDIAccess>());
    manager->AddSensor(lidar3);

    utils::CSV_writer csv(" ");

    UserDIBufferPtr data1 = lidar1->GetMostRecentBuffer<UserDIBufferPtr>();
    UserDIBufferPtr data2 = lidar2->GetMostRecentBuffer<UserDIBufferPtr>();
    UserDIBufferPtr data3 = lidar3->GetMostRecentBuffer<UserDIBufferPtr>();

    while (mphysicalSystem.GetChTime() < end_time) {
        // move the wall

        manager->Update();
        mphysicalSystem.DoStepDynamics(0.01);

        UserDIBufferPtr tmp_data1 = lidar1->GetMostRecentBuffer<UserDIBufferPtr>();
        UserDIBufferPtr tmp_data2 = lidar2->GetMostRecentBuffer<UserDIBufferPtr>();
        UserDIBufferPtr tmp_data3 = lidar3->GetMostRecentBuffer<UserDIBufferPtr>();
        if (tmp_data1->Buffer) {
            data1 = tmp_data1;
        }
        if (tmp_data2->Buffer) {
            data2 = tmp_data2;
        }
        if (tmp_data3->Buffer) {
            data3 = tmp_data3;
        }

        if (data1->Buffer && data2->Buffer && data3->Buffer) {
            // std::cout << "Lidar1 range: " << data1->Buffer[0].range << std::endl;
            // std::cout << "Lidar1 intensity: " << data1->Buffer[0].intensity << std::endl;

            csv << first_wall->GetPos().y()    //
                << data1->Buffer[0].range      //
                << data1->Buffer[0].intensity  //
                << data2->Buffer[0].range      //
                << data2->Buffer[0].intensity  //
                << data3->Buffer[0].range      //
                << data3->Buffer[0].intensity  //
                << std::endl;

            first_wall->SetPos(first_wall->GetPos() + ChVector<>({0, .001, 0}));
            std::cout << "y:" << first_wall->GetPos().y() << std::endl;
            data1->Buffer = NULL;
            data2->Buffer = NULL;
            data3->Buffer = NULL;
        }
    }
    csv.write_to_file("lidar_beam_results.csv");
    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
