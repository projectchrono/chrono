#include "gtest/gtest.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterRadarProcess.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/utils/Dbscan.h"
#include "chrono_sensor/utils/Kdtree.h"

// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Han Wang
// =============================================================================
//
// Unit test for Radar Sensor
//
// =============================================================================

using namespace chrono;
using namespace sensor;
using namespace chrono::geometry;

#define RADAR_TEST_EPSILLON 1e-6

// tests if the velocity inforatim
TEST(ChRadarSensor, check_velocity) {
//    // -----------------
//    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
//    // Create the system
//    // -----------------
//    ChSystemNSC sys;
//    sys.Set_G_acc(ChVector<>(0, 0, -0));
//
//    // ----------------------
//    // color visual materials
//    // ----------------------
//    auto red = chrono_types::make_shared<ChVisualMaterial>();
//    red->SetDiffuseColor({1, 0, 0});
//    red->SetSpecularColor({1.f, 1.f, 1.f});
//
//    auto green = chrono_types::make_shared<ChVisualMaterial>();
//    green->SetDiffuseColor({0, 1, 0});
//    green->SetSpecularColor({1.f, 1.f, 1.f});
//
//    // -------------------------------------------
//    // add a few box bodies to be sense by a radar
//    // -------------------------------------------
//    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
//    floor->SetPos({0, 0, -1});
//    floor->SetBodyFixed(true);
//    //    floor->SetWvel_par(ChVector<>(-0.2,-0.4,-0.3));
//    //    floor->SetPos_dt(ChVector<>(0.1, 0,0));
//    sys.Add(floor);
//
//    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(5.5, 5.5, 5.5, 1000, true, false);
//    box_body->SetPos({5, 0, 0});
//    box_body->SetPos_dt({0.1, 0, 0});
//    sys.Add(box_body);
//
//    // -----------------------
//    // Create a sensor manager
//    // -----------------------
//    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
//    // -----------------------------------------------
//    // Create a radar and add it to the sensor manager
//    // -----------------------------------------------
//    auto offset_pose = chrono::ChFrame<double>({0, 0, 1}, Q_from_AngZ(0));
//    float horizontal_fov = CH_C_PI / 10;
//    float max_vert_angle = CH_C_PI / 10;
//    float min_vert_angle = -CH_C_PI / 10;
//
//    auto radar = chrono_types::make_shared<ChRadarSensor>(floor, 10.0f, offset_pose, 20, 20, horizontal_fov,
//                                                          max_vert_angle, min_vert_angle, 100.0f);
//    radar->SetName("Radar Sensor");
//    radar->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>());
//    radar->PushFilter(chrono_types::make_shared<ChFilterRadarProcess>("PC from Range"));
//    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZAccess>());
//    manager->AddSensor(radar);
//
//    // -------------------
//    // Simulate the system
//    // -------------------
//    float ch_time = 0.0;
//
//    while (ch_time < 1.0f) {
//        manager->Update();
//
//        sys.DoStepDynamics(1e-3);
//
//        //        UserRadarBufferPtr raw_data = radar->GetMostRecentBuffer<UserRadarBufferPtr>();
//        UserRadarXYZBufferPtr raw_data = radar->GetMostRecentBuffer<UserRadarXYZBufferPtr>();
//        if (raw_data->Buffer) {
//            ASSERT_LT(raw_data->Buffer[0].x_vel - 0.1, RADAR_TEST_EPSILLON);
//        }
//
//        // Get the current time of the simulation
//        ch_time = (float)sys.GetChTime();
//    }
}

TEST(Dbscan, check_cluster) {}

TEST(ChRadarSensor, check_avg_velocity) {}

TEST(ChRadarSensor, check_centroid) {}