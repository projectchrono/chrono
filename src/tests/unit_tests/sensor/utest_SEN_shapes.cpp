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
// Authors: Asher Elmquist
// =============================================================================
//
// Unit test for MatrMultiplyAVX and MatrMultiplyTAVX.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/assets/ChBoxShape.h"

// sensor includes
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace sensor;

#define end_time 10.0

// for making sure we can add sensors safely while the simulation is running
TEST(ChOptixEngine, assign_sensor_safety) {
    ChSystemNSC mphysicalSystem;

    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(.5, .5, .5, 1000, false, true);
    box_body->SetBodyFixed(true);
    box_body->SetPos({0, 4, 1});
    mphysicalSystem.Add(box_body);

    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.25, 1000, false, true);
    sphere_body->SetBodyFixed(true);
    sphere_body->SetPos({0, 2, 1});
    mphysicalSystem.Add(sphere_body);

    auto cylinder_body = chrono_types::make_shared<ChBodyEasyCylinder>(.25, .5, 1000, false, true);
    cylinder_body->SetBodyFixed(true);
    cylinder_body->SetPos({0, 0, 1});
    mphysicalSystem.Add(cylinder_body);

    auto mesh_body = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("sensor/geometries/suzanne.obj"), 1000,
                                                               false, true);
    mesh_body->SetBodyFixed(true);
    mesh_body->SetFrame_REF_to_abs(ChFrame<>({0, 0, 3}, {1, 0, 0, 0}));
    mphysicalSystem.Add(mesh_body);
    std::dynamic_pointer_cast<ChVisualization>(mesh_body->GetAssetN(0))->SetStatic(true);

    auto box_item = chrono_types::make_shared<ChLoadContainer>();
    auto bshape = chrono_types::make_shared<ChBoxShape>();
    bshape->Pos = {0, 4, -1};
    bshape->GetBoxGeometry().SetLengths(ChVector<>(.5, .5, .5));
    box_item->AddAsset(bshape);
    mphysicalSystem.Add(box_item);

    auto sphere_item = chrono_types::make_shared<ChLoadContainer>();
    auto sshape = chrono_types::make_shared<ChSphereShape>();
    sshape->Pos = {0, 2, -1};
    sshape->GetSphereGeometry().rad = .25;
    sphere_item->AddAsset(sshape);
    mphysicalSystem.Add(sphere_item);

    auto cylinder_item = chrono_types::make_shared<ChLoadContainer>();
    auto cshape = chrono_types::make_shared<ChCylinderShape>();
    // cshape->Pos = {0, 0, -1};
    cshape->GetCylinderGeometry().rad = .25;
    cshape->GetCylinderGeometry().p1 = {0, .25, -1};
    cshape->GetCylinderGeometry().p2 = {0, -.25, -1};
    cylinder_item->AddAsset(cshape);
    mphysicalSystem.Add(cylinder_item);

    auto mesh_item = chrono_types::make_shared<ChLoadContainer>();
    // auto cshape = chrono_types::make_shared<ChCylinderShape>();
    // cshape->Pos = {0, 0, -1};
    // cshape->GetCylinderGeometry().rad = .25;
    // cshape->GetCylinderGeometry().p1 = {0, .25, -1};
    // cshape->GetCylinderGeometry().p2 = {0, -.25, -1};
    auto mshape = std::dynamic_pointer_cast<ChTriangleMeshShape>(mesh_body->GetAssetN(0));
    mshape->Pos = {0, 6, -2};
    mshape->SetStatic(true);
    mesh_item->AddAsset(mshape);
    mphysicalSystem.Add(mesh_item);

    // auto sphere_item = chrono_types::make_shared<ChBodyEasySphere>(.25, 1000, false, true);
    // sphere_item->SetBodyFixed(true);
    // sphere_item->SetPos({0, 2, 1});
    // mphysicalSystem.Add(sphere_item);
    //
    // auto cylinder_item = chrono_types::make_shared<ChBodyEasyCylinder>(.25, .5, 1000, false, true);
    // cylinder_item->SetBodyFixed(true);
    // cylinder_item->SetPos({0, 0, 1});
    // mphysicalSystem.Add(cylinder_item);

    // auto mesh_item =
    //     chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("sensor/geometries/suzanne.obj"), 1000, false,
    //     true);
    // mesh_item->SetBodyFixed(true);
    // mesh_item->SetFrame_REF_to_abs(ChFrame<>({0, 6, 1}, {1, 0, 0, 0}));
    // mphysicalSystem.Add(mesh_item);
    // std::dynamic_pointer_cast<ChVisualization>(mesh_item->GetAssetN(0))->SetStatic(true);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({-100, 0, 100}, {1, 1, 1}, 500);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box_body,                                                           // body camera is attached to
        10,                                                                 // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        640,                                                                // image width
        480,                                                                // image height
        CH_C_PI / 3);                                                       // FOV
    cam->SetName("Camera Sensor");

    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 480, "RGB Camera"));
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    bool success = true;

    double ch_time = 0;

    while (ch_time < end_time) {
        manager->Update();
        mphysicalSystem.DoStepDynamics(0.01);

        ch_time = (float)mphysicalSystem.GetChTime();
        // UserRGBA8BufferPtr camera_data = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        // LockedRGBA8BufferPtr camera_data = std::move(cam->GetMostRecentBuffer<LockedRGBA8BufferPtr>());
    }

    ASSERT_EQ(success, true);
}
