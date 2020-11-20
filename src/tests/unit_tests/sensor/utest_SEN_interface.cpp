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
// Unit test for interface between Chrono and Chrono::Sensor (making sure
// objects placed in Chrono appear in Chrono::Sensor)
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChTriangleMeshShape.h"

using namespace chrono;
using namespace chrono::sensor;
using namespace chrono::geometry;

const float SIM_RUN_TIME = 10.0;

TEST(SensorInterface, cameras) {
    ChSystemNSC mphysicalSystem;

    auto boxA = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, true, false);
    boxA->SetPos({2, 0, 0});
    boxA->SetBodyFixed(true);
    mphysicalSystem.Add(boxA);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);

    auto cam1 = chrono_types::make_shared<ChCameraSensor>(
        boxA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam1->SetName("Camera Sensor");
    manager->AddSensor(cam1);

    optix::Context context = manager->GetEngine(0)->GetContext();

    // check camera added correctly
    ASSERT_EQ(context->getEntryPointCount(), 1);

    auto cam2 = chrono_types::make_shared<ChCameraSensor>(
        boxA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam2->SetName("Camera Sensor");
    manager->AddSensor(cam2);

    // check second camera gets added correctly
    ASSERT_EQ(context->getEntryPointCount(), 2);
}

TEST(SensorInterface, lidars) {
    ChSystemNSC mphysicalSystem;

    auto boxA = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, true, false);
    boxA->SetPos({2, 0, 0});
    boxA->SetBodyFixed(true);
    mphysicalSystem.Add(boxA);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);

    auto lidar1 = chrono_types::make_shared<ChLidarSensor>(
        boxA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 6.2, .1, -.1, 100);
    lidar1->SetName("Camera Sensor");
    manager->AddSensor(lidar1);

    optix::Context context = manager->GetEngine(0)->GetContext();

    // check camera added correctly
    ASSERT_EQ(context->getEntryPointCount(), 1);

    auto lidar2 = chrono_types::make_shared<ChLidarSensor>(
        boxA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 6.2, .1, -.1, 100);
    lidar2->SetName("Camera Sensor");
    manager->AddSensor(lidar2);

    // check second camera gets added correctly
    ASSERT_EQ(context->getEntryPointCount(), 2);
}

TEST(SensorInterface, boxes) {
    ChSystemNSC mphysicalSystem;

    auto boxA = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, true, false);
    mphysicalSystem.Add(boxA);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        boxA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    optix::Context context = manager->GetEngine(0)->GetContext();

    // check root has a single object attached
    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 1);

    auto boxB = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, true, false);
    mphysicalSystem.Add(boxB);

    manager->ReconstructScenes();

    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 2);
}

TEST(SensorInterface, spheres) {
    ChSystemNSC mphysicalSystem;

    auto sphereA = chrono_types::make_shared<ChBodyEasySphere>(1, 100, true, false);
    mphysicalSystem.Add(sphereA);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        sphereA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    optix::Context context = manager->GetEngine(0)->GetContext();

    // check root has a single object attached
    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 1);

    auto sphereB = chrono_types::make_shared<ChBodyEasySphere>(1, 100, true, false);
    mphysicalSystem.Add(sphereB);

    manager->ReconstructScenes();

    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 2);
}

TEST(SensorInterface, cylinders) {
    ChSystemNSC mphysicalSystem;

    auto cylA = chrono_types::make_shared<ChBodyEasyCylinder>(1, 1, 100, true, false);
    mphysicalSystem.Add(cylA);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        cylA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    optix::Context context = manager->GetEngine(0)->GetContext();

    // check root has a single object attached
    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 1);

    auto cylB = chrono_types::make_shared<ChBodyEasyCylinder>(1, 1, 100, true, false);
    mphysicalSystem.Add(cylB);

    manager->ReconstructScenes();

    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 2);
}

TEST(SensorInterface, meshes) {
    ChSystemNSC mphysicalSystem;

    auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(GetChronoDataFile("cube.obj"), false, true);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetStatic(true);

    auto meshbodyA = chrono_types::make_shared<ChBody>();
    meshbodyA->AddAsset(trimesh_shape);
    mphysicalSystem.Add(meshbodyA);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        meshbodyA, 50, chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(0, {1, 0, 0})), 1, 1, 1);
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    optix::Context context = manager->GetEngine(0)->GetContext();

    // check root has a single object attached
    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 1);

    auto meshbodyB = chrono_types::make_shared<ChBody>();
    meshbodyB->AddAsset(trimesh_shape);
    mphysicalSystem.Add(meshbodyB);

    manager->ReconstructScenes();

    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 2);
}

TEST(SensorInterface, non_body_shapes) {
    ChSystemNSC mphysicalSystem;

    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(.5, .5, .5, 1000, true, false);
    mphysicalSystem.Add(box_body);

    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.25, 1000, true, false);
    mphysicalSystem.Add(sphere_body);

    auto cylinder_body = chrono_types::make_shared<ChBodyEasyCylinder>(.25, .5, 1000, true, false);
    mphysicalSystem.Add(cylinder_body);

    auto mesh_body = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("sensor/geometries/suzanne.obj"), 1000,
                                                               false, true);
    mphysicalSystem.Add(mesh_body);
    std::dynamic_pointer_cast<ChVisualization>(mesh_body->GetAssetN(0))->SetStatic(true);

    auto box_item = chrono_types::make_shared<ChLoadContainer>();
    auto bshape = chrono_types::make_shared<ChBoxShape>();
    bshape->GetBoxGeometry().SetLengths(ChVector<>(.5, .5, .5));
    box_item->AddAsset(bshape);
    mphysicalSystem.Add(box_item);

    auto sphere_item = chrono_types::make_shared<ChLoadContainer>();
    auto sshape = chrono_types::make_shared<ChSphereShape>();
    sshape->GetSphereGeometry().rad = .25;
    sphere_item->AddAsset(sshape);
    mphysicalSystem.Add(sphere_item);

    auto cylinder_item = chrono_types::make_shared<ChLoadContainer>();
    auto cshape = chrono_types::make_shared<ChCylinderShape>();
    cshape->GetCylinderGeometry().rad = .25;
    cshape->GetCylinderGeometry().p1 = {0, .25, -1};
    cshape->GetCylinderGeometry().p2 = {0, -.25, -1};
    cylinder_item->AddAsset(cshape);
    mphysicalSystem.Add(cylinder_item);

    auto mesh_item = chrono_types::make_shared<ChLoadContainer>();
    auto mshape = std::dynamic_pointer_cast<ChTriangleMeshShape>(mesh_body->GetAssetN(0));
    mshape->SetStatic(true);
    mesh_item->AddAsset(mshape);
    mphysicalSystem.Add(mesh_item);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box_body,                                                           // body camera is attached to
        10,                                                                 // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        640,                                                                // image width
        480,                                                                // image height
        CH_C_PI / 3);                                                       // FOV
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    optix::Context context = manager->GetEngine(0)->GetContext();

    // check root has correct number of objects attached to scene
    ASSERT_EQ(context["root_node"]->getGroup()->getChildCount(), 8);
}
