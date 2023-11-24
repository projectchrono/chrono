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

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/optix/ChOptixEngine.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include "chrono_sensor/optix/ChOptixUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

using namespace chrono;
using namespace chrono::sensor;
using namespace chrono::geometry;

const double ABS_ERR_D = 1e-15;
const float ABS_ERR_F = 1e-6f;

// adding sensors (camera, lidar, radar, gps, acc, gyro, mag)
TEST(SensorInterface, sensors) {
    ChSystemNSC sys;
    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, false, false);
    box->SetBodyFixed(true);
    sys.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    auto camera = chrono_types::make_shared<ChCameraSensor>(box, 100, chrono::ChFrame<double>(), 1, 1, 1);
    camera->SetLag(0.f);
    manager->AddSensor(camera);
    auto lidar = chrono_types::make_shared<ChLidarSensor>(box, 100, chrono::ChFrame<double>(), 1, 1, 1, 1, -1, 100);
    lidar->SetLag(0.f);
    manager->AddSensor(lidar);
//    auto radar = chrono_types::make_shared<ChRadarSensor>(box, 100, chrono::ChFrame<double>(), 1, 1, 1, 1, -1, 100, RadarReturnMode::RadarReturn);
//    radar->SetLag(0.f);
//    manager->AddSensor(radar);
    auto noise = chrono_types::make_shared<ChNoiseNone>();
    auto gps = chrono_types::make_shared<ChGPSSensor>(box, 100, chrono::ChFrame<double>(), ChVector<>(0, 0, 0), noise);
    gps->SetLag(0.f);
    manager->AddSensor(gps);
    auto acc = chrono_types::make_shared<ChAccelerometerSensor>(box, 100, chrono::ChFrame<double>(), noise);
    acc->SetLag(0.f);
    manager->AddSensor(acc);
    auto gyro = chrono_types::make_shared<ChGyroscopeSensor>(box, 100, chrono::ChFrame<double>(), noise);
    gyro->SetLag(0.f);
    manager->AddSensor(gyro);
    auto mag = chrono_types::make_shared<ChMagnetometerSensor>(box, 100, chrono::ChFrame<double>(), noise,
                                                               ChVector<>(0, 0, 0));
    mag->SetLag(0.f);
    manager->AddSensor(mag);

    // check doubly adding sensors
    manager->AddSensor(camera);
    manager->AddSensor(lidar);
//    manager->AddSensor(radar);
    manager->AddSensor(gps);
    manager->AddSensor(acc);
    manager->AddSensor(gyro);
    manager->AddSensor(mag);

    ASSERT_EQ(manager->GetSensorList().size(), 6);  // 7 total sensors

    ASSERT_EQ(manager->GetEngine(0)->GetNumSensor(), 2);  // 3 sensors are optix sensors

    while (sys.GetChTime() < 0.1) {
        manager->Update();
        sys.DoStepDynamics(0.001);
    }

    ASSERT_EQ(camera->GetNumLaunches(), 10);
    ASSERT_EQ(lidar->GetNumLaunches(), 10);
//    ASSERT_EQ(radar->GetNumLaunches(), 10);
    ASSERT_EQ(gps->GetNumLaunches(), 10);
    ASSERT_EQ(acc->GetNumLaunches(), 10);
    ASSERT_EQ(gyro->GetNumLaunches(), 10);
    ASSERT_EQ(mag->GetNumLaunches(), 10);
}

// adding objects (box, sphere, cylinder, mesh)
TEST(SensorInterface, shapes) {
    ChSystemNSC sys;
    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, false, false);
    box->SetBodyFixed(true);
    sys.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    auto lidar = chrono_types::make_shared<ChLidarSensor>(box, 10, chrono::ChFrame<double>(ChVector<>(0, 0, 0), QUNIT),
                                                          1, 1, 0, 0, 0, 100);
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    manager->AddSensor(lidar);

    // nothing there to begin with
    while (sys.GetChTime() < 0.05) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    auto buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
    ASSERT_FLOAT_EQ(buffer->Buffer[0].intensity, 0.f);

    // add box
    auto b = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, true, false);
    b->SetPos({2.5, 0.0, 0.0});
    b->SetBodyFixed(true);
    sys.Add(b);
    manager->ReconstructScenes();
    // nothing there to begin with
    while (sys.GetChTime() < 0.15) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();

    ASSERT_GT(buffer->Buffer[0].intensity, 0.f);
    ASSERT_FLOAT_EQ(buffer->Buffer[0].range, 2.f);

    // remove box, add sphere
    sys.RemoveBody(b);
    auto s = chrono_types::make_shared<ChBodyEasySphere>(0.5, 100, true, false);
    s->SetPos({2.5, 0.0, 0.0});
    s->SetBodyFixed(true);
    sys.Add(s);
    manager->ReconstructScenes();
    // nothing there to begin with
    while (sys.GetChTime() < 0.25) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();

    ASSERT_GT(buffer->Buffer[0].intensity, 0.f);
    ASSERT_FLOAT_EQ(buffer->Buffer[0].range, 2.f);

    // remove sphere, add cylinder
    sys.RemoveBody(s);
    auto c = chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, 0.5, 1.0, 100, true, false);
    c->SetPos({2.5, 0.0, 0.0});
    c->SetBodyFixed(true);
    sys.Add(c);
    manager->ReconstructScenes();
    // nothing there to begin with
    while (sys.GetChTime() < 0.35) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();

    ASSERT_GT(buffer->Buffer[0].intensity, 0.f);
    ASSERT_FLOAT_EQ(buffer->Buffer[0].range, 2.f);
}

// mesh with every valid combo of data channels (verts, normals, uv, vert id, norm id, uv id, mat id)
TEST(SensorInterface, mesh_channels) {
    // triangle data

    std::vector<ChVector<double>> vertices = {{2.f, 0.f, 0.5f}, {2.f, 0.5f, -0.5f}, {2.f, -0.5f, -0.5f}};
    std::vector<ChVector<double>> normals = {{-1.f, 0.f, 0.f}};
    std::vector<ChVector2<double>> uvs = {{0.5f, 1.f}, {0.f, 0.f}, {1.f, 0.f}};
    std::vector<ChVector<int>> vert_ids = {{0, 1, 2}};
    std::vector<ChVector<int>> norm_ids = {{0, 0, 0}};
    std::vector<ChVector<int>> uv_ids = {{0, 1, 2}};
    std::vector<int> mat_ids = {0};

    ChSystemNSC sys;
    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 100, false, false);
    box->SetBodyFixed(true);
    sys.Add(box);

    // triangle with only verts and vert ids
    auto triangle = chrono_types::make_shared<ChTriangleMeshConnected>();
    triangle->getCoordsVertices() = vertices;
    triangle->getIndicesVertexes() = vert_ids;

    auto triangle_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    triangle_shape->SetMesh(triangle);
    triangle_shape->SetMutable(false);

    auto tri_body = chrono_types::make_shared<ChBodyAuxRef>();
    tri_body->SetFrame_REF_to_abs(ChFrame<>());
    tri_body->AddVisualShape(triangle_shape,ChFrame<>());
    tri_body->SetBodyFixed(true);
    sys.Add(tri_body);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    auto lidar = chrono_types::make_shared<ChLidarSensor>(box, 10, chrono::ChFrame<double>(ChVector<>(0, 0, 0), QUNIT),
                                                          1, 1, 0, 0, 0, 100);
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    manager->AddSensor(lidar);

    while (sys.GetChTime() < 0.05) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    auto buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
    ASSERT_GT(buffer->Buffer[0].intensity, 0.f);
    ASSERT_FLOAT_EQ(buffer->Buffer[0].range, 2.f);

    // triangle with verts and uvs
    triangle->getCoordsUV() = uvs;
    triangle->getIndicesUV() = uv_ids;
    manager->ReconstructScenes();
    while (sys.GetChTime() < 0.15) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
    ASSERT_GT(buffer->Buffer[0].intensity, 0.f);
    ASSERT_FLOAT_EQ(buffer->Buffer[0].range, 2.f);

    // triangle with verts, uvs, and normals
    triangle->getCoordsNormals() = normals;
    triangle->getIndicesNormals() = norm_ids;
    manager->ReconstructScenes();
    while (sys.GetChTime() < 0.25) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
    ASSERT_GT(buffer->Buffer[0].intensity, 0.f);
    ASSERT_FLOAT_EQ(buffer->Buffer[0].range, 2.f);

    // triangle with verts, uv, normals, mat
    triangle->getIndicesMaterials() = mat_ids;
    manager->ReconstructScenes();
    while (sys.GetChTime() < 0.35) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    buffer = lidar->GetMostRecentBuffer<UserDIBufferPtr>();
    ASSERT_GT(buffer->Buffer[0].intensity, 0.f);
    ASSERT_FLOAT_EQ(buffer->Buffer[0].range, 2.f);
}
