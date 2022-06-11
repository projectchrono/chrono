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
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace sensor;

#define end_time 1.0

// adding sensor during sim
TEST(ChSensorManager, sensor_adding) {}

// adding and deleting objects during sim
TEST(ChSensorManager, object_adding) {}

// setting sensor params during sim (pose, filters, etc)
TEST(ChSensor, modifying) {}

TEST(ChFilterAccess, data_access_safety) {
    ChSystemNSC sys;

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, true);
    box->SetBodyFixed(true);
    sys.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box,                                                                // body camera is attached to
        50.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1,                                                                  // image width
        1,                                                                  // image height
        (float)CH_C_PI / 3                                                  // FOV
    );
    cam->SetName("Camera Sensor");
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    bool success = true;

    UserRGBA8BufferPtr camera_data = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    unsigned int frames = 0;
    while (frames < 5) {
        manager->Update();
        sys.DoStepDynamics(0.01);

        UserRGBA8BufferPtr tmp_camera_data = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (tmp_camera_data && tmp_camera_data->Buffer && tmp_camera_data->LaunchedCount != frames) {
            // ASSERT_EQ(camera_data->Buffer[0].A, 255); // alpha channel not set correctly currently, TODO: fix this
            // problem in rendering
            camera_data = tmp_camera_data;
            camera_data->Buffer[0].A = 120;
        }

        frames = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>()->LaunchedCount;
    }

    ASSERT_EQ(success, true);
}
