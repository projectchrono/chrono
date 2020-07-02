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
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace sensor;

#define end_time 1.0

TEST(ChFilterAccess, data_access_safety) {
    ChSystemNSC mphysicalSystem;

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, true);
    box->SetBodyFixed(true);
    mphysicalSystem.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box,                                                                // body camera is attached to
        50,                                                                 // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1280,                                                               // image width
        720,                                                                // image height
        CH_C_PI / 3);                                                       // FOV
    cam->SetName("Camera Sensor");

    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280,720,"RGB Camera"));
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    bool success = true;

    double ch_time = 0;

    // LockedRGBA8BufferPtr camera_data = cam->GetMostRecentBuffer<LockedRGBA8BufferPtr>();

    while (ch_time < end_time) {
        manager->Update();
        mphysicalSystem.DoStepDynamics(0.01);

        ch_time = (float)mphysicalSystem.GetChTime();
        UserRGBA8BufferPtr camera_data = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        // LockedRGBA8BufferPtr camera_data = std::move(cam->GetMostRecentBuffer<LockedRGBA8BufferPtr>());
    }

    ASSERT_EQ(success, true);
}
