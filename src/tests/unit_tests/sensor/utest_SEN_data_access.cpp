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
// Authors: Bocheng Zou
// =============================================================================
//
// Unit test for testing accessing RGBA data from an empty scene.
//
// =============================================================================


#include "gtest/gtest.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace sensor;

TEST(ChOptixEngine, data_access) {
    ChSystemNSC sys;

    auto anchor = chrono_types::make_shared<ChBody>();
    anchor->SetFixed(true);
    sys.Add(anchor);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);

    Background bg;
    bg.mode = BackgroundMode::SOLID_COLOR;
    bg.color_zenith  = {1.0f, 0.0f, 1.0f};
    bg.color_horizon = {1.0f, 0.0f, 1.0f}; 
    manager->scene->SetBackground(bg);

    const unsigned int W = 64;
    const unsigned int H = 48;

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        anchor,                                 // parent body
        30.0f,                                  // update rate Hz
        chrono::ChFrame<double>({0, 0, 0}, QUNIT),
        W, H,
        (float)CH_PI / 3,                       // HFOV
        1,                                      // supersample_factor
        CameraLensModelType::PINHOLE,
        false,                                  // use_gi
        2.2f,                                   // gamma
        false                                   // use_fog
    );

    cam->SetLag(0);
    cam->SetCollectionWindow(0);

    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    UserRGBA8BufferPtr rgba;
    const double step = 0.01;

    for (int i = 0; i < 200; ++i) {
        manager->Update();
        sys.DoStepDynamics(step);

        rgba = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba && rgba->Buffer) {
            break;
        }
    }

    ASSERT_TRUE(rgba);
    ASSERT_TRUE(rgba->Buffer);
    ASSERT_EQ(rgba->Width, W);
    ASSERT_EQ(rgba->Height, H);

    const uint8_t expR = 255;
    const uint8_t expG = 0;
    const uint8_t expB = 255;

    size_t mismatch = 0;
    auto* data = rgba->Buffer.get();  // PixelRGBA8[]

    for (unsigned int y = 0; y < rgba->Height; ++y) {
        for (unsigned int x = 0; x < rgba->Width; ++x) {
            const auto& p = data[y * rgba->Width + x];
            if (p.R != expR || p.G != expG || p.B != expB) {
                ++mismatch;
            }
        }
    }

    EXPECT_EQ(mismatch, 0u) << "Found " << mismatch << " pixels not matching background color";
}
