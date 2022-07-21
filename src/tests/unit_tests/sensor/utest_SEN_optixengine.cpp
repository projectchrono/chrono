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
// Unit test for ChOptixEngine
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"

using namespace chrono;
using namespace sensor;

#define end_time 1.0

// forcing sim to bottleneck each phase (dynamics, scene rebuild, scene trace, post-process)
TEST(ChOptixEngine, render_synchronization) {}

// for making sure we can add sensors safely while the simulation is running
TEST(ChOptixEngine, assign_sensor_safety) {
    ChSystemNSC sys;

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
    box->SetBodyFixed(true);
    sys.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box,                                                                // body camera is attached to
        10.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        64,                                                                 // image width
        48,                                                                 // image height
        (float)CH_C_PI / 3                                                  // FOV
    );
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    bool success = true;

    double ch_time = 0;

    while (ch_time < end_time) {
        manager->Update();
        sys.DoStepDynamics(0.01);

        ch_time = (float)sys.GetChTime();
    }

    auto cam2 = chrono_types::make_shared<ChCameraSensor>(
        box,                                                                // body camera is attached to
        10.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-4, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        64,                                                                 // image width
        48,                                                                 // image height
        (float)CH_C_PI / 3                                                  // FOV
    );
    cam2->SetName("Camera Sensor");
    manager->AddSensor(cam2);

    while (ch_time < 2 * end_time) {
        manager->Update();
        sys.DoStepDynamics(0.01);

        ch_time = (float)sys.GetChTime();
    }

    ASSERT_TRUE(success);
}

// for making sure user interaction with the render engine is safe while the simulation is running
TEST(ChOptixEngine, construct_scene_safety) {
    ChSystemNSC sys;

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
    box->SetBodyFixed(true);
    sys.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box,                                                                // body camera is attached to
        10.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        64,                                                                 // image width
        48,                                                                 // image height
        (float)CH_C_PI / 3                                                  // FOV
    );
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    auto engine = manager->GetEngine(0);
    engine->ConstructScene();

    bool success = true;

    double ch_time = 0;
    int frame = 0;

    while (ch_time < end_time) {
        manager->Update();
        sys.DoStepDynamics(0.01);

        if (frame == 10) {
            auto b = chrono_types::make_shared<ChBodyEasyBox>(.5, .5, .5, 1000, true, false);
            b->SetPos({0, 0, 2});
            b->SetBodyFixed(true);
            sys.Add(b);
            engine->ConstructScene();
        }

        ch_time = (float)sys.GetChTime();
        frame++;
    }

    ASSERT_TRUE(success);
}

TEST(ChOptixEngine, construct_scene_safety_2) {
    ChSystemNSC sys;

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
    box->SetBodyFixed(true);
    sys.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box,                                                                // body camera is attached to
        10.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        64,                                                                 // image width
        48,                                                                 // image height
        (float)CH_C_PI / 3                                                  // FOV
    );
    cam->SetName("Camera Sensor");
    // cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640,480,"RGB Camera"));
    manager->AddSensor(cam);

    auto engine = manager->GetEngine(0);
    engine->ConstructScene();

    bool success = true;

    double ch_time = 0;
    int frame = 0;

    while (ch_time < end_time) {
        manager->Update();
        sys.DoStepDynamics(0.01);

        if (frame == 10) {
            auto b = chrono_types::make_shared<ChBodyEasyBox>(.5, .5, .5, 1000, true, false);
            b->SetPos({0, 0, 2});
            b->SetBodyFixed(true);
            sys.Add(b);
            manager->ReconstructScenes();
        }

        ch_time = (float)sys.GetChTime();
        frame++;
    }

    ASSERT_TRUE(success);
}

TEST(ChOptixEngine, lights) {
    ChSystemNSC sys;

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
    box->SetBodyFixed(true);
    sys.Add(box);

    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        box,                                                                // body camera is attached to
        10.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1,                                                                  // image width
        1,                                                                  // image height
        (float)CH_C_PI / 3                                                  // FOV
    );
    cam->SetName("Camera Sensor");
    manager->AddSensor(cam);

    ASSERT_EQ(manager->scene->GetPointLights().size(), 1);

    while ((float)sys.GetChTime() < end_time) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    for (int i = 0; i < 10; i++) {
        manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);
    }

    while ((float)sys.GetChTime() < 2 * end_time) {
        manager->Update();
        sys.DoStepDynamics(0.01);
    }

    ASSERT_EQ(manager->scene->GetPointLights().size(), 11);
}
