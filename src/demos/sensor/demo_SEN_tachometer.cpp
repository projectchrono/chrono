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
// Authors: Han Wang
// =============================================================================
//
// Demonstration of Tachometer Sensor
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_sensor/sensors/ChTachometerSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace chrono::sensor;

int main(int argc, char* argv[]) {
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create the system
    ChSystemNSC mphysicalsystem;
    mphysicalsystem.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // add a ChBody to test
    auto body = chrono_types::make_shared<ChBodyEasyBox>(10, 1, 1, 1000, true, false);
    body->SetPos({0, 0, 0});
    body->SetAngVelLocal({0., 0.2, 0.});
    mphysicalsystem.Add(body);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(20, 20, 1, 1000, true, false);
    floor->SetPos({0, 0, -2});
    mphysicalsystem.Add(floor);

    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalsystem);
    float intensity = 0.3f;
    manager->scene->AddPointLight({100, 100, 100}, {intensity, intensity, intensity}, 500);

    auto cam_offset_pose = chrono::ChFrame<double>({-8, 0, 1}, QuatFromAngleZ(0));
    auto cam1 = chrono_types::make_shared<ChCameraSensor>(floor,            // body camera is attached to
                                                          5.f,              // update rate in Hz
                                                          cam_offset_pose,  // offset pose
                                                          400,              // image width
                                                          400,              // image height
                                                          CH_PI_3,        // camera's horizontal field of view
                                                          1,                // supersample factor for antialiasing
                                                          CameraLensModelType::PINHOLE,
                                                          false);  // FOV
    cam1->SetName("World Camera Sensor");
    cam1->SetLag(0);
    cam1->SetCollectionWindow(0.02f);
    cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(400, 400, "World Ray Tracing"));
    manager->AddSensor(cam1);

    auto tachometer = chrono_types::make_shared<ChTachometerSensor>(body, 5.f, cam_offset_pose, Y);
    tachometer->SetName("Tachometer");
    tachometer->SetLag(0);
    tachometer->SetCollectionWindow(0.02f);
    tachometer->PushFilter(chrono_types::make_shared<ChFilterTachometerAccess>());
    manager->AddSensor(tachometer);

    float ch_time = 0.0;
    float end_time = 100.0;

    UserTachometerBufferPtr data_ptr;

    while (ch_time < end_time) {
        data_ptr = tachometer->GetMostRecentBuffer<UserTachometerBufferPtr>();
        if (data_ptr->Buffer) {
            std::cout << data_ptr->Buffer[0].rpm << std::endl;
        }
        manager->Update();
        mphysicalsystem.DoStepDynamics(1e-3);
        ch_time = (float)mphysicalsystem.GetChTime();
    }
    std::cout << "simulation complete" << std::endl;
}