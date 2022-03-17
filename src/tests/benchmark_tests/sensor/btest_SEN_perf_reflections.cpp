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
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 10.0f;

int main(int argc, char* argv[]) {
    // for (int q = 0; q < 5; q++) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    auto reflective_color = std::make_shared<ChVisualMaterial>();
    reflective_color->SetDiffuseColor({.5f, .5f, .5f});
    reflective_color->SetSpecularColor({.99f, .99f, .99f});
    reflective_color->SetFresnelMin(1);
    reflective_color->SetRoughness(0);

    auto red_color = std::make_shared<ChVisualMaterial>();
    red_color->SetDiffuseColor({1.f, 0.f, 0.f});
    red_color->SetSpecularColor({0.f, 0.f, 0.f});
    red_color->SetFresnelMin(0.0f);
    red_color->SetRoughness(1.0f);

    auto mirror_left = chrono_types::make_shared<ChBodyEasyBox>(50, .1, 5, 1000, true, false);
    mirror_left->SetPos({0, -1, 0});
    mirror_left->SetBodyFixed(true);
    sys.Add(mirror_left);
    mirror_left->GetVisualModel()->GetShapes()[0].first->AddMaterial(reflective_color);

    auto mirror_right = chrono_types::make_shared<ChBodyEasyBox>(50, .1, 5, 1000, true, false);
    mirror_right->SetPos({0, 1, 0});
    mirror_right->SetBodyFixed(true);
    sys.Add(mirror_right);
    mirror_right->GetVisualModel()->GetShapes()[0].first->AddMaterial(reflective_color);

    auto ball = chrono_types::make_shared<ChBodyEasySphere>(.25, 1000, true, false);
    ball->SetPos({4, 0, 0});
    ball->SetBodyFixed(true);
    sys.Add(ball);
    ball->GetVisualModel()->GetShapes()[0].first->AddMaterial(red_color);


    auto cam_body = chrono_types::make_shared<ChBodyEasyBox>(.01, .01, .01, 1000, false, false);
    cam_body->SetBodyFixed(true);
    sys.Add(cam_body);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = std::make_shared<ChSensorManager>(&sys);
    manager->SetVerbose(false);
    manager->SetRayRecursions(21);
    float intensity = 1.f;
    manager->scene->AddPointLight({0, 0, 100}, {intensity, intensity, intensity}, 1000);
    manager->scene->AddPointLight({0, 0, -100}, {intensity, intensity, intensity}, 1000);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto cam = std::make_shared<ChCameraSensor>(
        cam_body,                                                            // body camera is attached to
        50.0f,                                                               // update rate in Hz
        chrono::ChFrame<double>({-10, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1920,                                                                // image width
        1080,                                                                // image height
        (float)CH_C_PI / 1.2f                                                // FOV
    );
    cam->SetName("Camera Sensor");
    cam->PushFilter(std::make_shared<ChFilterVisualize>(1920, 1080));
    // cam->PushFilter(std::make_shared<ChFilterRGBA8Access>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // ---------------
    // Simulate system
    // ---------------
    // float orbit_radius = (2 * x_instances) * x_spread + 0.f;
    // float orbit_rate = 0.5;
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        // cam->SetOffsetPose(chrono::ChFrame<double>(
        //     {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
        //     Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

        manager->Update();
        sys.DoStepDynamics(0.001);

        ch_time = (float)sys.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";
    std::cout << "Frames: " << 100 * ch_time << ", Mean FPS: " << 100 * ch_time / wall_time.count() << "\n";
    return 0;
}
