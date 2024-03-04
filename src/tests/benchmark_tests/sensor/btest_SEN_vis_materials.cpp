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
// Chrono demonstration of a creating visual materials
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iomanip>

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
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 100.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    int x_dim = 11;
    int y_dim = 7;

    for (int i = 0; i <= x_dim; i++) {
        for (int j = 0; j <= y_dim; j++) {
            auto sphere1 = chrono_types::make_shared<ChBodyEasySphere>(.4, 1000, true, false);
            sphere1->SetPos({0, i - (x_dim / 2.), j - (y_dim / 2.)});
            sphere1->SetBodyFixed(true);
            
            auto color = chrono_types::make_shared<ChVisualMaterial>();
            color->SetDiffuseColor({.8f, 0.f, 0.f});
            color->SetSpecularColor({(float)i / x_dim, (float)i / x_dim, (float)i / x_dim});
            color->SetMetallic((float)i / x_dim);
            color->SetRoughness(1 - (float)j / y_dim);
            color->SetUseSpecularWorkflow(false);

            sphere1->GetVisualModel()->GetShapes()[0].first->AddMaterial(color);

            sys.Add(sphere1);
        }
    }

    auto sphere2 = chrono_types::make_shared<ChBodyEasySphere>(.001, 1000, false, false);
    sphere2->SetPos({0, 0, 0});
    sphere2->SetBodyFixed(true);
    sys.Add(sphere2);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({-100, 0, 100}, {1, 1, 1}, 500);
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;  // GRADIENT
    b.color_zenith = {.5f, .6f, .7f};
    b.color_horizon = {.9f, .8f, .7f};
    b.env_tex = GetChronoDataFile("sensor/textures/sky_2_4k.hdr");
    manager->scene->SetBackground(b);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        sphere2,                                                             // body camera is attached to
        30.0f,                                                               // update rate in Hz
        chrono::ChFrame<double>({-12, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1920,                                                                // image width
        1080,                                                                // image height
        (float)CH_C_PI / 3                                                   // FOV
    );
    cam->SetName("Camera Sensor");
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "For user display"));
    manager->AddSensor(cam);

    auto cam_g = chrono_types::make_shared<ChCameraSensor>(
        sphere2,                                                             // body camera is attached to
        30.0f,                                                               // update rate in Hz
        chrono::ChFrame<double>({-12, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1920,                                                                // image width
        1080,                                                                // image height
        (float)CH_C_PI / 3, 1, CameraLensModelType::PINHOLE, true            // FOV
    );
    cam_g->SetName("Camera Sensor");
    cam_g->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "For user display, GI"));
    manager->AddSensor(cam_g);

    // ---------------
    // Simulate system
    // ---------------
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        manager->Update();
        sys.DoStepDynamics(0.001);

        ch_time = (float)sys.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
