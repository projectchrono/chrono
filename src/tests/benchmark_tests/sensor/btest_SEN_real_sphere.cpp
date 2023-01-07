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

float end_time = 100.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    auto phys_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    phys_mat->SetFriction(0.2f);

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------

    ChColor dif = {1, 0, 0};

    auto color1 = std::make_shared<ChVisualMaterial>();
    color1->SetDiffuseColor(dif);
    color1->SetSpecularColor({0.1f, 0.1f, 0.1f});
    color1->SetRoughness(0.5f);
    color1->SetFresnelMin(0.2f);
    color1->SetFresnelMax(1.0f);
    color1->SetOpacity(1.0f);

    auto color2 = std::make_shared<ChVisualMaterial>();
    color2->SetDiffuseColor(dif);
    color2->SetSpecularColor({0.5f, -.5f, 0.5f});
    color2->SetRoughness(0.5f);
    color2->SetFresnelMin(0.2f);
    color2->SetFresnelMax(1.0f);
    color2->SetOpacity(0.2f);

    auto color3 = std::make_shared<ChVisualMaterial>();
    color3->SetDiffuseColor(dif);
    color3->SetSpecularColor({1.0f, 1.0f, 1.0f});
    color3->SetRoughness(0.1f);
    color3->SetFresnelMin(0.5f);
    color3->SetFresnelMax(1.0f);
    color3->SetOpacity(1.0f);

    auto sphere1 = std::make_shared<ChBodyEasySphere>(.5, 1000, true, true, phys_mat);
    sphere1->SetPos({0, -1.2, 0});
    sphere1->SetBodyFixed(true);
    sphere1->GetVisualModel()->GetShapes()[0].first->AddMaterial(color1);

    auto sphere2 = std::make_shared<ChBodyEasySphere>(.5, 1000, true, true, phys_mat);
    sphere2->SetPos({0, 0, 0});
    sphere2->SetBodyFixed(true);
    sphere2->GetVisualModel()->GetShapes()[0].first->AddMaterial(color2);

    auto sphere3 = std::make_shared<ChBodyEasySphere>(.5, 1000, true, true, phys_mat);
    sphere3->SetPos({0, 1.2, 0});
    sphere3->SetBodyFixed(true);
    sphere3->GetVisualModel()->GetShapes()[0].first->AddMaterial(color3);

    

    sys.Add(sphere1);
    sys.Add(sphere2);
    sys.Add(sphere3);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = std::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({-10, 0, 100}, {1, 1, 1}, 1000);
    // manager->scene->AddPointLight({-100, 0, -100}, {1, 1, 1}, 1000);
    // manager->scene->AddPointLight({-10, 0, 100}, {1, 1, 1}, 1000);

    Background b;
    b.mode = BackgroundMode::GRADIENT;
    b.color_horizon = {0.6f, 0.7f, 0.8f};
    b.color_zenith = {0.4f, 0.5f, 0.6f};
    manager->scene->SetBackground(b);
    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto cam = std::make_shared<ChCameraSensor>(
        sphere2,                                                            // body camera is attached to
        20.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-4, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        800,                                                                // image width
        800,                                                                // image height
        (float)CH_C_PI / 4                                                  // FOV
    );
    cam->SetName("Camera Sensor");
    cam->PushFilter(std::make_shared<ChFilterVisualize>(800, 800));
    // cam->PushFilter(std::make_shared<ChFilterRGBA8Access>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // ---------------
    // Simulate system
    // ---------------

    // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (sys.GetChTime() < end_time) {
        // cam->SetOffsetPose(chrono::ChFrame<double>(
        //     {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
        //     Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

        manager->Update();
        sys.DoStepDynamics(0.001);

        // ch_time = (float)sys.GetChTime();
    }
    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
