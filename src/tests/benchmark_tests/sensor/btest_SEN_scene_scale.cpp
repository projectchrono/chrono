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

float time_interval = 5.0f;

float x_bound = 10;
float y_bound = 10;
float z_bound = 2;

int start_exp = 17;
int stop_exp = 21;

int obj_type = 0;  // 0=box, 1=sphere, 2=cylinder

bool vis = true;

float randf() {
    return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

int main(int argc, char* argv[]) {
    srand(0);
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    auto manager = std::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 5000);
    manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 5000);
    manager->scene->AddPointLight({100, -100, 100}, {1, 1, 1}, 5000);
    manager->scene->AddPointLight({-100, -100, 100}, {1, 1, 1}, 5000);

    auto cam_body = chrono_types::make_shared<ChBodyEasyBox>(.01, .01, .01, 1000, false, false);
    cam_body->SetBodyFixed(true);
    sys.Add(cam_body);
    auto cam = std::make_shared<ChCameraSensor>(
        cam_body,                                                           // body camera is attached to
        10.0f,                                                              // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1280,                                                               // image width
        720,                                                                // image height
        (float)CH_C_PI / 3                                                  // FOV
    );
    cam->SetName("Camera Sensor");
    if (vis)
        cam->PushFilter(std::make_shared<ChFilterVisualize>(1280, 720));
    manager->AddSensor(cam);

    int curr_item_cnt = 0;

    // ChBox box_asset = ChBox({0, 0, 0}, ChMatrix33<>(1), {1, 1, 1});
    //
    // auto box_shape = chrono_types::make_shared<ChBoxShape>(box_asset);
    // box_shape->SetMutable(false);

    for (int q = start_exp; q <= stop_exp; q++) {
        int target_item_cnt = pow(2, q);
        std::cout << "Box Count: " << target_item_cnt << std::endl;
        while (curr_item_cnt < target_item_cnt) {
            if (obj_type == 2) {
                // cylinder
                auto cyl = std::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y,              //
                                                                randf() + .05, 2 * randf() + .1,  //
                                                                1000,                             //
                                                                true, false);
                cyl->SetBodyFixed(true);
                cyl->SetPos({2 * x_bound * (randf() - .5), 2 * y_bound * (randf() - .5), 2 * z_bound * (randf() - .5)});

                auto vis_mat = std::make_shared<ChVisualMaterial>();
                vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
                vis_mat->SetDiffuseColor({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()});
                vis_mat->SetSpecularColor({.2f, .2f, .2f});
                cyl->GetVisualModel()->GetShapes()[0].first->AddMaterial(vis_mat);

                sys.Add(cyl);
                curr_item_cnt++;
            } else if (obj_type == 1) {
                // sphere
                auto sphere = std::make_shared<ChBodyEasySphere>(randf() + .05, 1000, true, false);
                sphere->SetBodyFixed(true);
                sphere->SetPos(
                    {2 * x_bound * (randf() - .5), 2 * y_bound * (randf() - .5), 2 * z_bound * (randf() - .5)});
                auto vis_mat = std::make_shared<ChVisualMaterial>();
                vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
                vis_mat->SetDiffuseColor({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()});
                vis_mat->SetSpecularColor({.2f, .2f, .2f});
                sphere->GetVisualModel()->GetShapes()[0].first->AddMaterial(vis_mat);

                sys.Add(sphere);
                curr_item_cnt++;
            } else {
                // box
                auto box = std::make_shared<ChBodyEasyBox>(2 * randf() + .1, 2 * randf() + .1, 2 * randf() + .1, 1000,
                                                           true, false);
                box->SetBodyFixed(true);
                box->SetPos({2 * x_bound * (randf() - .5), 2 * y_bound * (randf() - .5), 2 * z_bound * (randf() - .5)});
                auto vis_mat = std::make_shared<ChVisualMaterial>();
                vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
                vis_mat->SetDiffuseColor({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()});
                vis_mat->SetSpecularColor({.2f, .2f, .2f});
                box->GetVisualModel()->GetShapes()[0].first->AddMaterial(vis_mat);

                sys.Add(box);
                curr_item_cnt++;
            }
        }

        manager->ReconstructScenes();
        // ---------------
        // Simulate system
        // ---------------
        float orbit_radius = 2 * sqrt(x_bound * x_bound + y_bound * y_bound);
        float orbit_rate = 0.5;
        float ch_time = 0.0;

        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        float start_time = (float)sys.GetChTime();
        while (ch_time < start_time + time_interval) {
            cam->SetOffsetPose(chrono::ChFrame<double>(
                {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate),
                 .2 * orbit_radius},
                Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1}) * Q_from_AngAxis(.3, {0, 1, 0})));

            manager->Update();
            sys.DoStepDynamics(0.1);

            ch_time = (float)sys.GetChTime();
        }
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "Num instances: " << curr_item_cnt << ", Simulation time: " << time_interval
                  << ", wall time: " << wall_time.count() << ", RTF: " << time_interval / wall_time.count() << "\n";
    }
    return 0;
}
