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

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
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
    ChSystemNSC mphysicalSystem;

    int x_dim = 11;
    int y_dim = 7;

    for (int i = 0; i <= x_dim; i++) {
        for (int j = 0; j <= y_dim; j++) {
            auto sphere1 = chrono_types::make_shared<ChBodyEasySphere>(.4, 1000, true, false);
            sphere1->SetPos({0, i - (x_dim / 2.), j - (y_dim / 2.)});
            sphere1->SetBodyFixed(true);
            auto sphere_asset1 = sphere1->GetAssets()[0];
            if (std::shared_ptr<ChVisualization> visual_asset =
                    std::dynamic_pointer_cast<ChVisualization>(sphere_asset1)) {
                auto color = chrono_types::make_shared<ChVisualMaterial>();
                color->SetDiffuseColor({j / (float)y_dim, 0.0, 1 - j / (float)y_dim});
                color->SetSpecularColor({j / (float)y_dim, 0.0, 1 - j / (float)y_dim});
                color->SetFresnelMax(i / (float)x_dim);
                color->SetFresnelMin((float)(.9 * i / x_dim));
                visual_asset->material_list.push_back(color);
            }
            mphysicalSystem.Add(sphere1);
        }
    }

    auto sphere2 = chrono_types::make_shared<ChBodyEasySphere>(.001, 1000, false, false);
    sphere2->SetPos({0, 0, 0});
    sphere2->SetBodyFixed(true);
    mphysicalSystem.Add(sphere2);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({-100, 0, 100}, {1, 1, 1}, 500);
    manager->scene->GetBackground().has_texture = true;
    manager->scene->GetBackground().env_tex = "sensor/textures/cloud_layers_8k.hdr";
    manager->scene->GetBackground().has_changed = true;

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    int alias_factor = 4;
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        sphere2,                                                             // body camera is attached to
        30,                                                                  // update rate in Hz
        chrono::ChFrame<double>({-12, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1280,                                                                // image width
        720,                                                                 // image height
        CH_C_PI / 3);                                                        // FOV
    cam->SetName("Camera Sensor");
    // cam->SetLag(0);
    // cam->SetCollectionWindow(0);

    // --------------------------------------------------------------------
    // Create a filter graph for post-processing the images from the camera
    // --------------------------------------------------------------------

    // we want to visualize this sensor right after rendering, so add the visualize filter to the filter list.
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "For user display"));
    // cam->PushFilter(chrono_types::make_shared<ChFilterSave>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 10.f;
    float orbit_rate = 0.5;
    float ch_time = 0.0;

    double render_time = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        manager->Update();
        mphysicalSystem.DoStepDynamics(0.001);

        ch_time = (float)mphysicalSystem.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
