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
// Authors: Asher Elmquist, Yan Xiao
// =============================================================================
//
// Chrono demonstration of a camera sensor.
// Generates a mesh object and rotates camera sensor around the mesh.
//
// =============================================================================

#include "chrono/assets/ChVisualMaterial.h"
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
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Camera lens model
CameraLensModelType lens_model = CameraLensModelType::RADIAL;
// CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

// Update rate in Hz
float update_rate = 30;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = 1.431f;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0.f;

// Exposure (in seconds) of each image
float exposure_time = 0.0f;

int alias_factor = 1;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Simulation end time
float end_time = 4.0f;

// Save camera images
bool save = true;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

    // -----------------------------------------
    // add box with checkerboard pattern to the scene
    // -----------------------------------------

    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(0.001, 10 * .023, 7 * .023, 1000, true, false);
    box_body->SetPos({1.9, 0, 0});
    box_body->SetBodyFixed(true);
    mphysicalSystem.Add(box_body);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetBodyFixed(true);
    mphysicalSystem.Add(floor);

    auto checkerboard = chrono_types::make_shared<ChVisualMaterial>();
    checkerboard->SetKdTexture(GetChronoDataFile("sensor/textures/checkerboard.png"));
    checkerboard->SetRoughness(0.8f);
    box_body->GetVisualModel()->GetShape(0)->SetMaterial(0, checkerboard);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({-10.f, 0.0f, 0.f}, {2.0f / 2, 1.8902f / 2, 1.7568f / 2}, 50.0f);

    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
    chrono::ChFrame<double> offset_pose2({1.5, 0, 0}, QUNIT);
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose2,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         false);        // use global illumination or not
    cam->SetRadialLensParameters({-0.369f, 0.1257f, -0.0194f});
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, ""));
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "camera_rad/"));
    manager->AddSensor(cam);

    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
    auto cam_fov = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                             update_rate,   // update rate in Hz
                                                             offset_pose2,  // offset pose
                                                             image_width,   // image width
                                                             image_height,  // image height
                                                             fov,           // camera's horizontal field of view
                                                             alias_factor,  // supersample factor for antialiasing
                                                             CameraLensModelType::FOV_LENS,  // FOV
                                                             false);  // use global illumination or not
    if (vis)
        cam_fov->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, ""));
    if (save)
        cam_fov->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "camera_fov/"));
    // manager->AddSensor(cam_fov);

    // ---------------
    // Simulate system
    // ---------------
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    double x_min = 1.5;
    double x_max = 2.5;
    double y_min = -0.5;
    double y_max = 0.5;
    double z_min = -0.4;
    double z_max = 0.4;

    double ax_min = -CH_C_PI / 6;
    double ax_max = CH_C_PI / 6;
    double ay_min = -CH_C_PI / 6;
    double ay_max = CH_C_PI / 6;
    double az_min = -CH_C_PI / 6;
    double az_max = CH_C_PI / 6;

    while (ch_time < end_time) {
        // Update sensor manager
        // Will render/save/filter automatically

        // Set object pose randomly to generate calibration images
        box_body->SetPos({x_min + ChRandom() * (x_max - x_min), y_min + ChRandom() * (y_max - y_min),
                          z_min + ChRandom() * (z_max - z_min)});
        box_body->SetRot(
            Q_from_Euler123({ax_min + ChRandom() * (ax_max - ax_min), ay_min + ChRandom() * (ay_max - ay_min),
                             az_min + ChRandom() * (az_max - az_min)}));

        manager->Update();

        // Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)mphysicalSystem.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}