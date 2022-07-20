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

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualization.h"
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

// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL,     // Gaussian noise with constant mean and standard deviation
    PIXEL_DEPENDENT,  // Pixel dependent gaussian noise
    NONE              // No noise model
};
NoiseModel noise_model = PIXEL_DEPENDENT;

// Camera lens model
// Either PINHOLE or SPHERICAL
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

// Update rate in Hz
float update_rate = 30;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = (float)CH_C_PI / 2.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = .05f;

// Exposure (in seconds) of each image
float exposure_time = 0.02f;

int alias_factor = 1;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Simulation end time
float end_time = 20.0f;

// Save camera images
bool save = true;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    GetLog() << "Alias Factor: " << alias_factor << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

    // ---------------------------------------
    // add set of boxes to be visualized by camera
    // ---------------------------------------
    auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    
    
    mmesh->LoadWavefrontMesh(GetChronoDataFile("sensor/geometries/dragon_highres_red.obj"), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Box");
    trimesh_shape->SetStatic(true);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddAsset(trimesh_shape);
    mesh_body->SetBodyFixed(true);
    mphysicalSystem.Add(mesh_body);

    auto vis_mat1 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat1->SetAmbientColor({1.f, 0.f, 0.f});
    vis_mat1->SetSpecularColor({1.f, 1.f, 1.f});
  
    
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    floor->SetPos({0, 0, 0});
    floor->SetBodyFixed(true);
    mphysicalSystem.Add(floor);
    /*
   
    {
        auto asset = floor->GetAssets()[0];
        if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
            visual_asset->material_list.push_back(vis_mat1);
        }
    }*/
    /*
    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat2->SetDiffuseColor({1.0, 1.0, 0.0});
    vis_mat2->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat2->SetUseSpecularWorkflow(true);
    vis_mat2->SetRoughness(0.5f);
    vis_mat2->SetClassID(30000);
    vis_mat2->SetInstanceID(20000);

    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    sphere_body->SetPos({-1, -0.5, 1.5});
    sphere_body->SetBodyFixed(true);
    mphysicalSystem.Add(sphere_body);
    {
        auto asset = sphere_body->GetAssets()[0];
        if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
            visual_asset->material_list.push_back(vis_mat2);
        }
    }
    */

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->SetVerbose(true);
    manager->scene->AddPointLight({0.0f, 0.0f, 3.8f}, {2.0f/2, 1.8902f/2, 1.7568f/2}, 5.0f);

    // -------------------------------------------------------
    // Create a camera and add it to the sensor manager
    // -------------------------------------------------------
    chrono::ChFrame<double> offset_pose2({-.1, .1, 0}, QUNIT);
    // chrono::ChFrame<double> offset_pose2({-3, 0, 0}, QUNIT);
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose2,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         true);         // use global illumination or not
    cam->SetName("Global Illum Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Global Illumination"));
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "globalillum_area_lights/"));
    manager->AddSensor(cam);

    auto cam2 = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose2,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // supersample factor for antialiasing
                                                          lens_model,    // FOV
                                                          false);        // use global illumination or not
    cam2->SetName("Whitted Camera");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Whitted Ray Tracing"));
    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "whitted_area_lights/"));
    manager->AddSensor(cam2);

    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Each camera begins on opposite sides of the object, but rotate at the same speed
    // float orbit_radius = 10.f;
    // float orbit_rate = 2.5;
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        // Update sensor manager
        // Will render/save/filter automatically
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