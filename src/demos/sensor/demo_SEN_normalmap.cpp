
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
// Authors: Asher Elmquist, Han Wang
// =============================================================================
//
// Chrono demonstration of applying texture
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iomanip>

#include "chrono/assets/ChVisualization.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Update rate in Hz
float update_rate = 30.f;

// Image width and height
unsigned int image_width = 720;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = (float) CH_C_PI / 3;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0.f;

// Exposure (in seconds) of each image
float exposure_time = 0.f;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Texture files
// const std::string normalmap = "sensor/textures/brick_normal.png";
// const std::string kdmap = "sensor/textures/brick.png";
const std::string normalmap = "sensor/textures/FaceNormal.jpg";
const std::string kdmap = "sensor/textures/grass_texture.jpg";
// const std::string kdmap = "sensor/textures/rock.png";
// const std::string normalmap = "sensor/textures/rock_normalmap.jpg";
const std::string objfile = "sensor/cube_bumpy.obj";

const std::string floor_kdmap = "sensor/textures/white.png";

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 60.0f;

// Save camera images
bool save = false;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/NORMALMAP_DEMO/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //--------------------
    // Create the system
    //--------------------
    ChSystemNSC mphysicalSystem;

    //----------------------------------------------------------------------------------
    // Create the surface and visual materials with textures to be applied to the bodies
    //----------------------------------------------------------------------------------
    auto phys_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    phys_mat->SetFriction(0.2f);

    // Create a visual material for the floor
    auto vis_mat_floor = std::make_shared<ChVisualMaterial>();
    vis_mat_floor->SetKdTexture(GetChronoDataFile(floor_kdmap));

    // Create a visual material with both normal map and color
    auto vis_mat_both = std::make_shared<ChVisualMaterial>();
    vis_mat_both->SetKdTexture(GetChronoDataFile(kdmap));
    vis_mat_both->SetNormalMapTexture(GetChronoDataFile(normalmap));

    // Create a visual material with only color
    auto vis_mat_color = std::make_shared<ChVisualMaterial>();
    vis_mat_color->SetKdTexture(GetChronoDataFile(kdmap));

    //----------------------------------------
    // Create a floor and add it to the system
    //----------------------------------------
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(100, 100, 1,  // x,y,z size
                                                          1000,         // density
                                                          true,         // collide enable?
                                                          true,         // visualization
                                                          phys_mat);    // physical material
    floor->SetPos({0, 0, -1.0});
    floor->SetBodyFixed(true);
    auto floor_asset = floor->GetAssets()[0];
    if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(floor_asset)) {
        visual_asset->material_list.push_back(vis_mat_floor);
    }
    mphysicalSystem.Add(floor);

    //----------------------------------
    // Create a box without a normal map
    //----------------------------------
    auto box = chrono_types::make_shared<ChBodyEasyBox>(1.8, 1.8, 1.8,  // x,y,z size
                                                        1000,           // density
                                                        true,           // collide enable?
                                                        true,           // visualization
                                                        phys_mat);      // physical material
    box->SetPos({0, -2.5, 0});
    box->SetBodyFixed(true);
    auto box_asset = box->GetAssets()[0];
    if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(box_asset)) {
        visual_asset->material_list.push_back(vis_mat_color);
    }
    mphysicalSystem.Add(box);

    //--------------------------------------------------
    // Create another box with both normal map and color
    //--------------------------------------------------
    auto box2 = chrono_types::make_shared<ChBodyEasyBox>(2.0, 2.0, 2.0,  // x,y,z size
                                                         1000,           // density
                                                         true,           // collide enable?
                                                         true,           // visualization
                                                         phys_mat);      // phyiscal material
    box2->SetPos({0, 2.5, 0});
    box2->SetBodyFixed(true);
    auto box2_asset = box2->GetAssets()[0];
    if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(box2_asset)) {
        visual_asset->material_list.push_back(vis_mat_both);
    }
    mphysicalSystem.Add(box2);

    // --------------------------------------
    // Create a trimesh cube with normal map
    // --------------------------------------
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile(objfile));

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetStatic(true);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddAsset(trimesh_shape);
    mesh_body->SetBodyFixed(true);
    mphysicalSystem.Add(mesh_body);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({-10, 0, 10}, {1, 1, 1}, 3000);

    // Get the point lights
    std::vector<PointLight>& lights = manager->scene->GetPointLights();

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    chrono::ChFrame<double> offset_pose({0, 0, 10}, Q_from_AngY(CH_C_PI / 2));
    auto cam = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose,   // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov            // camera's horizontal field of view
    );
    cam->SetName("Top View Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    if (vis)
        // Visualize the image
        cam->PushFilter(std::make_shared<ChFilterVisualize>(image_width, image_height));

    if (save)
        // Save the current image to a png file at the specified path
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "top_cam/"));

    // add sensor to the manager
    manager->AddSensor(cam);

    // -------------------------------------------------------
    // Create a second camera and add it to the sensor manager
    // -------------------------------------------------------

    chrono::ChFrame<double> offset_pose2({-7, 0, 2}, Q_from_AngAxis(0, {1, 1, 1}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(floor,         // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose2,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          3              // Super sample factor
    );
    cam2->SetName("Side View Camera");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);

    if (vis)
        // Visualize the image
        cam2->PushFilter(std::make_shared<ChFilterVisualize>(image_width, image_height));

    if (save)
        // Save the current image to a png file at the specified path
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "side_cam/"));

    // add sensor to the manager
    manager->AddSensor(cam2);

    //----------------
    // Simulate system
    //----------------
    float orbit_radius = 6.5;
    float orbit_rate = 0.10;
    float ch_time = 0.0;

    while (ch_time < end_time) {
        // Rotate the cameras around the mesh at a fixed rate
        //        cam->SetOffsetPose(chrono::ChFrame<double>(
        //            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 3},
        //            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1}) * Q_from_AngAxis(.5, {0, 1, 0})));
        //
        //        cam2->SetOffsetPose(chrono::ChFrame<double>(
        //            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 3},
        //            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

        mesh_body->SetRot(Q_from_AngX(ch_time * orbit_rate));
        box2->SetRot(Q_from_AngX(ch_time * orbit_rate));

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)mphysicalSystem.GetChTime();
    }
}
