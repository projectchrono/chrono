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
// Chrono demonstration of a camera sensor.
// Generates a mesh object and rotates camera sensor around the mesh.
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

#include "chrono/assets/ChModelFileShape.h"

#include "chrono_sensor/sensors/ChSegmentationCamera.h"
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
NoiseModel noise_model = NONE;

// Camera lens model
// Either PINHOLE or FOV_LENS
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;

// Update rate in Hz
float update_rate = 30.f;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = (float)CH_C_PI / 3.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = .05f;

// Exposure (in seconds) of each image
float exposure_time = 0.02f;

int alias_factor = 2;

bool use_gi = true;  // whether cameras should use global illumination

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Simulation end time
float end_time = 200.0f;

// Save camera images
bool save = false;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/CAM_DEMO/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                                                                  false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Audi Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({-6, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(ChVector<>(0, 0, 0)));
    mesh_body->SetBodyFixed(true);
    sys.Add(mesh_body);

    auto vis_mat3 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat3->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat3->SetDiffuseColor({.5, .5, .5});
    vis_mat3->SetSpecularColor({.0f, .0f, .0f});
    vis_mat3->SetUseSpecularWorkflow(true);
    vis_mat3->SetClassID(30000);
    vis_mat3->SetInstanceID(30000);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(20, 20, .1, 1000, true, false);
    floor->SetPos({0, 0, -1});
    floor->SetBodyFixed(true);
    sys.Add(floor);
    {
        auto shape = floor->GetVisualModel()->GetShapes()[0].first;
        if(shape->GetNumMaterials() == 0){
            shape->AddMaterial(vis_mat3);
        }
        else{
            shape->GetMaterials()[0] = vis_mat3;
        }
    }

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({0.0, 1.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);

    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.0, 1.0, 1000, true, false);
    box_body->SetPos({0, -2, 0});
    box_body->SetBodyFixed(true);
    sys.Add(box_body);
    {
        auto shape = box_body->GetVisualModel()->GetShapes()[0].first;
        if(shape->GetNumMaterials() == 0){
            shape->AddMaterial(vis_mat);
        }
        else{
            shape->GetMaterials()[0] = vis_mat;
        }
    }

    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat2->SetDiffuseColor({1.0, 0.0, 0.0});
    vis_mat2->SetSpecularColor({.0f, .0f, .0f});
    vis_mat2->SetUseSpecularWorkflow(true);
    vis_mat2->SetRoughness(0.5f);
    vis_mat2->SetClassID(30000);
    vis_mat2->SetInstanceID(20000);

    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    sphere_body->SetPos({0, 0, 0});
    sphere_body->SetBodyFixed(true);
    sys.Add(sphere_body);
    {
        auto shape = sphere_body->GetVisualModel()->GetShapes()[0].first;
        if(shape->GetNumMaterials() == 0){
            shape->AddMaterial(vis_mat2);
        }
        else{
            shape->GetMaterials()[0] = vis_mat2;
        }
    }

    auto vis_mat4 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat4->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat4->SetDiffuseColor({0.0, 0.0, 1.0});
    vis_mat4->SetSpecularColor({.0f, .0f, .0f});
    vis_mat4->SetUseSpecularWorkflow(true);
    vis_mat4->SetRoughness(0.5f);
    vis_mat4->SetClassID(30000);
    vis_mat4->SetInstanceID(1000);

    auto cyl_body = chrono_types::make_shared<ChBodyEasyCylinder>(.25, 1, 1000, true, false);
    cyl_body->SetPos({0, 2, 0});
    cyl_body->SetBodyFixed(true);
    sys.Add(cyl_body);
    {
        auto shape = cyl_body->GetVisualModel()->GetShapes()[0].first;
        if(shape->GetNumMaterials() == 0){
            shape->AddMaterial(vis_mat4);
        }
        else{
            shape->GetMaterials()[0] = vis_mat4;
        }
    }

    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetBodyFixed(true);
    sys.Add(ground_body);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    chrono::ChFrame<double> offset_pose1({-8, 0, 2}, Q_from_AngAxis(.2, {0, 1, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(ground_body,   // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose1,  // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // super sampling factor
                                                         lens_model,    // lens model type
                                                         use_gi, 2.2);
    cam->SetName("Camera Sensor");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    // --------------------------------------------------------------------
    // Create a filter graph for post-processing the images from the camera
    // --------------------------------------------------------------------

    // Add a noise model filter to the camera sensor
    switch (noise_model) {
        case CONST_NORMAL:
            cam->PushFilter(chrono_types::make_shared<ChFilterCameraNoiseConstNormal>(0.f, .0004f));
            break;
        case PIXEL_DEPENDENT:
            cam->PushFilter(chrono_types::make_shared<ChFilterCameraNoisePixDep>(.0004f, .0004f));
            break;
        case NONE:
            // Don't add any noise models
            break;
    }

    // Renders the image at current point in the filter graph
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Global Illumination"));

    // Provides the host access to this RGBA8 buffer
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    if (save)
        // Save the current image to a png file at the specified path
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "rgb/"));

    // Filter the sensor to grayscale
    cam->PushFilter(chrono_types::make_shared<ChFilterGrayscale>());

    // Render the buffer again to see the new grayscaled image
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Final Visualization"));

    // Save the grayscaled image at the specified path
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "gray/"));

    // Resizes the image to the provided width and height
    cam->PushFilter(chrono_types::make_shared<ChFilterImageResize>(image_width / 2, image_height / 2));

    // Access the grayscaled buffer as R8 pixels
    cam->PushFilter(chrono_types::make_shared<ChFilterR8Access>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // -------------------------------------------------------
    // Create a second camera and add it to the sensor manager
    // -------------------------------------------------------

    chrono::ChFrame<double> offset_pose2({5, 0, 0}, Q_from_AngAxis(CH_C_PI, {0, 0, 1}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(ground_body,   // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose2,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          alias_factor,  // supersample factor for antialiasing
                                                          lens_model, false, 2.2);  // FOV
    cam2->SetName("Antialiasing Camera Sensor");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);

    // Render the antialiased image
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Whitted Ray Tracing"));

    // Save the antialiased image
    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "antialiased/"));

    // Provide the host access to the RGBA8 buffer
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    // Add the second camera to the sensor manager
    manager->AddSensor(cam2);

    // -------------------------------------------------------
    // Create a semantic segmentation camera that shadows camera2
    // -------------------------------------------------------
    auto seg = chrono_types::make_shared<ChSegmentationCamera>(ground_body,   // body camera is attached to
                                                               update_rate,   // update rate in Hz
                                                               offset_pose2,  // offset pose
                                                               image_width,   // image width
                                                               image_height,  // image height
                                                               fov,           // camera's horizontal field of view
                                                               lens_model);   // FOV
    seg->SetName("Semantic Segmentation Camera");
    seg->SetLag(lag);
    seg->SetCollectionWindow(exposure_time);

    // Render the semantic mask
    if (vis)
        seg->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Semantic Segmentation"));

    // Save the semantic mask
    if (save)
        seg->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "segmentation/"));

    // Provide the host access to the RGBA8 buffer
    seg->PushFilter(chrono_types::make_shared<ChFilterSemanticAccess>());

    // Add the second camera to the sensor manager
    // manager->AddSensor(seg);

    manager->Update();

    if (std::shared_ptr<ChVisualShape> visual_asset = std::dynamic_pointer_cast<ChVisualShape>(trimesh_shape)) {
        for (const auto& v : visual_asset->GetMaterials()) {
            v->SetClassID(200);
            v->SetInstanceID(200);
        }
    }

    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Each camera begins on opposite sides of the object, but rotate at the same speed
    float orbit_radius = 10.f;
    float orbit_rate = 0.1f;
    float ch_time = 0.0f;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    UserRGBA8BufferPtr rgba8_ptr;
    UserR8BufferPtr r8_ptr;

    while (ch_time < end_time) {
        // Rotate the cameras around the mesh at a fixed rate
        cam->SetOffsetPose(chrono::ChFrame<double>(
            {orbit_radius * cos(ch_time * orbit_rate), orbit_radius * sin(ch_time * orbit_rate), 2},
            Q_from_AngAxis(ch_time * orbit_rate + CH_C_PI, {0, 0, 1})));

        cam2->SetOffsetPose(chrono::ChFrame<double>(
            {orbit_radius * cos(ch_time * orbit_rate), orbit_radius * sin(ch_time * orbit_rate), 2},
            Q_from_AngAxis(ch_time * orbit_rate + CH_C_PI, {0, 0, 1})));

        seg->SetOffsetPose(chrono::ChFrame<double>(
            {orbit_radius * cos(ch_time * orbit_rate), orbit_radius * sin(ch_time * orbit_rate), 2},
            Q_from_AngAxis(ch_time * orbit_rate + CH_C_PI, {0, 0, 1})));

        // Access the RGBA8 buffer from the first camera
        // rgba8_ptr = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        // if (rgba8_ptr->Buffer) {
        //     std::cout << "RGBA8 buffer recieved from cam. Camera resolution: " << rgba8_ptr->Width << "x"
        //               << rgba8_ptr->Height << ", frame= " << rgba8_ptr->LaunchedCount << ", t=" <<
        //               rgba8_ptr->TimeStamp
        //               << std::endl
        //               << std::endl;
        // }
        //
        // // Access the R8 buffer from the first camera
        // r8_ptr = cam->GetMostRecentBuffer<UserR8BufferPtr>();
        // if (r8_ptr->Buffer) {
        //     // Calculate the average gray value in the buffer
        //     unsigned int height = r8_ptr->Height;
        //     unsigned int width = r8_ptr->Width;
        //     uint8_t running_total = 0;
        //     for (int i = 0; i < height; i++) {
        //         for (int j = 0; j < width; j++) {
        //             running_total += uint8_t(r8_ptr->Buffer[i * width + j]);
        //         }
        //     }
        //     std::cout << "Average gray value: " << int(running_total) / double(height * width) << std::endl
        //               << std::endl;
        // }
        //
        // // Access the RGBA8 buffer from the second camera
        // rgba8_ptr = cam2->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        // if (rgba8_ptr->Buffer) {
        //     // Retreive and print the first RGBA pixel
        //     PixelRGBA8 first_pixel = rgba8_ptr->Buffer[0];
        //     std::cout << "First Pixel: [ " << unsigned(first_pixel.R) << ", " << unsigned(first_pixel.G) << ", "
        //               << unsigned(first_pixel.B) << ", " << unsigned(first_pixel.A) << " ]" << std::endl
        //               << std::endl;
        //
        //     // Retreive and print the last RGBA pixel
        //     int buffer_length = rgba8_ptr->Height * rgba8_ptr->Width;
        //     PixelRGBA8 last_pixel = rgba8_ptr->Buffer[buffer_length - 1];
        //     std::cout << "Last Pixel: [ " << unsigned(last_pixel.R) << ", " << unsigned(last_pixel.G) << ", "
        //               << unsigned(last_pixel.B) << ", " << unsigned(last_pixel.A) << " ]" << std::endl
        //               << std::endl;
        // }

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Perform step of dynamics
        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";

    return 0;
}
