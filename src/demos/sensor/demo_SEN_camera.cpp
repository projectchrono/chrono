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
// Chrono demonstration of a camera sensor with different types of lights.
// Generates a mesh object and rotates camera sensor around the mesh.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualShapeModelFile.h"

#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

using namespace chrono;
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
float fov = (float)CH_PI_3;  // [rad]

// Lag (in seconds) between sensing and when data becomes accessible
float lag = .05f;

// Exposure (in seconds) of each image for generating motion blur effect
float exposure_time = 0.02f;

int alias_factor = 2;

bool use_diffuse_1 = false;   // whether Camera 1 consinders diffuse reflection
bool use_diffuse_2 = false;   // whether Camera 2 consinders diffuse reflection
bool use_denoiser_1 = false;  // whether Camera 1 uses the OptiX denoiser
bool use_denoiser_2 = false;  // whether Camera 2 uses the OptiX denoiser

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;  // [sec]

// Simulation end time
float end_time = 200.0f;  // [sec]

// Save camera images
bool save = false;

// Render camera images
bool vis = true;

// Verbose terminal output
bool verbose = false;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/CAM_DEMO/";

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Address arguments
    if (argc < 3) {
        std::cout << "Please assign the number of samples per pixel (spp) and light type to illuminate the scene."
                  << std::endl;
        std::cout << "And optionally, you can decide whether Cameras 1 and 2 consider diffuse reflection and attach "
                     "the OptiX denoisers,";
        std::cout << "following the command line as:" << std::endl;
        std::cout
            << "demo_SEN_camera <number of samples per pixel> <light type: point, spot, directional, environment> ";
        std::cout << "--use_diffuse_1 --use_diffuse_2 --use_denoiser_1 --use_denoiser_2" << std::endl;
        exit(1);
    }

    alias_factor = (int)sqrt(std::atof(argv[1]));
    std::string light_type_str = argv[2];

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--use_diffuse_1") == 0) {
            use_diffuse_1 = true;
            std::cout << "\nPath Camera considers diffuse reflection\n";
        } else if (std::strcmp(argv[i], "--use_diffuse_2") == 0) {
            use_diffuse_2 = true;
            std::cout << "\nLegacy Camera considers diffuse reflection\n";
        } else if (std::strcmp(argv[i], "--use_denoiser_1") == 0) {
            use_denoiser_1 = true;
            std::cout << "\nPath Camera uses OptiX denoiser\n";
        } else if (std::strcmp(argv[i], "--use_denoiser_2") == 0) {
            use_denoiser_2 = true;
            std::cout << "\nLegacy Camera uses OptiX denoiser\n";
        }
    }

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;
    sys.SetGravityY();

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                                                                  true, true);
    mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Audi Chassis Mesh");

    // Create a dielectric and diffuse visual material
    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({-6, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(ChVector3d(0, 0, 0)));
    mesh_body->GetVisualShape(0)->GetMaterial(0)->SetRoughness(1.f);
    mesh_body->GetVisualShape(0)->GetMaterial(0)->SetMetallic(0.f);
    mesh_body->SetFixed(true);
    sys.Add(mesh_body);

    // Create a dielectric and diffuse visual material
    auto vis_mat3 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat3->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat3->SetDiffuseColor({.5, .5, .5});
    vis_mat3->SetSpecularColor({.0f, .0f, .0f});
    vis_mat3->SetOpacity(1.f);
    vis_mat3->SetRoughness(1.f);
    vis_mat3->SetMetallic(0.f);
    vis_mat3->SetUseSpecularWorkflow(true);
    vis_mat3->SetClassID(30000);
    vis_mat3->SetInstanceID(30000);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(20, 20, .1, 1000, true, false);
    floor->SetPos({0, 0, -0.6});
    floor->SetFixed(true);
    {
        auto shape = floor->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat3);
        } else {
            shape->GetMaterials()[0] = vis_mat3;
        }
    }
    sys.Add(floor);

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({0.0, 1.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetOpacity(1.f);
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);

    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.0, 1.0, 1000, true, false);
    box_body->SetPos({0, -2, 0});
    box_body->SetFixed(true);
    sys.Add(box_body);
    {
        auto shape = box_body->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat);
        } else {
            shape->GetMaterials()[0] = vis_mat;
        }
    }

    // Create a metallic and specular visual material
    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetBSDF(BSDFType::SPECULAR);
    vis_mat2->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat2->SetDiffuseColor({1.0, 1.0, 1.0});
    vis_mat2->SetSpecularColor({1.0f, 1.0f, 1.0f});
    vis_mat2->SetOpacity(1.f);
    vis_mat2->SetUseSpecularWorkflow(true);
    vis_mat2->SetRoughness(0.f);
    vis_mat2->SetMetallic(1.f);
    vis_mat2->SetClassID(30000);
    vis_mat2->SetInstanceID(20000);

    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    sphere_body->SetPos({0, 0, 0});
    sphere_body->SetFixed(true);
    sys.Add(sphere_body);
    {
        auto shape = sphere_body->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat2);
        } else {
            shape->GetMaterials()[0] = vis_mat2;
        }
    }

    auto vis_mat4 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat4->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat4->SetDiffuseColor({0.0, 0.0, 1.0});
    vis_mat4->SetSpecularColor({.0f, .0f, .0f});
    vis_mat4->SetOpacity(1.f);
    vis_mat4->SetUseSpecularWorkflow(true);
    vis_mat4->SetRoughness(0.5f);
    vis_mat4->SetClassID(30000);
    vis_mat4->SetInstanceID(1000);

    auto cyl_body = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, .25, 1, 1000, true, false);
    cyl_body->SetPos({0, 2, 0});
    cyl_body->SetFixed(true);
    sys.Add(cyl_body);
    {
        auto shape = cyl_body->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat4);
        } else {
            shape->GetMaterials()[0] = vis_mat4;
        }
    }

    auto ground_body = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    ground_body->SetPos({0, 0, 0});
    ground_body->SetFixed(true);
    sys.Add(ground_body);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetRayRecursions(6);
    manager->SetVerbose(verbose);
    manager->scene->SetAmbientLight({0.f, 0.f, 0.f});
    Background b;

    float intensity = 1.0f;
    unsigned int light_idx = 0;
    if (light_type_str.find("point") != std::string::npos) {
        // (pos, color, max_range, const_color)
        light_idx = manager->scene->AddPointLight({0, 0, 3}, {intensity, intensity, intensity}, 500, true);
    } else if (light_type_str.find("directional") != std::string::npos) {
        float light_elevation = 15;  // [deg]
        float light_azimuth = 45;    // [deg]
        // (color, elevation, azimuth)
        light_idx = manager->scene->AddDirectionalLight({intensity, intensity, intensity},
                                                        light_elevation * (float)CH_DEG_TO_RAD,
                                                        light_azimuth * (float)CH_DEG_TO_RAD);
    } else if (light_type_str.find("spot") != std::string::npos) {
        intensity = 6.0f;
        // (pos, color, max_range, light_dir, angle_falloff_start, angle_range, const_color)
        light_idx =
            manager->scene->AddSpotLight({-5.f, 5.f, 5.f}, {intensity, intensity, intensity}, 8.67f, {1.f, -1.f, -1.f},
                                         60 * (float)CH_DEG_TO_RAD, 90 * (float)CH_DEG_TO_RAD, false);
    } else if (light_type_str.find("environment") != std::string::npos) {
        b.mode = BackgroundMode::ENVIRONMENT_MAP;
        float env_light_scale = 0.f;

        b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
        // b.env_tex = GetChronoDataFile("sensor/textures/dreifaltigkeitsberg_2k.hdr");
        // b.env_tex = GetChronoDataFile("sensor/textures/UVChecker_byValle_4K.png");
        env_light_scale = 1.f;

        // b.env_tex = GetChronoDataFile("sensor/textures/envmap_sun_at_270_045.hdr");
        // env_light_scale = 1000.f;

        manager->scene->SetBackground(b);
        manager->Update();
        light_idx = manager->scene->AddEnvironmentLight(b.env_tex, env_light_scale);
    } else {
        std::cout << "\nUnsupported type of light in this scene ...\n" << std::endl;
        exit(1);
    }

    // Set up the background
    if (light_type_str.find("environment") == std::string::npos) {
        b.mode = BackgroundMode::SOLID_COLOR;
        b.color_zenith = ChVector3f(0.f, 0.f, 0.f);
        manager->scene->SetBackground(b);
    }

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    chrono::ChFrame<double> offset_pose1({-8, 0, 2}, QuatFromAngleAxis(.2, {0, 1, 0}));
    auto cam1 = chrono_types::make_shared<ChCameraSensor>(
        ground_body,       // body camera is attached to
        update_rate,       // update rate in Hz
        offset_pose1,      // offset pose
        image_width,       // image width
        image_height,      // image height
        fov,               // camera's horizontal field of view
        alias_factor,      // super sampling factor
        lens_model,        // lens model type
        use_diffuse_1,     // whether consider diffuse reflection. If false, then only considers specular reflection
        use_denoiser_1,    // whether use OptiX denoiser for diffuse reflection or area lights
        Integrator::PATH,  // integrator algorithm for rendering
        2.2);              // gamma correction
    cam1->SetName("Camera Sensor");
    cam1->SetLag(lag);
    cam1->SetCollectionWindow(exposure_time);

    // --------------------------------------------------------------------
    // Create a filter graph for post-processing the images from the camera
    // --------------------------------------------------------------------

    // Add a noise model filter to the camera sensor
    switch (noise_model) {
        case CONST_NORMAL:
            cam1->PushFilter(chrono_types::make_shared<ChFilterCameraNoiseConstNormal>(0.f, .0004f));
            break;
        case PIXEL_DEPENDENT:
            cam1->PushFilter(chrono_types::make_shared<ChFilterCameraNoisePixDep>(.0004f, .0004f));
            break;
        case NONE:
            // Don't add any noise models
            break;
    }

    // Renders the image at current point in the filter graph
    if (vis)
        cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
            image_width, image_height, (use_denoiser_1) ? "Path Integrator Denoised" : "Path Integrator"));

    // Provides the host access to this RGBA8 buffer
    cam1->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    if (save)
        // Save the current image to a png file at the specified path
        cam1->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "rgb/"));

    // Filter the sensor to grayscale
    cam1->PushFilter(chrono_types::make_shared<ChFilterGrayscale>());

    // Render the buffer again to see the new grayscaled image
    if (vis)
        cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Final Visualization"));

    // Save the grayscaled image at the specified path
    if (save)
        cam1->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "gray/"));

    // Resizes the image to the provided width and height
    cam1->PushFilter(chrono_types::make_shared<ChFilterImageResize>(image_width / 2, image_height / 2));

    // Access the grayscaled buffer as R8 pixels
    cam1->PushFilter(chrono_types::make_shared<ChFilterR8Access>());

    // add sensor to the manager
    manager->AddSensor(cam1);

    // -------------------------------------------------------
    // Create a second camera and add it to the sensor manager
    // -------------------------------------------------------

    chrono::ChFrame<double> offset_pose2({5, 0, 0}, QuatFromAngleAxis(CH_PI, {0, 0, 1}));
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(
        ground_body,         // body camera is attached to
        update_rate,         // update rate in Hz
        offset_pose2,        // offset pose
        image_width,         // image width
        image_height,        // image height
        fov,                 // camera's horizontal field of view
        alias_factor,        // supersample factor for antialiasing
        lens_model,          // lens model type
        use_diffuse_2,       // whether consider diffuse reflection. If false, then only considers specular reflection
        use_denoiser_2,      // whether use OptiX denoiser for diffuse reflection or area lights
        Integrator::LEGACY,  // integrator algorithm for rendering
        2.2);                // gamma correction
    cam2->SetName("Antialiasing Camera Sensor");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);

    // Render the antialiased image
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(
            image_width, image_height, (use_denoiser_2) ? "Legacy Integrator Denoised" : "Legacy Integrator"));

    // Save the antialiased image
    if (save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "antialiased/"));

    // Provide the host access to the RGBA8 buffer
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    // Add the second camera to the sensor manager
    manager->AddSensor(cam2);

    // -------------------------------------------------------
    // Create a depth camera that shadows camera2
    // -------------------------------------------------------
    auto depth = chrono_types::make_shared<ChDepthCamera>(ground_body,   // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          offset_pose2,  // offset pose
                                                          image_width,   // image width
                                                          image_height,  // image height
                                                          fov,           // camera's horizontal field of view
                                                          1000,          // maximum depth
                                                          lens_model);   // FOV
    depth->SetName("Depth Camera");
    depth->SetLag(lag);
    depth->SetCollectionWindow(exposure_time);

    // Render the depth map
    if (vis)
        depth->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Depth Camera"));

    // Set max depth of the depth camera
    depth->SetMaxDepth(30.f);  // meters

    // Note: With Depth camera, an access filter is already added to the filter graph internally. DO NOT add another.

    // Add the depth camera to the sensor manager
    manager->AddSensor(depth);

    // -------------------------------------------------------
    // Create a normal camera that shadows camera2
    // -------------------------------------------------------

    auto normal = chrono_types::make_shared<ChNormalCamera>(ground_body,   // body camera is attached to
                                                            update_rate,   // update rate in Hz
                                                            offset_pose2,  // offset pose
                                                            image_width,   // image width
                                                            image_height,  // image height
                                                            fov,           // camera's horizontal field of view
                                                            lens_model     // FOV
    );
    normal->SetName("Normal Camera");
    normal->SetLag(lag);
    normal->SetCollectionWindow(exposure_time);

    // Render the normal map
    if (vis)
        normal->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 360, "Normal Camera"));

    // Save the normal map
    if (save)
        normal->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "normal/"));

    // Note: Within Normal Camera, an access filter is already added to the filter graph internally. DO NOT add another.

    // Add the depth camera to the sensor manager
    manager->AddSensor(normal);

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
    manager->AddSensor(seg);

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
    float orbit_radius = 13.f;                // [m]
    float orbit_rate = 0.1f;                  // [rad/sec]
    float orbit_start = 0.f * CH_PI / 180.f;  // [rad]
    float ch_time = 0.0f;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    UserRGBA8BufferPtr rgba8_ptr;
    UserR8BufferPtr r8_ptr;
    UserDepthBufferPtr depth_ptr;

    while (ch_time < end_time) {
        // Rotate the cameras around the mesh at a fixed rate
        float orbit_angle = ch_time * orbit_rate + orbit_start;
        cam1->SetOffsetPose(
            chrono::ChFrame<double>({orbit_radius * cos(orbit_angle), orbit_radius * sin(orbit_angle), 2},
                                    QuatFromAngleAxis(orbit_angle + CH_PI, {0, 0, 1})));

        cam2->SetOffsetPose(
            chrono::ChFrame<double>({orbit_radius * cos(orbit_angle), orbit_radius * sin(orbit_angle), 2},
                                    QuatFromAngleAxis(orbit_angle + CH_PI, {0, 0, 1})));

        seg->SetOffsetPose(
            chrono::ChFrame<double>({orbit_radius * cos(orbit_angle), orbit_radius * sin(orbit_angle), 2},
                                    QuatFromAngleAxis(orbit_angle + CH_PI, {0, 0, 1})));

        depth->SetOffsetPose(
            chrono::ChFrame<double>({orbit_radius * cos(orbit_angle), orbit_radius * sin(orbit_angle), 2},
                                    QuatFromAngleAxis(orbit_angle + CH_PI, {0, 0, 1})));

        normal->SetOffsetPose(
            chrono::ChFrame<double>({orbit_radius * cos(orbit_angle), orbit_radius * sin(orbit_angle), 2},
                                    QuatFromAngleAxis(orbit_angle + CH_PI, {0, 0, 1})));

        // Access the depth buffer from depth camera
        depth_ptr = depth->GetMostRecentBuffer<UserDepthBufferPtr>();
        if (verbose && depth_ptr->Buffer) {
            // Print max depth values
            // float min_depth = depth_ptr->Buffer[0].depth;
            // float max_depth = depth_ptr->Buffer[0].depth;
            // for (int i = 0; i < depth_ptr->Height * depth_ptr->Width; i++) {
            //     max_depth = std::max(max_depth, depth_ptr->Buffer[i].depth);
            // }
            float d = depth_ptr->Buffer[depth_ptr->Height * depth_ptr->Width / 2].depth;
            std::cout << "Depth buffer recieved from depth camera. Camera resolution: " << depth_ptr->Width << "x"
                      << depth_ptr->Height << ", frame= " << depth_ptr->LaunchedCount << ", t=" << depth_ptr->TimeStamp
                      << ", depth [" << depth_ptr->Height * depth_ptr->Width / 2 << "] =" << d << "m" << std::endl;
        }

        // Access the RGBA8 buffer from the first camera
        /*
        rgba8_ptr = cam1->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba8_ptr->Buffer) {
            std::cout << "RGBA8 buffer recieved from cam1. Camera resolution: " << rgba8_ptr->Width << "x"
                      << rgba8_ptr->Height << ", frame= " << rgba8_ptr->LaunchedCount << ", t=" <<
                      rgba8_ptr->TimeStamp
                      << std::endl
                      << std::endl;
        }

        // Access the R8 buffer from the first camera
        r8_ptr = cam1->GetMostRecentBuffer<UserR8BufferPtr>();
        if (r8_ptr->Buffer) {
            // Calculate the average gray value in the buffer
            unsigned int height = r8_ptr->Height;
            unsigned int width = r8_ptr->Width;
            uint8_t running_total = 0;
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    running_total += uint8_t(r8_ptr->Buffer[i * width + j]);
                }
            }
            std::cout << "Average gray value: " << int(running_total) / double(height * width) << std::endl
                      << std::endl;
        }
        */

        // Access the RGBA8 buffer from the second camera
        /*
        rgba8_ptr = cam2->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba8_ptr->Buffer) {
            // Retreive and print the first RGBA pixel
            PixelRGBA8 first_pixel = rgba8_ptr->Buffer[0];
            std::cout << "First Pixel: [ " << unsigned(first_pixel.R) << ", " << unsigned(first_pixel.G) << ", "
                      << unsigned(first_pixel.B) << ", " << unsigned(first_pixel.A) << " ]\n"
                      << std::endl;

            // Retreive and print the last RGBA pixel
            int buffer_length = rgba8_ptr->Height * rgba8_ptr->Width;
            PixelRGBA8 last_pixel = rgba8_ptr->Buffer[buffer_length - 1];
            std::cout << "Last Pixel: [ " << unsigned(last_pixel.R) << ", " << unsigned(last_pixel.G) << ", "
                      << unsigned(last_pixel.B) << ", " << unsigned(last_pixel.A) << " ]\n"
                      << std::endl;
        }
        */

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
