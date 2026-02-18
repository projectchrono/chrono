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
// Authors: Bo-Hsun Chen
// =============================================================================
//
// Chrono demonstration of a camera sensor.
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

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"

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
float update_rate = 5.f;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = (float)CH_PI / 3.;  // [rad]

// Lag (in seconds) between sensing and when data becomes accessible
float lag = .05f;

// Exposure (in seconds) of each image for motion blur effect
float exposure_time = 0.100f;

int alias_factor = 4;

bool use_diffuse =
    false;  // whether cameras consider diffuse reflection. If false, then only considers specular reflection

// Integrator integrator = Integrator::LEGACY;
Integrator integrator = Integrator::PATH;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-2;

// Simulation end time in seconds
float end_time = 2000.0f;

// Save camera images
bool save = false;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/CAM_DEMO/";

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/audi/audi_chassis.obj"),
                                                                  false, true);
    mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("Audi Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({-6, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape, ChFrame<>(ChVector3d(0, 0, 0)));
    mesh_body->SetFixed(true);
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
    floor->SetFixed(true);
    sys.Add(floor);
    {
        auto shape = floor->GetVisualModel()->GetShapeInstances()[0].shape;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat3);
        } else {
            shape->GetMaterials()[0] = vis_mat3;
        }
    }

    // add box object
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat->SetDiffuseColor({0.0, 1.0, 0.0});
    vis_mat->SetSpecularColor({1.f, 1.f, 1.f});
    vis_mat->SetUseSpecularWorkflow(true);
    vis_mat->SetRoughness(.5f);
    vis_mat->SetClassID(30000);
    vis_mat->SetInstanceID(50000);
    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(1.0, 1.0, 1.0, 1000, true, false);
    box_body->SetPos({0, -5, 0});
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

    // add sphere object
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

    // add cylinder object
    auto vis_mat4 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat4->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat4->SetDiffuseColor({0.0, 0.0, 1.0});
    vis_mat4->SetSpecularColor({.0f, .0f, .0f});
    vis_mat4->SetUseSpecularWorkflow(true);
    vis_mat4->SetRoughness(0.5f);
    vis_mat4->SetClassID(30000);
    vis_mat4->SetInstanceID(1000);

    auto cyl_body = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, .25, 1, 1000, true, false);
    cyl_body->SetPos({0, 5, 0});
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
    float intensity = 1.0;
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/quarry_01_4k.hdr");
    manager->scene->SetBackground(b);

    chrono::ChFrame<double> offset_pose({5, 0, 0}, QuatFromAngleAxis(CH_PI, {0, 0, 1}));
    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto cam = chrono_types::make_shared<ChCameraSensor>(ground_body,   // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose,   // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // lens model type
                                                         use_diffuse,   // whether consider diffuse reflection
                                                         use_diffuse,   // whether use OptiX denoiser
                                                         integrator,    // integrator algorithm for rendering
                                                         2.2,           // gamma correction value
                                                         false          // whether use fog
    );

    cam->SetName("Camera Sensor");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    // Render the antialiased image
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Camera Sensor"));

    // Save the antialiased image
    if (save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "camera/"));

    // Provide the host access to the RGBA8 buffer
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    // Add the second camera to the sensor manager
    manager->AddSensor(cam);

    // -------------------------------------------------------
    // Create a physics-based camera and add it to the sensor manager
    // -------------------------------------------------------
    auto phys_cam = chrono_types::make_shared<ChPhysCameraSensor>(
        ground_body,   // body camera is attached to
        update_rate,   // update rate, [Hz]
        offset_pose,   // offset pose
        image_width,   // image width, [px]
        image_height,  // image height, [px]
        lens_model,    // lens model to use
        alias_factor,  // super sampling factor for antialiasing
        use_diffuse,   // whether consider diffuse reflection
        use_diffuse,   // whether use OptiX denoiser
        true,          // whether to activate defocus blur
        true,          // whether to activate vignetting
        true,          // whether to aggregate illumination irradiance
        true,          // whether to add noises
        true,          // whether to convert exposure to digital values
        integrator,    // integrator algorithm for rendering
        2.2f,          // gamma correction, 1.0 for linear color space, 2.2 for sRGB
        false,         // whether to use fog on this camera
        false          // whether to use motion blur effect
    );

    // camera model parameters (Blackfly S BFS-U3-31S4C)
    float phys_cam_px_size = 3.45e-6f;      // [m]
    float max_scene_light_amount = 1000.0;  // [lux = lm/m^2], consider distance-diminishing effect

    // lens parameters (12mm lens)
    float phys_cam_focal_length = 0.012f;                                               // [m]
    float phys_cam_hFOV = fov;                                                          // [rad]
    float phys_cam_sensor_width = 2 * phys_cam_focal_length * tanf(phys_cam_hFOV / 2);  // [m]
    // ChVector<float> phys_cam_distort_params(-0.144202f, 0.273712f, -0.256368f);
    ChVector3f phys_cam_distort_params(-0.16f, 0.2f, -0.12f);  // better result by chessboard

    PhysCameraGainParams phys_cam_gain_params;
    PhysCameraNoiseParams phys_cam_noise_params;

    phys_cam_gain_params.defocus_gain = 10.0f;
    phys_cam_gain_params.defocus_bias = 0.f;

    phys_cam_gain_params.vignetting_gain = 0.6f;
    phys_cam_gain_params.aggregator_gain = 1e8f;
    phys_cam_gain_params.expsr2dv_gains = {1.0f, 1.0f, 1.0f};
    phys_cam_gain_params.expsr2dv_gamma = 0.f;
    phys_cam_gain_params.expsr2dv_crf_type = 2;  // 0: gamma_correct, 1:sigmoid, 2: linear
    ChVector3f phys_cam_rgb_QE_vec(0.4453f, 0.5621f, 0.4713f);
    phys_cam_gain_params.expsr2dv_biases = {0.02f, 0.04f, 0.2f};

    // set noise parameters
    phys_cam_noise_params.FPN_rng_seed = 1234;

    phys_cam_noise_params.dark_currents = {0.000166311f, 0.000341295f, 0.000680946f};

    phys_cam_noise_params.noise_gains = {0.67f * 0.00182512f, 0.67f * 0.00215293f, 0.67f * 0.00318984f};
    phys_cam_noise_params.STD_reads = {0.67f * 2.56849e-05f, 0.67f * 4.08999e-05f, 0.67f * 8.33132e-05f};

    // (apertur_num, expsr_time, ISO, focal_length, focus_dist)
    phys_cam->SetCtrlParameters(4.0f, 0.256f, 100.0f, 0.012f, 10.0f);
    phys_cam->SetModelParameters(phys_cam_sensor_width, phys_cam_px_size, max_scene_light_amount, phys_cam_rgb_QE_vec,
                                 phys_cam_gain_params, phys_cam_noise_params);
    phys_cam->SetRadialLensParameters(phys_cam_distort_params);
    phys_cam->SetName("Phys Camera Sensor");
    phys_cam->SetLag(lag);
    phys_cam->SetCollectionWindow(exposure_time);

    // Render the antialiased image
    if (vis)
        phys_cam->PushFilter(
            chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Physics-based Camera Sensor"));

    // Save the antialiased image
    if (save)
        phys_cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "phys_cam/"));

    // Provide the host access to the RGBDHalf4 buffer
    // phys_cam->PushFilter(chrono_types::make_shared<ChFilterRGBDHalf4Access>());

    // Provide the host access to the RGBA buffer
    // phys_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    phys_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA16Access>());

    // Add the phys camera to the sensor manager
    manager->AddSensor(phys_cam);

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
    float orbit_radius = 10.0f;
    float orbit_rate = 0.02f;
    float orbit_start = 0 * (float)CH_DEG_TO_RAD;
    float ch_time = 0.0f;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    UserRGBA8BufferPtr rgba8_ptr;
    UserRGBA16BufferPtr rgba16_ptr;

    // Print Camera Matrix
    ChMatrix33<float> intrinsic = cam->GetCameraIntrinsicMatrix();
    std::cout << "Camera Instrinsic Matrix: \n"
              << intrinsic(0, 0) << ", " << intrinsic(0, 1) << ", " << intrinsic(0, 2) << ",\n"
              << intrinsic(1, 0) << ", " << intrinsic(1, 1) << ", " << intrinsic(1, 2) << ",\n"
              << intrinsic(2, 0) << ", " << intrinsic(2, 1) << ", " << intrinsic(2, 2) << std::endl;
    // Print Camera distortion coefficients
    ChVector3f dist_coef = cam->GetCameraDistortionCoefficients();
    std::wcout << "Camera Distortion Coefs: \n"
               << "K1: " << dist_coef.x() << ", K2: " << dist_coef.y() << ", K3: " << dist_coef.y() << std::endl;

    while (ch_time < end_time) {
        // Rotate the cameras around the mesh at a fixed rate

        float orbit_angle = ch_time * orbit_rate + orbit_start;

        cam->SetOffsetPose(
            chrono::ChFrame<double>({orbit_radius * cos(orbit_angle), orbit_radius * sin(orbit_angle), 2},
                                    QuatFromAngleAxis(orbit_angle + CH_PI, {0, 0, 1})));

        phys_cam->SetOffsetPose(
            chrono::ChFrame<double>({orbit_radius * cos(orbit_angle), orbit_radius * sin(orbit_angle), 2},
                                    QuatFromAngleAxis(orbit_angle + CH_PI, {0, 0, 1})));

        // Access the RGBA8 buffer from the first camera
        /*
        rgba8_ptr = cam->GetMostRecentBuffer<UserRGBA8BufferPtr>();
        if (rgba8_ptr->Buffer) {
            std::cout << "RGBA8 buffer recieved from cam. Camera resolution: " << rgba8_ptr->Width << "x"
                    << rgba8_ptr->Height << ", frame= " << rgba8_ptr->LaunchedCount << ", t=" <<
                    rgba8_ptr->TimeStamp
                    << std::endl
                    << std::endl;
        }
        */

        // Access the RGBA16 buffer from the physics-based camera
        /*
        rgba16_ptr = phys_cam->GetMostRecentBuffer<UserRGBA16BufferPtr>();
        if (rgba16_ptr->Buffer) {
            int img_w = rgba16_ptr->Width;
            int img_h = rgba16_ptr->Height;
            int buffer_length = img_h * img_w;
            // for (int i = 0; i < buffer_length; i++) {
            PixelRGBA16 pixel = rgba16_ptr->Buffer[0.5 * img_h * img_w + 0.5 * img_w];
            std::cout << "middle PixelRGBA16: [ " << pixel.R << ", " << pixel.G << ", " << pixel.B << ", " << pixel.A
                    << "] " << std::endl;
            pixel = rgba16_ptr->Buffer[0.25 * img_h * img_w + 0.25 * img_w];
            std::cout << "1st PixelRGBA16: [ " << pixel.R << ", " << pixel.G << ", " << pixel.B << ", " << pixel.A
                    << "] " << std::endl;
            pixel = rgba16_ptr->Buffer[0.25 * img_h * img_w + 0.75 * img_w];
            std::cout << "2nd PixelRGBA16: [ " << pixel.R << ", " << pixel.G << ", " << pixel.B << ", " << pixel.A
                    << "] " << std::endl;
            pixel = rgba16_ptr->Buffer[0.75 * img_h * img_w + 0.25 * img_w];
            std::cout << "3rd PixelRGBA16: [ " << pixel.R << ", " << pixel.G << ", " << pixel.B << ", " << pixel.A
                    << "] " << std::endl;
            pixel = rgba16_ptr->Buffer[0.75 * img_h * img_w + 0.75 * img_w];
            std::cout << "4th PixelRGBA16: [ " << pixel.R << ", " << pixel.G << ", " << pixel.B << ", " << pixel.A
                    << "] " << std::endl;
            // }
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
