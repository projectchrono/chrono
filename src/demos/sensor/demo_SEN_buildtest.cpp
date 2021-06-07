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
// Chrono demonstration for testing the chrono sensor module
//
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
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

int num_cameras = 1 - 1;
int num_bodies = 100;
int num_groups = 1;

bool save_data = false;
bool display_data = true;

bool run_chrono = true;
float time_step = 0.002f;
float end_time = 100.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    auto phys_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    phys_mat->SetFriction(0.2f);

    ChSystemNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc({0, 0, -9.81});

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(100, 100, 1,      // x,y,z size
                                                          1000,             // density
                                                          true,             // collide enable?
                                                          true, phys_mat);  // visualization?
    floor->SetPos({0, 0, -1.0});
    floor->SetRot(Q_from_AngZ(CH_C_PI / 2.0));
    floor->SetBodyFixed(true);

    mphysicalSystem.Add(floor);

    // add a mesh
    // auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    // mmesh->LoadWavefrontMesh(GetChronoDataFile("models/bulldozer/shoe_view.obj"), false, true);
    // mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    // mmesh->RepairDuplicateVertexes(1e-9);
    //
    // double mmass;
    // ChVector<> mcog;
    // ChMatrix33<> minertia;
    // double mdensity = 1000;
    // mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
    // ChMatrix33<> principal_inertia_rot;
    // ChVector<> principal_I;
    // ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    std::shared_ptr<ChBody> imu_parent;
    std::shared_ptr<ChBody> gps_parent;

    // float size = 3;
    // auto origin = chrono_types::make_shared<ChBodyEasyBox>(size, size, size, 1000, true, false);
    // origin->SetPos({0, 0, 1});
    // origin->SetBodyFixed(true);
    //
    // mphysicalSystem.Add(origin);
    // auto origin_asset = origin->GetAssets()[0];
    // if (std::shared_ptr<ChVisualization> v_asset = std::dynamic_pointer_cast<ChVisualization>(origin_asset)) {
    //     auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    //     vis_mat->SetDiffuseColor({1, 0, 0});
    //     vis_mat->SetSpecularColor({.5f, .5f, .5f});
    //
    //     v_asset->material_list.push_back(vis_mat);
    // }

    auto wall1 = chrono_types::make_shared<ChBodyEasyBox>(40.0, .1, 10.0, 1000, true, true, phys_mat);
    wall1->SetPos({0, -20, 4});
    wall1->SetBodyFixed(true);
    mphysicalSystem.Add(wall1);

    auto wall2 = chrono_types::make_shared<ChBodyEasyBox>(40.0, .1, 10.0, 1000, true, true, phys_mat);
    wall2->SetPos({0, 20, 4});
    wall2->SetBodyFixed(true);
    mphysicalSystem.Add(wall2);

    auto wall3 = chrono_types::make_shared<ChBodyEasyBox>(.1, 40.0, 10.0, 1000, true, true, phys_mat);
    wall3->SetPos({-20, 0, 4});
    wall3->SetBodyFixed(true);
    mphysicalSystem.Add(wall3);

    auto wall4 = chrono_types::make_shared<ChBodyEasyBox>(.1, 40.0, 10.0, 1000, true, true, phys_mat);
    wall4->SetPos({20, 0, 4});
    wall4->SetBodyFixed(true);
    mphysicalSystem.Add(wall4);

    for (int i = 0; i < num_bodies; i++) {
        // add a box
        auto box =
            chrono_types::make_shared<ChBodyEasyBox>((float)ChRandom() / 2.0 + 0.1, (float)ChRandom() / 2.0 + 0.1,
                                                     (float)ChRandom() / 2.0 + 0.1,  // x,y,z size
                                                     1000,                           // density
                                                     true,                           // collide enable?
                                                     true, phys_mat);                // visualization?
        box->SetPos({(float)ChRandom(), (float)ChRandom(), 2.0 + i});
        box->SetRot(Q_from_Euler123({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()}));
        mphysicalSystem.Add(box);

        if (!imu_parent) {
            imu_parent = box;
        }

        auto cyl = chrono_types::make_shared<ChBodyEasyCylinder>((float)ChRandom() / 2.0 + 0.1,  // radius
                                                                 (float)ChRandom() / 2.0 + 0.1,  // height
                                                                 1000,                           // density
                                                                 true,                           // collide enable?
                                                                 true, phys_mat);                // visualization?
        cyl->SetPos({(float)ChRandom(), (float)ChRandom(), 2.0 + i});
        cyl->SetRot(Q_from_Euler123({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()}));
        mphysicalSystem.Add(cyl);

        auto cyl_asset = cyl->GetAssets()[0];
        if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(cyl_asset)) {
            auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
            // vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
            vis_mat->SetDiffuseColor({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()});
            // vis_mat->SetAmbientColor(.2 * vis_mat->GetDiffuseColor());
            vis_mat->SetSpecularColor({.2f, .2f, .2f});

            visual_asset->material_list.push_back(vis_mat);
        }

        auto sphere = chrono_types::make_shared<ChBodyEasySphere>((float)ChRandom() / 2.0 + 0.1,  // radius
                                                                  1000,                           // density
                                                                  true,                           // collide enable?
                                                                  true, phys_mat);                // visualization?
        sphere->SetPos({(float)ChRandom(), (float)ChRandom(), 2.0 + i});
        // sphere->SetRot(Q_from_Euler123({(float)ChRandom(), (float)ChRandom(), (float)rand() /
        // RAND_MAX}));
        mphysicalSystem.Add(sphere);
        if (!gps_parent) {
            gps_parent = sphere;
        }

        auto sphere_asset = sphere->GetAssets()[0];
        if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(sphere_asset)) {
            auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
            vis_mat->SetAmbientColor({0.f, 0.f, 0.f});
            vis_mat->SetDiffuseColor({(float)ChRandom(), (float)ChRandom(), (float)ChRandom()});
            vis_mat->SetSpecularColor({.2f, .2f, .2f});

            visual_asset->material_list.push_back(vis_mat);
        }
    }

    std::cout << "sensor manager being made\n";
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->SetDeviceList({2, 1, 0});
    manager->SetMaxEngines(num_groups);  // THIS NEEDS MORE PERFORMANCE TESTING

    // make some changes/additions to the scene
    manager->scene->AddPointLight({10, 10, 10}, {1, 1, 1}, 1000);
    manager->scene->AddPointLight({1, 1, 3}, {0, 0, 1}, 1000);
    std::vector<PointLight>& lights = manager->scene->GetPointLights();

    manager->scene->AddPointLight({0, 0, 100}, {1, 1, 1}, 1000);

    auto cam = chrono_types::make_shared<ChCameraSensor>(
        floor,                                                               // body camera is attached to
        25.0f,                                                               // update rate in Hz
        chrono::ChFrame<double>({-10, 0, 1}, Q_from_AngAxis(0, {0, 0, 1})),  // offset pose
        1280,                                                                // image width
        720,                                                                 // image height
        (float)CH_C_PI / 3                                                   // field of view
    );

    std::string color_data_path = "SENSOR_OUTPUT/cam_color/";
    std::string gray_data_path = "SENSOR_OUTPUT/cam_gray/";

    cam->SetName("Camera Sensor 0");
    // cam->SetLag();
    // cam->SetCollectionWindow(imu_collection_time);
    // we want to visualize this sensor right after rendering, so add the visualize filter to the filter list.
    if (display_data)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Before Grayscale Filter"));

    // we want to have access to this RGBA8 buffer on the host.
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    // we want to save the RGBA buffer to png
    if (save_data)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(color_data_path));

    // // filter the sensor to grayscale
    // cam->PushFilter(chrono_types::make_shared<ChFilterGrayscale>());
    //
    // // we want to visualize this sensor after grayscale, so add the visualize filter to the filter list.
    // if (display_data)
    //     cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Final Visualization"));
    //
    // // we want to save the grayscale buffer to png
    // if (save_data)
    //     cam->PushFilter(chrono_types::make_shared<ChFilterSave>(gray_data_path));
    //
    // // we also want to have access to this grayscale buffer on the host.
    // cam->PushFilter(chrono_types::make_shared<ChFilterR8Access>());

    // add sensor to the manager
    manager->AddSensor(cam);

    // add a lidar to the floor facing the falling objects
    auto lidar = chrono_types::make_shared<ChLidarSensor>(
        floor,                                                              // body to which the IMU is attached
        10.0f,                                                              // update rate
        chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose from body
        923,                                                                // horizontal samples
        24,                                                                 // vertical samples/channels
        2.0f * (float)CH_C_PI / 3.0f,                                       // horizontal field of view
        (float)CH_C_PI / 8.0f, -(float)CH_C_PI / 8.0f, 100.0f               // vertical field of view
    );
    lidar->SetName("Lidar Sensor");
    lidar->SetLag(.1f);
    lidar->SetCollectionWindow(.1f);

    if (display_data)
        lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(923, 48, "Raw Lidar Data"));
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    if (display_data)
        lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 2.0f, "Lidar Point Cloud"));
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    // lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    manager->AddSensor(lidar);

    // add an IMU sensor to one of the boxes
    // auto imu_noise_none = chrono_types::make_shared<ChIMUNoiseNone>();
    // auto imu = chrono_types::make_shared<ChIMUSensor>(imu_parent,  // body to which the IMU is attached
    //                                                   10,          // update rate
    //                                                   chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0,
    //                                                   0})), .01, .01, imu_noise_none);  // offset pose from body
    // imu->SetName("IMU");
    // imu->PushFilter(chrono_types::make_shared<ChFilterIMUAccess>());
    // manager->AddSensor(imu);
    //
    // // add an IMU sensor to one of the boxes
    // auto noise_model =
    //     chrono_types::make_shared<ChGPSNoiseNormal>(ChVector<float>(0.f, 0.f, 0.f), ChVector<float>(1.f, 1.f, 1.f));
    // auto gps = chrono_types::make_shared<ChGPSSensor>(
    //     gps_parent,                                                        // body to which the GPS is attached
    //     10,                                                                // update rate
    //     chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose from body
    //     0, 0,
    //     ChVector<double>(43.300, -89.000, 260.0),  // reference GPS location (GPS coordinates of simulation origin)
    //     noise_model                                // noise model to use for adding GPS noise (NOT THREAD SAFE)
    // );
    // gps->SetName("GPS");
    // gps->PushFilter(chrono_types::make_shared<ChFilterGPSAccess>());
    // manager->AddSensor(gps);

    std::vector<std::shared_ptr<ChCameraSensor>> cams;

    for (int i = 0; i < num_cameras; i++) {
        auto cam1 = chrono_types::make_shared<ChCameraSensor>(
            floor,                                                              // body camera is attached to
            10.0f + 10.0f * (i % 4 + 1),                                        // 30 + i, // update rate in Hz
            chrono::ChFrame<double>({-3, 0, 2}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose
            1280,                                                               // image width
            720,                                                                // image height
            (float)CH_C_PI / 3);
        cams.push_back(cam1);

        std::stringstream nm;
        nm << "Camera Sensor " << i + 1;
        cam1->SetName(nm.str());
        cam1->SetLag(.1f);
        cam1->SetCollectionWindow(0);

        // we want to visualize this sensor, so add the visualize filter to the filter list.
        if (display_data)
            cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Before Grayscale Filter"));

        // filter the sensor to grayscale
        cam1->PushFilter(chrono_types::make_shared<ChFilterGrayscale>());

        // we want to visualize this sensor after grayscale, so add the visualize filter to the filter list.
        if (display_data)
            cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "After Grayscale Filter"));

        if (save_data)
            cam1->PushFilter(chrono_types::make_shared<ChFilterSave>("SENSOR_OUTPUT/cam" + std::to_string(i) + "/"));

        // add sensor to the manager
        manager->AddSensor(cam1);
    }

    // ---------------
    // Simulate system
    // ---------------
    float orbit_radius = 15.f;
    float orbit_rate = .2f;
    float ch_time = 0.f;

    double render_time = 0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    float light_change = -.002f;

    std::cout << "Sensor Mangager has: " << manager->GetNumEngines() << " engines running.\n";
    while (ch_time < end_time) {
        std::chrono::high_resolution_clock::time_point r0 = std::chrono::high_resolution_clock::now();
        cam->SetOffsetPose(chrono::ChFrame<double>(
            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 3},
            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));
        lights[0].pos = {-orbit_radius * cos(ch_time * orbit_rate * 2), -orbit_radius * sin(ch_time * orbit_rate * 2),
                         10};

        lights[1].color += make_float3(0, 0, light_change);
        if (lights[1].color.z < 0) {
            lights[1].color = make_float3(0, 0, 0);
            light_change = -light_change;
        }
        if (lights[1].color.z > 1) {
            lights[1].color = make_float3(0, 0, 1);
            light_change = -light_change;
        }

        // origin->SetRot(Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1}));
        // origin->SetPos({0, 0, 3 * sin(ch_time * orbit_rate)});

        manager->Update();
        std::chrono::high_resolution_clock::time_point r1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> t_render = std::chrono::duration_cast<std::chrono::duration<double>>(r1 - r0);
        render_time += t_render.count();

        mphysicalSystem.DoStepDynamics(time_step);

        ch_time = (float)mphysicalSystem.GetChTime();
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";
    std::cout << "Time spent in Chrono: " << wall_time.count() - render_time
              << ", extra time due to rendering: " << render_time << std::endl;

    return 0;
}
