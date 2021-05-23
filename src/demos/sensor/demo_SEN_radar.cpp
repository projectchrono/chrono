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
// Authors: Han Wang 
// =============================================================================
//
// Chrono demonstration of a radar sensor
//
// =============================================================================

#include <cstdio>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_sensor/Sensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/ChRadarSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterRadarProcess.h"
#include "chrono_sensor/filters/ChFilterRadarSavePC.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"


#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"


#include "chrono_irrlicht/ChIrrApp.h"
#include <chrono>


using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;
using namespace chrono::irrlicht;
// using namespace irr;
// using namespace irr::core;
// using namespace irr::scene;
// using namespace irr::video;


// ------------------------------------
// Radar Parameters
// ------------------------------------

// Update rate in Hz
float update_rate = 5.f;

// horizontal field of view of camera
float fov = (float)CH_C_PI / 3.;
int alias_factor = 1;
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Exposure (in seconds) of each image
float exposure_time = 0.02f;


// Number of horizontal and vertical samples
unsigned int horizontal_samples = 500;
unsigned int vertical_samples = 500;

// Field of View
float horizontal_fov = CH_C_PI / 3; // 60 degree scan
float max_vert_angle = (float) CH_C_PI / 10; // 60 degrees up
float min_vert_angle = (float) -CH_C_PI / 10; // 15 degrees down

// max detection range
float max_distance = 100;

// lag time
float lag = 0.f;

// Collection window for the radar
float collection_time = 1 / update_rate; //typically 1/update rate

// Output directories
const std::string out_dir = "RADAR_OUTPUT/";
// ------------------------------------
//  Simulation Parameters
// ------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 2000.0f;

int main(int argc, char* argv[]){
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION <<"\n\n";

    // -----------------
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;
    mphysicalSystem.Set_G_acc(ChVector<>(0, 0, -1));

    // ----------------------------------
    // add a mesh to be sensed by a lidar
    // ----------------------------------
    auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), false, true);
    mmesh->Transform(ChVector<>(0,0,0), ChMatrix33<>()); // scale to a difference size

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("HMMWV Chassis Mesh");
    trimesh_shape->SetStatic(true);

    // ----------------------
    // color visual materials 
    // ----------------------
    auto red = chrono_types::make_shared<ChVisualMaterial>();
    red->SetDiffuseColor({1,0,0});
    red->SetSpecularColor({1.f, 1.f, 1.f});

    auto green = chrono_types::make_shared<ChVisualMaterial>();
    green->SetDiffuseColor({0,1,0});
    green->SetSpecularColor({1.f,1.f,1.f});


    // -------------------------------------------
    // add a few box bodies to be sense by a lidar
    // -------------------------------------------
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, true, false);
    floor->SetPos({0, 0, -1});
    floor->SetBodyFixed(true);
//    floor->SetWvel_par(ChVector<>(-0.2,-0.4,-0.3));
//    floor->SetPos_dt(ChVector<>(0.1, 0,0));
    mphysicalSystem.Add(floor);
    {
        auto asset = floor->GetAssets()[0];
        if ( auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)){
            visual_asset->material_list.push_back(green);
        }
    }

    for (int i = 0; i < 100; i++){
        int x = rand() % 20;
        int y = rand() % 20;
        int z = rand() % 30;
        auto box_body = chrono_types::make_shared<ChBodyEasyBox>(0.5, 0.5, 0.5, 1000, true, false);
        box_body->SetPos({-10 + x, y + 5, 3 + z});
        mphysicalSystem.Add(box_body);
        {
            auto asset = box_body->GetAssets()[0];
            if ( auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)){
                visual_asset->material_list.push_back(red);
            }
        }
    }

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    float intensity = 0.3;
    manager->scene->AddPointLight({100, 100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->AddPointLight({-100, 100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->AddPointLight({100, -100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->AddPointLight({-100, -100, 100}, {intensity, intensity, intensity}, 500);
    manager->SetVerbose(true);
    // -----------------------------------------------
    // Create a radar and add it to the sensor manager
    // -----------------------------------------------
    auto offset_pose = chrono::ChFrame<double>({0, 0, 1}, Q_from_AngZ(CH_C_PI / 2));

    auto radar = 
        chrono_types::make_shared<ChRadarSensor>(floor,
                                                 update_rate,
                                                 offset_pose,
                                                 horizontal_samples,
                                                 vertical_samples,
                                                 horizontal_fov,
                                                 max_vert_angle,
                                                 min_vert_angle,
                                                 max_distance
                                                 );
    radar->SetName("Radar Sensor");
    radar->SetLag(lag);
    radar->SetCollectionWindow(collection_time);

    radar->PushFilter(chrono_types::make_shared<ChFilterRadarProcess>("PC from Range"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarVisualizeCluster>(640, 480, 1, "Radar Clusters"));
    const std::string out_dir = "RADAR_OUPUT/";
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarSavePC>(out_dir));
    manager->AddSensor(radar);


    auto lidar = chrono_types::make_shared<ChLidarSensor>(floor,
                                                          update_rate,
                                                          offset_pose,
                                                          horizontal_samples,
                                                          vertical_samples,
                                                          horizontal_fov,
                                                          max_vert_angle,
                                                          min_vert_angle,
                                                          max_distance);
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(lag);
    lidar->SetCollectionWindow(collection_time);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>("DI Access"));
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 1, " Lidar PC"));
//    manager->AddSensor(lidar);

    auto cam_offset_pose = chrono::ChFrame<double>({0, 0, 1}, Q_from_AngZ(CH_C_PI / 2));
    auto cam1 = chrono_types::make_shared<ChCameraSensor>(floor,   // body camera is attached to
                                                          update_rate,   // update rate in Hz
                                                          cam_offset_pose,  // offset pose
                                                          1280,   // image width
                                                          720,  // image height
                                                          horizontal_fov,           // camera's horizontal field of view
                                                          alias_factor,  // supersample factor for antialiasing
                                                          lens_model,
                                                          false);  // FOV
    cam1->SetName("World Camera Sensor");
    cam1->SetLag(lag);
    cam1->SetCollectionWindow(exposure_time);

    // Render the antialiased image
    cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "World Ray Tracing"));

  // Add the second camera to the sensor manager
    manager->AddSensor(cam1);

//    //
//    // The Visualization System
//    //
//    ChIrrApp application(&mphysicalSystem, L"Motors", core::dimension2d<u32>(800, 600));
//
//    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
//    application.AddTypicalLogo();
//    application.AddTypicalSky();
//    application.AddTypicalLights();
//    application.AddTypicalCamera(core::vector3df(-15, 15, -0));
//    application.AddLightWithShadow(vector3df(20.0f, 35.0f, -25.0f), vector3df(0, 0, 0), 55, 20, 55, 35, 512,
//                                   video::SColorf(0.6f, 0.8f, 1.0f));
//
//    // Use this function for adding a ChIrrNodeAsset to all items
//    // Otherwise use application.AssetBind(myitem); on a per-item basis.
//    application.AssetBindAll();
//
//    // Use this function for 'converting' assets into Irrlicht meshes
//    application.AssetUpdateAll();
//
//    // This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
//    // for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)
//    application.AddShadowAll();

    // -------------------
    // Simulate the system
    // -------------------
    double render_time = 0;
    float ch_time = 0.0;
    
    while(ch_time < end_time) {
//        application.BeginScene(true, true, SColor(255, 140, 161, 192));
//        application.DrawAll();

        manager->Update();

        mphysicalSystem.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float) mphysicalSystem.GetChTime();

//        application.DoStep();
//        application.EndScene();
    }
}