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
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterRadarProcess.h"
#include "chrono_sensor/filters/ChFilterRadarSavePC.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

// ------------------------------------
// Radar Parameters
// ------------------------------------

// Update rate in Hz
float update_rate = 5.f;

// horizontal field of view of camera
int alias_factor = 1;
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Exposure (in seconds) of each image
float exposure_time = 0.02f;

// Number of horizontal and vertical samples
unsigned int horizontal_samples = 100;
unsigned int vertical_samples = 100;

// Field of View
float horizontal_fov = float(CH_C_PI / 2);  // 20 degree scan
float vertical_fov = float(CH_C_PI / 3);    // 12 degrees down

// camera can have same view as radar
float aspect_ratio = horizontal_fov / vertical_fov;
float width = 960;
float height = width / aspect_ratio;

// max detection range
float max_distance = 100;

// lag time
float lag = 0.f;

// Collection window for the radar
float collection_time = 1 / update_rate;  // typically 1/update rate

// Output directories
const std::string out_dir = "RADAR_OUTPUT/";
// ------------------------------------
//  Simulation Parameters
// ------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 2000.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    // Create the system
    // -----------------
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, 0));

    // ----------------------
    // color visual materials
    // ----------------------
    auto red = chrono_types::make_shared<ChVisualMaterial>();
    red->SetDiffuseColor({1, 0, 0});
    red->SetSpecularColor({1.f, 1.f, 1.f});

    auto green = chrono_types::make_shared<ChVisualMaterial>();
    green->SetDiffuseColor({0, 1, 0});
    green->SetSpecularColor({1.f, 1.f, 1.f});

    // -------------------------------------------
    // add a few box bodies to be sense by a radar
    // -------------------------------------------
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false);
    floor->SetPos({0, 0, -1});
    floor->SetBodyFixed(true);
    //    floor->SetWvel_par(ChVector<>(-0.2,-0.4,-0.3));
    //    floor->SetPos_dt(ChVector<>(0.1, 0, 0));
    sys.Add(floor);
    floor->GetVisualModel()->GetShapes()[0].first->AddMaterial(green);



//    auto wall = chrono_types::make_shared<ChBodyEasyBox>(1,30,30, 1000, true, false);
//    wall->SetPos({15,0,4});
//    wall->SetBodyFixed(true);
//    sys.Add(wall);
//    wall->GetVisualModel()->GetShapes()[0].first->AddMaterial(red);

    auto box = chrono_types::make_shared<ChBodyEasyBox>(1,1,1, 1000, true, false);
    box->SetPos({4,3,2});
    box->SetBodyFixed(true);
    sys.Add(box);
    box->GetVisualModel()->GetShapes()[0].first->AddMaterial(green);

    auto box1 = chrono_types::make_shared<ChBodyEasyBox>(1,1,1, 1000, true, false);
    box1->SetPos({4,-3,2});
    box1->SetBodyFixed(true);
    sys.Add(box1);
    box1->GetVisualModel()->GetShapes()[0].first->AddMaterial(green);

    auto box2 = chrono_types::make_shared<ChBodyEasyBox>(1,1,1, 1000, true, false);
    box2->SetPos({4,0,2});
    box2->SetBodyFixed(true);
    sys.Add(box2);
    box2->GetVisualModel()->GetShapes()[0].first->AddMaterial(green);


    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    float intensity = 0.3f;
    manager->scene->AddPointLight({100, 100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->AddPointLight({-100, 100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->AddPointLight({100, -100, 100}, {intensity, intensity, intensity}, 500);
    manager->scene->AddPointLight({-100, -100, 100}, {intensity, intensity, intensity}, 500);
    manager->SetVerbose(false);
    // -----------------------------------------------
    // Create a radar and add it to the sensor manager
    // -----------------------------------------------
    auto offset_pose = chrono::ChFrame<double>({0, 0, 1}, Q_from_AngZ(0));

    auto radar =
        chrono_types::make_shared<ChRadarSensor>(floor, update_rate, offset_pose, horizontal_samples, vertical_samples,
                                                 horizontal_fov, vertical_fov, max_distance);
    radar->SetName("Radar Sensor");
    radar->SetLag(lag);
    radar->SetCollectionWindow(collection_time);

    radar->PushFilter(chrono_types::make_shared<ChFilterRadarProcess>("PC from Range"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarVisualizeCluster>(640, 480, 1, "Radar Clusters"));
    manager->AddSensor(radar);

    auto cam_offset_pose = chrono::ChFrame<double>({0, 0, 1}, Q_from_AngZ(0));
    auto cam1 = chrono_types::make_shared<ChCameraSensor>(floor,            // body camera is attached to
                                                          update_rate,      // update rate in Hz
                                                          cam_offset_pose,  // offset pose
                                                          width,            // image width
                                                          height,           // image height
                                                          horizontal_fov,   // camera's horizontal field of view
                                                          alias_factor,     // supersample factor for antialiasing
                                                          lens_model,
                                                          false);  // FOV
    cam1->SetName("World Camera Sensor");
    cam1->SetLag(lag);
    cam1->SetCollectionWindow(exposure_time);
    cam1->PushFilter(chrono_types::make_shared<ChFilterVisualize>(width, height, "World Ray Tracing"));
    manager->AddSensor(cam1);

    // -------------------
    // Simulate the system
    // -------------------
    // double render_time = 0;
    float ch_time = 0.0;

    while (ch_time < end_time) {
        manager->Update();

        sys.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float)sys.GetChTime();
    }
}