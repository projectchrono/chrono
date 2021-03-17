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
#include "chrono_sensor/filters/ChFilterRadarPCfromRange.h"
#include "chrono_sensor/filters/ChFilterRadarSavePC.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterRadarDBScan.h"



using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;


// ------------------------------------
// Radar Parameters
// ------------------------------------

// Update rate in Hz
float update_rate = 5.f;

// Number of horizontal and vertical samples
unsigned int horizontal_samples = 500;
unsigned int vertical_samples = 16;

// Field of View
float horizontal_fov = (float)(2 * CH_C_PI); // 360 degree scan
float max_vert_angle = (float) CH_C_PI / 12; // 15 degrees up
float min_vert_angle = (float) CH_C_PI / 30; // 30 degrees down

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
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

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

    // -------------------------------------------
    // add a few box bodies to be sense by a lidar
    // -------------------------------------------
    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(100, 100, 1, 1000, true, false);
    box_body->SetPos({0, 0, -1});
    box_body->SetBodyFixed(true);
    mphysicalSystem.Add(box_body);

    auto box_body_1 = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 10, 1000, true, false);
    box_body_1->SetPos({0, -10, 0});
    box_body_1->SetBodyFixed(true);
    mphysicalSystem.Add(box_body_1);

    auto box_body_2 = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 10, 1000, true, false);
    box_body_2->SetPos({0, 10, 0});
    box_body_2->SetBodyFixed(true);
    mphysicalSystem.Add(box_body_2);


    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->SetVerbose(true);
    manager->SetKeyframeSizeFromTimeStep((float)step_size, 0.2f);
    std::cout<<std::endl<<std::endl;
    // -----------------------------------------------
    // Create a radar and add it to the sensor manager
    // -----------------------------------------------
    auto offset_pose = chrono::ChFrame<double>({-4, 0, 1}, Q_from_AngAxis(0, {0 ,1, 0}));
    auto radar = 
        chrono_types::make_shared<ChRadarSensor>(box_body,
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

    radar->PushFilter(chrono_types::make_shared<ChFilterRadarAccess>("DI Access"));
    radar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(horizontal_samples / 2, vertical_samples * 5, "Raw Radar Range Data"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarPCfromRange>("PC from Range"));
    radar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 1, "Radar Point Cloud"));
    radar->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>(out_dir));
//    radar->PushFilter(chrono_types::make_shared<ChFilterRadarDBScan>("DBScan"));
    manager->AddSensor(radar);


//    auto lidar = chrono_types::make_shared<ChLidarSensor>(box_body,
//                                                          update_rate,
//                                                          offset_pose,
//                                                          horizontal_samples,
//                                                          vertical_samples,
//                                                          horizontal_fov,
//                                                          max_vert_angle,
//                                                          min_vert_angle,
//                                                          max_distance);
//    lidar->SetName("Lidar Sensor 1");
//    lidar->SetLag(lag);
//    lidar->SetCollectionWindow(collection_time);
//
//    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
//    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(horizontal_samples / 2, vertical_samples * 5, "Raw Lidar Depth Data"));
//    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 1, " Lidar PC"));
//    lidar->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>(out_dir));
//    manager->AddSensor(lidar);

    // -------------------
    // Simulate the system
    // -------------------
    double render_time = 0;
    float ch_time = 0.0;
    
    int count = 0;

    while(ch_time < end_time) {
        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size);

        // Get the current time of the simulation
        ch_time = (float) mphysicalSystem.GetChTime();
    }
}