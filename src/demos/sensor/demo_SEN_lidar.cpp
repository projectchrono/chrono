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
// Chrono demonstration of a lidar sensor
// Simple demonstration of certain filters and the visualization of a static mesh
//
// =============================================================================

#include <cmath>
#include <cstdio>
#include <iomanip>

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/sensors/Sensor.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Lidar parameters
// -----------------------------------------------------------------------------

// Noise model attached to the sensor
enum NoiseModel {
    CONST_NORMAL_XYZI,  // Gaussian noise with constant mean and standard deviation
    NONE                // No noise model
};
NoiseModel noise_model = CONST_NORMAL_XYZI;

// Lidar return mode
// Either STRONGEST_RETURN, MEAN_RETURN, FIRST_RETURN, LAST_RETURN
LidarReturnMode return_mode = LidarReturnMode::STRONGEST_RETURN;

// Update rate in Hz
float update_rate = 5.f;

// Number of horizontal and vertical samples
unsigned int horizontal_samples = 4500;
unsigned int vertical_samples = 32;

// Horizontal and vertical field of view (radians)
float horizontal_fov = (float)(2 * CH_C_PI);  // 360 degree scan
float max_vert_angle = (float)CH_C_PI / 12;   // 15 degrees up
float min_vert_angle = (float)-CH_C_PI / 6;   // 30 degrees down

// Lag time
float lag = 0.f;

// Collection window for the lidar
float collection_time = 1 / update_rate;  // typically 1/update rate

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 2000.0f;

// Save lidar point clouds
bool save = false;

// Render lidar point clouds
bool vis = true;

// Output directories
const std::string out_dir = "SENSOR_OUTPUT/LIDAR_DEMO/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ----------------------------------
    // add a mesh to be sensed by a lidar
    // ----------------------------------
    auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
                                                                  false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("HMMWV Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape,ChFrame<>());
    mesh_body->SetBodyFixed(true);
    // sys.Add(mesh_body);

    // --------------------------------------------
    // add a few box bodies to be sensed by a lidar
    // --------------------------------------------
    auto box_body = chrono_types::make_shared<ChBodyEasyBox>(100, 100, 1, 1000, true, false);
    box_body->SetPos({0, 0, -1});
    box_body->SetBodyFixed(true);
    sys.Add(box_body);

    auto box_body_1 = chrono_types::make_shared<ChBodyEasyBox>(100, 1, 100, 1000, true, false);
    box_body_1->SetPos({0, -10, -3});
    box_body_1->SetBodyFixed(true);
    sys.Add(box_body_1);

    auto box_body_2 = chrono_types::make_shared<ChBodyEasyBox>(100, 1, 100, 1000, true, false);
    box_body_2->SetPos({0, 10, -3});
    box_body_2->SetBodyFixed(true);
    sys.Add(box_body_2);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->SetVerbose(false);

    // -----------------------------------------------
    // Create a lidar and add it to the sensor manager
    // -----------------------------------------------
    auto offset_pose = chrono::ChFrame<double>({-4, 0, 1}, Q_from_AngAxis(0, {0, 1, 0}));

    auto lidar =
        chrono_types::make_shared<ChLidarSensor>(box_body,                               // body lidar is attached to
                                                 update_rate,                            // scanning rate in Hz
                                                 offset_pose,                            // offset pose
                                                 900,                                    // number of horizontal samples
                                                 30,                                     // number of vertical channels
                                                 horizontal_fov,                         // horizontal field of view
                                                 max_vert_angle, min_vert_angle, 100.0f  // vertical field of view
        );
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(lag);
    lidar->SetCollectionWindow(collection_time);

    // -----------------------------------------------------------------
    // Create a filter graph for post-processing the data from the lidar
    // -----------------------------------------------------------------

    // Provides the host access to the Depth,Intensity data
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());

    // Renders the raw lidar data
    if (vis)
        lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(horizontal_samples / 2, vertical_samples * 5,
                                                                       "Raw Lidar Depth Data"));

    // Convert Depth,Intensity data to XYZI point
    // cloud data
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());

    // Add a noise model filter to the lidar sensor
    switch (noise_model) {
        case CONST_NORMAL_XYZI:
            lidar->PushFilter(chrono_types::make_shared<ChFilterLidarNoiseXYZI>(0.01f, 0.001f, 0.001f, 0.01f));
            break;
        case NONE:
            // Don't add any noise models
            break;
    }

    // Render the point cloud
    if (vis)
        lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 2, "Lidar Point Cloud"));

    // Access the lidar data as an XYZI buffer
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());

    // Save the XYZI data
    if (save)
        lidar->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>(out_dir + "ideal/"));

    // add sensor to the manager
    manager->AddSensor(lidar);

    // -----------------------------------------------------------------------
    // Create a multi-sample lidar, where each beam
    // is traced by multiple rays
    // -----------------------------------------------------------------------
    unsigned int sample_radius = 2;        // radius of samples to use, 1->1
                                           // sample,2->9 samples, 3->25 samples...
    float vert_divergence_angle = 0.003f;  // 3mm radius (as cited by velodyne)
    float hori_divergence_angle = 0.003f;
    auto lidar2 = chrono_types::make_shared<ChLidarSensor>(box_body,        // body lidar is attached to
                                                           update_rate,     // scanning rate in Hz
                                                           offset_pose,     // offset pose
                                                           1080,            // number of horizontal samples
                                                           32,              // number of vertical channels
                                                           horizontal_fov,  // horizontal field of view
                                                           max_vert_angle,
                                                           min_vert_angle,              // vertical field of view
                                                           100,                         // max distance
                                                           LidarBeamShape::ELLIPTICAL,  // beam shape
                                                           sample_radius,               // sample radius
                                                           vert_divergence_angle,       // vertical divergence angle
                                                           hori_divergence_angle,       // horizontal divergence angle
                                                           return_mode                  // return mode for the lidar
    );
    lidar2->SetName("Lidar Sensor 2");
    lidar2->SetLag(lag);
    lidar2->SetCollectionWindow(collection_time);

    // -----------------------------------------------------------------
    // Create a filter graph for post-processing the
    // data from the lidar
    // -----------------------------------------------------------------

    // Provides the host access to the
    // Depth,Intensity data
    lidar2->PushFilter(chrono_types::make_shared<ChFilterDIAccess>("DI Access"));

    // Renders the raw lidar data
    if (vis)
        lidar2->PushFilter(
            chrono_types::make_shared<ChFilterVisualize>(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"));

    // Convert Depth,Intensity data to XYZI point
    // cloud data
    lidar2->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>("PC from depth"));

    // Add a noise model filter to the camera sensor
    switch (noise_model) {
        case CONST_NORMAL_XYZI:
            lidar2->PushFilter(
                chrono_types::make_shared<ChFilterLidarNoiseXYZI>(0.01f, 0.001f, 0.001f, 0.01f, "Noise"));
            break;
        case NONE:
            // Don't add any noise models
            break;
    }

    // Render the point cloud
    if (vis)
        lidar2->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 1, "Lidar Point Cloud"));

    // Access the lidar data as an XYZI buffer
    lidar2->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>("XYZI Access"));

    // Save the XYZI data
    if (save)
        lidar2->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>(out_dir + "model/"));

    // add sensor to the manager
    manager->AddSensor(lidar2);

    // Lidar from JSON file - Velodyne VLP-16
    auto vlp16 = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/Velodyne/VLP-16.json"), box_body, offset_pose);
    manager->AddSensor(vlp16);

    auto hdl32e = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/Velodyne/HDL-32E.json"), box_body, offset_pose);
    manager->AddSensor(hdl32e);

    // ---------------
    // Simulate system
    // ---------------
    float orbit_rate = 2.5;
    float ch_time = 0.0;

    UserDIBufferPtr di_ideal_ptr;

    UserXYZIBufferPtr xyzi_buf;
    UserXYZIBufferPtr xyzi_buf_old;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        mesh_body->SetRot(Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1}));
        // Access the DI buffer from the ideal lidar
        // di_ideal_ptr =
        // lidar->GetMostRecentBuffer<UserDIBufferPtr>();
        // if (di_ideal_ptr->Buffer) {
        //     std::cout << "DI buffer recieved from
        //     ideal lidar model." << std::endl;
        //     std::cout << "\tLidar resolution: " <<
        //     di_ideal_ptr->Width << "x" <<
        //     di_ideal_ptr->Height << std::endl;
        //     std::cout << "\tFirst Point: [" <<
        //     di_ideal_ptr->Buffer[0].range << ", "
        //               <<
        //               di_ideal_ptr->Buffer[0].intensity
        //               << "]" << std::endl
        //               << std::endl;
        // }

        // Access the XYZI buffer from the model
        // lidar xyzi_model_ptr =
        // lidar2->GetMostRecentBuffer<UserXYZIBufferPtr>();
        // if (xyzi_model_ptr->Buffer &&
        // xyzi_ideal_ptr->Buffer) {
        //     // Calculate the mean error between
        //     the ideal and model lidar double
        //     total_error = 0; int samples = 0; for
        //     (int i = 0; i <
        //     xyzi_ideal_ptr->Height; i++) {
        //         for (int j = 0; j <
        //         xyzi_ideal_ptr->Width; j++) {
        //             if (xyzi_ideal_ptr->Buffer[i *
        //             xyzi_ideal_ptr->Width +
        //             j].intensity > 1e-3 &&
        //                 xyzi_model_ptr->Buffer[i *
        //                 xyzi_ideal_ptr->Width +
        //                 j].intensity > 1e-3) {
        //                 total_error +=
        //                 abs(xyzi_ideal_ptr->Buffer[i
        //                 * xyzi_ideal_ptr->Width +
        //                 j].y -
        //                                    xyzi_model_ptr->Buffer[i
        //                                    * xyzi_ideal_ptr->Width + j].y);
        //                 total_error +=
        //                 abs(xyzi_ideal_ptr->Buffer[i
        //                 * xyzi_ideal_ptr->Width +
        //                 j].z -
        //                                    xyzi_model_ptr->Buffer[i
        //                                    * xyzi_ideal_ptr->Width + j].z);
        //                 total_error +=
        //                 abs(xyzi_ideal_ptr->Buffer[i
        //                 * xyzi_ideal_ptr->Width +
        //                 j].intensity -
        //                                    xyzi_model_ptr->Buffer[i
        //                                    * xyzi_ideal_ptr->Width + j].intensity);
        //                 samples++;
        //             }
        //         }
        //     }
        //     std::cout << "Mean difference in lidar
        //     values: " << total_error / samples <<
        //     std::endl << std::endl;
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
