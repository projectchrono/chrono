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
// Chrono demonstration of a camera sensor
// Demonstrates a camera with a neural network filter
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

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"

#include "chrono_sensor/tensorrt/ChFilterUFF.h"
#include "chrono_sensor/tensorrt/ChFilterONNX.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Neural network filter to add
// All filters implemented using TensorRT
enum NNType {
    ONNX,  // ONNX formatted neural network
    UFF    // UFF formatted neural network
};
NNType nn_type = ONNX;

// Update rate in Hz
int update_rate = 30;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Camera's horizontal field of view
float fov = CH_C_PI / 3.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0;

// Exposure (in seconds) of each image
float exposure_time = 0;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 1e-3;

// Simulation end time
float end_time = 20.0f;

// Save camera images
bool save = false;

// Render camera images
bool vis = true;

// Output directory
const std::string out_dir = "SENSOR_OUTPUT/NN_DEMO/";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC mphysicalSystem;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------
    auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(mmesh);
    trimesh_shape->SetName("HMMWV Chassis Mesh");
    trimesh_shape->SetMutable(false);

    auto mesh_body = chrono_types::make_shared<ChBody>();
    mesh_body->SetPos({0, 0, 0});
    mesh_body->AddVisualShape(trimesh_shape,ChFrame<>());
    mesh_body->SetBodyFixed(true);
    mphysicalSystem.Add(mesh_body);

    // -----------------------
    // Create a sensor manager
    // -----------------------
    auto manager = chrono_types::make_shared<ChSensorManager>(&mphysicalSystem);
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 500);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------
    auto offset_pose = chrono::ChFrame<double>({-8, 0, 1}, Q_from_AngAxis(0, {0, 1, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(mesh_body,     // body camera is attached to
                                                         update_rate,   // update rate in Hz
                                                         offset_pose,   // offset pose
                                                         image_width,   // image width
                                                         image_height,  // image height
                                                         fov            // camera's horizontal field of view
    );
    cam->SetName("Camera Sensor");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    // --------------------------------------------------------------------
    // Create a filter graph for post-processing the images from the camera
    // --------------------------------------------------------------------

    // add a neural net as a filter
    switch (nn_type) {
        case ONNX:
            // Add an onnx formatted neural network filter
            cam->PushFilter(chrono_types::make_shared<ChFilterONNX>());
            break;
        case UFF:
            // Add an uff formated neural network filter
            cam->PushFilter(chrono_types::make_shared<ChFilterUFF>());
            break;
    }

    // Visualizes the image at current point in the filter graph
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Before Grayscale Filter"));

    if (save)
        // Save the current image to a png file at the specified path
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir + "onnx/"));

    // add sensor to the manager
    manager->AddSensor(cam);

    // ---------------
    // Simulate system
    // ---------------
    // Demonstration shows cameras panning around a stationary mesh.
    // Parameters used to compute offset pose
    float orbit_radius = 10.f;
    float orbit_rate = 0.5;
    float ch_time = 0.0;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    while (ch_time < end_time) {
        // Update camera's offset pose
        cam->SetOffsetPose(chrono::ChFrame<double>(
            {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
            Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

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
