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
// Benchmark for testing changes to rendering algorimths
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
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 100.0f;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // -----------------
    // Create the system
    // -----------------
    ChSystemNSC sys;

    // ---------------------------------------
    // add a mesh to be visualized by a camera
    // ---------------------------------------

    auto floor = std::make_shared<ChBodyEasyBox>(100, 100, .01, 1000, true, false);
    floor->SetPos({0, 0, 0});
    floor->SetBodyFixed(true);
    sys.Add(floor);

    auto cart = std::make_shared<ChBodyEasyBox>(1, 1, 1, 1000, false, false);
    cart->SetPos({0, 0, 1});
    cart->SetBodyFixed(true);
    sys.Add(cart);

    // create alternating walls on left and right

    int walls = 1000;
    float wall_size = .5;
    for (int i = 0; i < walls; i++) {
        auto wall_body = std::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, wall_size, 1, 1000, true, true);
        wall_body->SetPos({4 * i * wall_size, 4, wall_size / 2});
        wall_body->SetRot(Q_from_AngX(CH_C_PI / 2));
        wall_body->SetBodyFixed(true);
        sys.Add(wall_body);

        auto wall_body1 = std::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, wall_size, 1, 1000, true, true);
        wall_body1->SetPos({4 * i * wall_size, -4, wall_size / 2});
        wall_body1->SetRot(Q_from_AngX(CH_C_PI / 2));
        wall_body1->SetBodyFixed(true);
        sys.Add(wall_body1);
    }

    // -----------------------
    // Create a sensor manager
    // -----------------------
    float step_size = 0.01f;
    auto manager = std::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 1000);

    // ------------------------------------------------
    // Create a camera and add it to the sensor manager
    // ------------------------------------------------

    auto lidar1 = std::make_shared<ChLidarSensor>(
        cart,                                                              // body lidar is attached to
        1.0f,                                                              // scanning rate in Hz
        chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1000,                                                              // number of horizontal samples
        10,                                                                // number of vertical channels
        2 * (float)CH_C_PI,                                                // horizontal field of view
        0.1f, -0.1f, 100.0f, LidarBeamShape::RECTANGULAR                   // vertical field of view
    );
    lidar1->SetName("Lidar Sensor");
    lidar1->SetLag(1);
    lidar1->SetCollectionWindow(1);
    lidar1->PushFilter(std::make_shared<ChFilterPCfromDepth>());
    lidar1->PushFilter(std::make_shared<ChFilterVisualizePointCloud>(800, 800, 1.5f));
    manager->AddSensor(lidar1);

    auto camera = std::make_shared<ChCameraSensor>(
        cart,                                                              // body lidar is attached to
        10.0f,                                                             // scanning rate in Hz
        chrono::ChFrame<double>({0, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
        1280,                                                              // number of horizontal samples
        720,                                                               // number of vertical channels
        (float)CH_C_PI / 4                                                 // horizontal field of view
    );
    camera->SetName("Camera Sensor");
    camera->SetLag(0);
    camera->SetCollectionWindow(0);
    camera->PushFilter(std::make_shared<ChFilterVisualize>(1280, 720));
    manager->AddSensor(camera);

    float speed = 16;

    while (sys.GetChTime() < end_time) {
        // move the cart
        cart->SetPos(cart->GetPos() + ChVector<>({speed * step_size, 0, 0}));

        manager->Update();
        sys.DoStepDynamics(step_size);
    }

    return 0;
}
