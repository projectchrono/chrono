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

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::sensor;

float end_time = 10.0f;
bool vis = true;

int main(int argc, char* argv[]) {
    for (int q = 2; q <= 3; q++) {
        GetLog() << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

        // -----------------
        // Create the system
        // -----------------
        ChSystemNSC sys;

        // ---------------------------------------
        // add a mesh to be visualized by a camera
        // ---------------------------------------
        auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
            GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

        int x_instances = q;
        int y_instances = q;
        int z_instances = q;
        float x_spread = 2.5f;
        float y_spread = 1.5f;
        float z_spread = 1.f;
        srand(0);
        for (int i = 0; i < x_instances; i++) {
            for (int j = 0; j < y_instances; j++) {
                for (int k = 0; k < z_instances; k++) {
                    ChVector<> p = {x_spread * (i + .5 - x_instances / 2.), y_spread * (j + .5 - y_instances / 2.),
                                    z_spread * (k + .5 - z_instances / 2.)};

                    // ChVector<> p = {10 * (float)rand() - 5, 10 * (float)rand() - 5, 10 * (float)rand() - 5};

                    ChQuaternion<> quat = Q_from_AngAxis(p.Length(), {0, 0, 1});

                    ChFrame<> f = ChFrame<>(p, quat);

                    auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
                    trimesh_shape->SetMesh(mmesh);
                    trimesh_shape->SetName("HMMWV Chassis Mesh");
                    trimesh_shape->SetMutable(false);
                    float scale = 1 * (float)rand() / (float)RAND_MAX + .01f;
                    trimesh_shape->SetScale({scale, scale, scale});

                    auto mesh_body = chrono_types::make_shared<ChBody>();
                    mesh_body->SetPos(p);
                    mesh_body->SetRot(quat);
                    mesh_body->SetBodyFixed(true);
                    mesh_body->AddVisualShape(trimesh_shape);
                    sys.Add(mesh_body);
                }
            }
        }

        auto cam_body = chrono_types::make_shared<ChBodyEasyBox>(.01, .01, .01, 1000, false, false);
        cam_body->SetBodyFixed(true);
        sys.Add(cam_body);

        // -----------------------
        // Create a sensor manager
        // -----------------------
        auto manager = std::make_shared<ChSensorManager>(&sys);
        manager->SetRayRecursions(0);
        manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 5000);
        // manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 5000);
        // manager->scene->AddPointLight({100, -100, 100}, {1, 1, 1}, 5000);
        // manager->scene->AddPointLight({-100, -100, 100}, {1, 1, 1}, 5000);

        // ------------------------------------------------
        // Create a camera and add it to the sensor manager
        // ------------------------------------------------
        auto cam = std::make_shared<ChCameraSensor>(
            cam_body,                                                            // body camera is attached to
            60.0f,                                                               // update rate in Hz
            chrono::ChFrame<double>({-10, 0, 0}, Q_from_AngAxis(0, {0, 1, 0})),  // offset pose
            1920,                                                                // image width
            1080,                                                                // image height
            (float)CH_C_PI / 3, 1, CameraLensModelType::PINHOLE, true            // FOV
        );
        cam->SetName("Camera Sensor");
        if (vis)
            cam->PushFilter(std::make_shared<ChFilterVisualize>(1280, 720));

        manager->AddSensor(cam);

        // ---------------
        // Simulate system
        // ---------------
        float orbit_radius = (x_instances / 2.0f) * x_spread + 5.0f;
        float orbit_rate = 0.5f;
        float ch_time = 0.0f;

        // double render_time = 0;

        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        while (ch_time < end_time) {
            cam->SetOffsetPose(chrono::ChFrame<double>(
                {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
                Q_from_AngAxis(ch_time * orbit_rate, {0, 0, 1})));

            manager->Update();
            sys.DoStepDynamics(0.001);

            ch_time = (float)sys.GetChTime();
        }
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        int num_meshes = (x_instances) * (y_instances) * (z_instances);
        std::cout << "Num meshes: " << num_meshes << ", Num triangles: " << num_meshes * 164745
                  << ", Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";
    }
    return 0;
}
