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

// Run benchmark tests for a number of meshes between MIN^3 and MAX^3 (inclusive)
#define TEST_MIN_MESHES 2
#define TEST_MAX_MESHES 4

// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
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
using namespace chrono::sensor;

float end_time = 10.0f;
bool vis = true;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2019 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    for (int q = TEST_MIN_MESHES; q <= TEST_MAX_MESHES; q++) {
        // Create the system
        ChSystemNSC sys;
        sys.SetGravityY();

        // Add mesh instances to be visualized by a camera
        auto mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
            GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), false, true);
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size

        srand(0);

        float x_spread = 2.5f;
        float y_spread = 1.5f;
        float z_spread = 1.0f;

        for (int i = 0; i < q; i++) {
            for (int j = 0; j < q; j++) {
                for (int k = 0; k < q; k++) {
                    ChVector3d p = {x_spread * (i + 0.5 * (1 - q)), y_spread * (j + 0.5 * (1 - q)),
                                    z_spread * (k + 0.5 * (1 - q))};

                    auto trimesh_shape = std::make_shared<ChVisualShapeTriangleMesh>();
                    trimesh_shape->SetMesh(mmesh);
                    trimesh_shape->SetName("HMMWV Chassis Mesh");
                    float scale = 1 * (float)rand() / (float)RAND_MAX + .01f;
                    trimesh_shape->SetScale({scale, scale, scale});

                    auto mesh_body = chrono_types::make_shared<ChBody>();
                    mesh_body->SetPos(p);
                    mesh_body->SetRot(QuatFromAngleZ(p.Length()));
                    mesh_body->SetFixed(true);
                    mesh_body->AddVisualShape(trimesh_shape);
                    sys.Add(mesh_body);
                }
            }
        }

        auto cam_body = chrono_types::make_shared<ChBodyEasyBox>(.01, .01, .01, 1000, false, false);
        cam_body->SetFixed(true);
        sys.Add(cam_body);

        // -----------------------
        // Create a sensor manager
        // -----------------------
        auto manager = std::make_shared<ChSensorManager>(&sys);
        manager->SetRayRecursions(9); // number of ray bounces for tracing each path
        manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 5000);
        // manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 5000);
        // manager->scene->AddPointLight({100, -100, 100}, {1, 1, 1}, 5000);
        // manager->scene->AddPointLight({-100, -100, 100}, {1, 1, 1}, 5000);

        // ------------------------------------------------
        // Create a camera and add it to the sensor manager
        // ------------------------------------------------
        auto cam = std::make_shared<ChCameraSensor>(
            cam_body,                                                               // body camera is attached to
            60.0f,                                                                  // update rate in Hz
            chrono::ChFrame<double>({-10, 0, 0}, QuatFromAngleAxis(0, {0, 1, 0})),  // offset pose
            1920,                                                                   // image width
            1080,                                                                   // image height
            (float)CH_PI / 3, 1, CameraLensModelType::PINHOLE, true, true           // FOV, samples per pixel (spp), lens model type, consider diffuse reflection, use OptiX denoiser
        );
        cam->SetName("Camera Sensor");
        if (vis)
            cam->PushFilter(std::make_shared<ChFilterVisualize>(1280, 720));

        manager->AddSensor(cam);

        // ---------------
        // Simulate system
        // ---------------
        float orbit_radius = (q / 2.0f) * x_spread + 5.0f;
        float orbit_rate = 0.5f;
        float ch_time = 0.0f;

        // double render_time = 0;

        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        while (ch_time < end_time) {
            cam->SetOffsetPose(chrono::ChFrame<double>(
                {-orbit_radius * cos(ch_time * orbit_rate), -orbit_radius * sin(ch_time * orbit_rate), 1},
                QuatFromAngleAxis(ch_time * orbit_rate, {0, 0, 1})));

            manager->Update();
            sys.DoStepDynamics(0.001);

            ch_time = (float)sys.GetChTime();
        }
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        int num_meshes = q * q * q;
        std::cout << "Num meshes: " << num_meshes << ", Num triangles: " << num_meshes * 164745
                  << ", Simulation time: " << ch_time << "s, wall time: " << wall_time.count() << "s.\n";
    }
    return 0;
}
