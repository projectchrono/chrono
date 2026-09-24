// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================
// A fixed scene for comparing car-paint reflections between render backends.
//
// A stationary Audi on a flat terrain patch under the shipped sensor/textures/sky_2_4k.hdr
// environment map, with the camera parked looking at the left rear quarter panel. That view is
// chosen because a curved, low-roughness body panel is where two ray tracers are most likely to
// disagree: it reflects the car's own silhouette against the sky, and the boundary between the
// two is a hard edge whose position depends on the reflection model rather than on the geometry.
//
// Build it on whichever backend the configuration selected and compare the images.
//
// It uses ONLY data that ships with Chrono, so no external data dir is required.
//
// Camera settings are chosen to match the Metal demo exactly: 1280x720, 60 deg HFOV, supersample_factor = 1,
// PINHOLE lens, LEGACY integrator (no GI), no denoiser, gamma 2.2.
// =============================================================================

#include <cmath>
#include <string>
#include <iostream>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRotation.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

// The scene is identical on every backend; only the label and the output directory follow the one
// the build selected, so images from two backends can sit side by side.
#if defined(CHRONO_HAS_OPTIX)
static const std::string backend_dir = "optix";
static const std::string backend_name = "OptiX";
#elif defined(CHRONO_HAS_METAL_RT)
static const std::string backend_dir = "metal";
static const std::string backend_name = "Metal";
#elif defined(CHRONO_HAS_VULKAN_RT)
static const std::string backend_dir = "vulkan";
static const std::string backend_name = "Vulkan RT";
#endif

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\n";

    // Optional CLI arg: supersample factor (default 1 to match the Metal demo). Pass 2/3 for AA comparison.
    unsigned int ss = (argc > 1) ? (unsigned int)std::max(1, atoi(argv[1])) : 1u;
    // Optional second CLI arg: nonzero enables the OptiX denoiser (off by default, matching the Metal demo).
    bool use_denoiser = (argc > 2) && atoi(argv[2]) != 0;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads(4);

    // ---- flat terrain patch (portable, ships with Chrono) ----
    RigidTerrain terrain(&sys);
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    auto patch_mat = minfo.CreateMaterial(ChContactMethod::SMC);
    auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300.0, 120.0);
    patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 60, 24);
    terrain.Initialize();

    // ---- Audi (JSON), at the origin, facing +x ----
    WheeledVehicle audi(&sys, GetVehicleDataFile("audi/json/audi_Vehicle.json"));
    audi.Initialize(ChCoordsys<>(ChVector3d(0, 0, 0.55), QUNIT));
    audi.SetChassisVisualizationType(VisualizationType::MESH);
    audi.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    audi.SetWheelVisualizationType(VisualizationType::MESH);
    {
        auto engine = ReadEngineJSON(GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"));
        auto transmission = ReadTransmissionJSON(GetVehicleDataFile("audi/json/audi_AutomaticTransmissionSimpleMap.json"));
        audi.InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));
        for (auto& axle : audi.GetAxles())
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile("audi/json/audi_TMeasyTire.json"));
                tire->SetStepsize(1e-3);
                audi.InitializeTire(tire, wheel, VisualizationType::MESH);
            }
    }

    // ---- sensor manager: match the Metal scene's lighting + HDR background exactly ----
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->SetAmbientLight(ChVector3f(0.35f, 0.35f, 0.37f));
    // Metal uses a directional "sun" with travel-direction (-0.45,-0.25,-0.85); the direction TO the source
    // is (0.45,0.25,0.85) -> elevation ~1.027 rad, azimuth ~0.508 rad (OptiX convention:
    // light_dir = (cos(el)cos(az), cos(el)sin(az), sin(el))).
    manager->scene->AddDirectionalLight(ChColor(1.4f, 1.37f, 1.3f), 1.027f, 0.508f);
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/sky_2_4k.hdr");
    manager->scene->SetBackground(b);

    // ---- camera parked close on the LEFT REAR QUARTER PANEL (offset relative to chassis; +y = left) ----
    ChVector3d cam_off(-3.0, 3.0, 1.7);
    ChVector3d look(-1.4, 0.7, 0.75);
    ChVector3d d = (look - cam_off).GetNormalized();
    ChFrame<double> cam_pose(cam_off, QuatFromAngleZ(std::atan2(d.y(), d.x())) * QuatFromAngleY(-std::asin(d.z())));

    auto cam = chrono_types::make_shared<ChCameraSensor>(audi.GetChassisBody(),  // attached to chassis
                                                         30.0f,                  // update rate (Hz)
                                                         cam_pose,               // offset pose
                                                         1280, 720,              // resolution
                                                         (float)(CH_PI / 3),     // 60 deg HFOV
                                                         ss,                     // supersample factor (default 1)
                                                         CameraLensModelType::PINHOLE,
                                                         false,          // no diffuse/GI (LEGACY default)
                                                         use_denoiser);  // denoiser (off unless argv[2])
    cam->SetName("quarter_panel");
    std::string out_dir = "quarterpanel_out/" + backend_dir + "_ss" + std::to_string(ss) + (use_denoiser ? "_denoise" : "") + "/";
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(1280, 720, "Left rear quarter panel (" + backend_name + ")"));
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir));
    manager->AddSensor(cam);

    std::cout << "Quarter-panel scene (" << backend_name << "). PNGs -> " << out_dir << ". Runs 3 s then exits.\n";

    // ---- run: parked car settles; save a handful of frames ----
    const double step = 2e-3;
    double time = 0;
    DriverInputs in;
    in.m_throttle = 0;
    in.m_steering = 0;
    in.m_braking = 1.0;  // parked
    while (time < 3.0) {
        terrain.Synchronize(time);
        audi.Synchronize(time, in, terrain);
        terrain.Advance(step);
        audi.Advance(step);
        sys.DoStepDynamics(step);
        manager->Update();
        time += step;
    }
    std::cout << "Done. Compare " << out_dir << " against the same scene built on another backend.\n";
    return 0;
}
