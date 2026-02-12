// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of using a RigidTerrain constructed from different patches.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// #define USE_JSON

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create systems
    // --------------

    // Simulation step sizes
    double step_size = 3e-3;
    double tire_step_size = 1e-3;

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Reduced hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::NSC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetChassisCollisionType(CollisionType::NONE);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-10, 0, 1), ChQuaternion<>(1, 0, 0, 0)));
    hmmwv.SetEngineType(EngineModelType::SIMPLE);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv.SetBrakeType(BrakeType::SHAFTS);
    hmmwv.SetTireType(TireModelType::TMEASY);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

#ifdef USE_JSON
    // Create the terrain from JSON specification file
    RigidTerrain terrain(hmmwv.GetSystem(), GetVehicleDataFile("terrain/RigidPatches.json"), true);
#else
    // Create the terrain patches programatically
    RigidTerrain terrain(hmmwv.GetSystem());

    if (true) {
        auto patch1_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        patch1_mat->SetFriction(0.9f);
        patch1_mat->SetRestitution(0.01f);
        auto patch1 = terrain.AddPatch(patch1_mat, ChCoordsys<>(ChVector3d(-16, 0, 0.08), QUNIT), 32, 20);
        patch1->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        patch1->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 20, 20);

        auto patch2_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        patch2_mat->SetFriction(0.9f);
        patch2_mat->SetRestitution(0.01f);
        auto patch2 = terrain.AddPatch(patch1_mat, ChCoordsys<>(ChVector3d(16, 0, 0.08), QUNIT), 32, 20);
        patch2->SetColor(ChColor(1.0f, 0.5f, 0.5f));
        patch2->SetTexture(GetVehicleDataFile("terrain/textures/concrete.jpg"), 20, 20);

        auto patch3_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        patch3_mat->SetFriction(0.9f);
        patch3_mat->SetRestitution(0.01f);
        auto patch3 = terrain.AddPatch(patch3_mat, ChCoordsys<>(ChVector3d(0, -42, 0), QUNIT),
                                       GetVehicleDataFile("terrain/meshes/bump.obj"));
        patch3->SetColor(ChColor(0.5f, 0.5f, 0.8f));
        patch3->SetTexture(GetVehicleDataFile("terrain/textures/dirt.jpg"), 6.0f, 6.0f);

        auto patch4_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        patch4_mat->SetFriction(0.9f);
        patch4_mat->SetRestitution(0.01f);
        auto patch4 = terrain.AddPatch(patch4_mat, ChCoordsys<>(ChVector3d(0, 42, 0), QuatFromAngleZ(CH_PI_2)),
                                       GetVehicleDataFile("terrain/height_maps/convex64.bmp"), 64.0, 64.0, 0.0, 3.0);
        patch4->SetTexture(GetVehicleDataFile("terrain/textures/grass.jpg"), 6.0f, 6.0f);
    }
    
    if (false) {
        auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
        patch_mat->SetFriction(0.9f);
        patch_mat->SetRestitution(0.01f);
        auto patch = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(0, 0, 10), QUNIT),
                                      GetVehicleDataFile("terrain/multilayer/multilayer-terrain.obj"));
    }

    terrain.Initialize();
#endif

    // Set the time response for steering and throttle keyboard inputs.
    double render_step_size = 1.0 / 50;  // FPS = 50
    double steering_time = 1.0;          // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;          // time to go from 0 to +1
    double braking_time = 0.3;           // time to go from 0 to +1

    // Create an interactive driver
    ChInteractiveDriver driver(hmmwv.GetVehicle());
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);
    driver.Initialize();

    // Create the vehicle run-time visualization interface
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Rigid Terrain Demo");
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.75);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&hmmwv.GetVehicle());
            vis_irr->AttachDriver(&driver);

            vis = vis_irr;
#endif
            break;
        }

        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Rigid Terrain Demo");
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 8.0, 0.75);
            vis_vsg->AttachVehicle(&hmmwv.GetVehicle());
            vis_vsg->AttachDriver(&driver);
            vis_vsg->AttachTerrain(&terrain);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->EnableSkyBox();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    ////ChWriterCSV out(" ");
    ////for (int ix = 0; ix < 20; ix++) {
    ////    double x = ix * 1.0;
    ////    for (int iy = 0; iy < 100; iy++) {
    ////        double y = iy * 1.0;
    ////        double z = terrain.CalcHeight(x, y);
    ////        out << x << y << z << std::endl;
    ////    }
    ////}
    ////out.WriteToFile("terrain.out");

    // ---------------
    // Simulation loop
    // ---------------

    hmmwv.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}
