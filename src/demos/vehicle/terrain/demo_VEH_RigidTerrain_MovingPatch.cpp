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
// Authors: Radu Serban
// =============================================================================
//
// Demonstration of using dynamic RigidTerrain patches.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// TODO: currently not working properly with Chrono::VSG.

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Simulation step sizes
    double step_size = 3e-3;
    double tire_step_size = 1e-3;

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Reduced hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::NSC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetChassisCollisionType(CollisionType::NONE);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT));
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

    auto sys = hmmwv.GetSystem();
    auto coll_sys = sys->GetCollisionSystem();

    // Create the initial two patches
    RigidTerrain terrain(sys);

    double patch_len = 10;

    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);

    std::vector<std::string> textfiles = {GetVehicleDataFile("terrain/textures/concrete.jpg"),
                                          GetVehicleDataFile("terrain/textures/dirt.jpg")};
    std::vector<std::shared_ptr<RigidTerrain::Patch>> patches(2);
    double x_patch;

    x_patch = -patch_len / 2;
    patches[0] = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(x_patch, 0, 0), QUNIT), patch_len, 12);
    patches[0]->SetColor(ChColor(1, 1, 1));
    patches[0]->SetTexture(textfiles[0], 2, 2);

    x_patch += patch_len;
    patches[1] = terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(x_patch, 0, 0), QUNIT), patch_len, 12);
    patches[1]->SetColor(ChColor(1, 1, 1));
    patches[1]->SetTexture(textfiles[1], 2, 2);

    double x_switch = x_patch + patch_len / 2 - 4;
    int last_patch = 0;

    terrain.Initialize();

    // Create a straight line path and the path-follower driver
    auto path = StraightLinePath(ChVector3d(-1, 0, 0.1), ChVector3d(10 * patch_len, 0, 0.1), 4);
    double x_end = path->GetPoints().back().x() - 10;

    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "my_path", 8.0);
    driver.SetColor(ChColor(0.0f, 0.0f, 0.8f));
    driver.GetSteeringController().SetLookAheadDistance(5);
    driver.GetSteeringController().SetGains(0.8, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle run-time visualization interface
    std::shared_ptr<ChVisualSystem> vis_sys;
    ChVector3d cam_pos;
    {
        cam_pos = ChVector3d(0, -10, 1.5);

        // Create the vehicle Irrlicht interface
        auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
        vis_irr->AttachSystem(hmmwv.GetSystem());
        vis_irr->SetWindowTitle("Rigid Terrain Demo");
        vis_irr->SetWindowSize(1200, 800);
        vis_irr->Initialize();
        vis_irr->AddLightDirectional();
        vis_irr->AddSkyBox();
        vis_irr->AddLogo();
        vis_irr->AddCamera(cam_pos, hmmwv.GetVehicle().GetPos());

        vis_sys = vis_irr;
    }

    // Simulation loop
    hmmwv.GetVehicle().EnableRealtime(true);

    while (vis_sys->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();

        // Dynamically remove and add patches when reaching current switch distance
        auto veh_pos = hmmwv.GetVehicle().GetPos();

        if (veh_pos.x() > x_switch) {
            // Delete the patch currently behind vehicle
            // The patch visual and collision models are automatically dissociated
            terrain.RemovePatch(patches[last_patch]);

            // Create new patch ahead of the vehicle.
            x_patch += patch_len;
            patches[last_patch] =
                terrain.AddPatch(patch_mat, ChCoordsys<>(ChVector3d(x_patch, 0, 0), QUNIT), patch_len, 12);
            patches[last_patch]->SetColor(ChColor(1, 1, 1));
            patches[last_patch]->SetTexture(textfiles[last_patch], 2, 2);

            // Explicitly associate (bind) the visual and collision models of new patch
            terrain.BindPatch(patches[last_patch]);

            // Set next switch location
            x_switch += patch_len;

            // Update index of patch behind vehicle
            last_patch = 1 - last_patch;
        }

        // Update camera position
        cam_pos.x() = veh_pos.x();
        vis_sys->SetCameraPosition(cam_pos);
        vis_sys->SetCameraTarget(veh_pos);

        // Render scene
        vis_sys->BeginScene();
        vis_sys->Render();
        vis_sys->EndScene();

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();
        if (veh_pos.x() > x_end) {
            driver_inputs.m_braking = 1;
            driver_inputs.m_throttle = 0;
        }

        // Update systems
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);

        // Advance simulation for all systems
        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
    }

    return 0;
}
