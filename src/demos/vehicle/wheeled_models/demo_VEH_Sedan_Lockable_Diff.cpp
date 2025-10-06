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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Demo the Sedan model with lockable differential gear box.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

// =============================================================================

const double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    bool lock_diff = (argc > 1) ? true : false;

    if (lock_diff) {
        std::cout << "The differential box is locked\n";
    } else {
        std::cout << "The differential box is unlocked\n";
    }

    // Create the vehicle
    Sedan sedan;
    sedan.SetContactMethod(ChContactMethod::SMC);
    sedan.SetChassisCollisionType(CollisionType::NONE);
    sedan.SetChassisFixed(false);
    sedan.SetInitPosition(ChCoordsys<>(ChVector3d(-40, 0, 1.0)));
    sedan.SetTireType(TireModelType::TMEASY);
    sedan.SetTireStepSize(1e-3);
    sedan.Initialize();

    sedan.SetChassisVisualizationType(VisualizationType::NONE);
    sedan.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    sedan.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    sedan.SetWheelVisualizationType(VisualizationType::MESH);
    sedan.SetTireVisualizationType(VisualizationType::MESH);

    sedan.LockAxleDifferential(0, lock_diff);

    // Associate a collision system
    sedan.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the terrain
    RigidTerrain terrain(sedan.GetSystem());

    auto patch1_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    patch1_mat->SetFriction(0.1f);
    patch1_mat->SetRestitution(0.01f);
    patch1_mat->SetYoungModulus(2e7f);
    patch1_mat->SetPoissonRatio(0.3f);

    auto patch2_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    patch2_mat->SetFriction(0.9f);
    patch2_mat->SetRestitution(0.01f);
    patch2_mat->SetYoungModulus(2e7f);
    patch2_mat->SetPoissonRatio(0.3f);

    auto patch1 = terrain.AddPatch(patch1_mat, ChCoordsys<>(ChVector3d(0, -25, 0), QUNIT), 100, 50);
    auto patch2 = terrain.AddPatch(patch2_mat, ChCoordsys<>(ChVector3d(0, +25, 0), QUNIT), 100, 50);

    patch1->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch1->SetTexture(GetVehicleDataFile("terrain/textures/dirt.jpg"), 200, 50);

    patch2->SetColor(ChColor(0.5f, 0.5f, 0.8f));
    patch2->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 50);

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Sedan Demo Locked Diff");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 1.5), 4.0, 0.5);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&sedan.GetVehicle());

    // Initialize output
    const std::string out_dir = GetChronoOutputPath() + "SEDAN";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    utils::ChWriterCSV wheelomega_csv("\t");

    // Simulation loop
    while (vis->Run()) {
        double time = sedan.GetSystem()->GetChTime();

        if (time > 15 || sedan.GetVehicle().GetPos().x() > 49)
            break;

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Driver inputs
        DriverInputs driver_inputs = {0, 0, 0};
        if (time > 2)
            driver_inputs.m_throttle = 0.6;
        else if (time > 1)
            driver_inputs.m_throttle = 0.6 * (time - 1);

        // Output
        double omega_front_left = sedan.GetVehicle().GetSuspension(0)->GetAxleSpeed(LEFT);
        double omega_front_right = sedan.GetVehicle().GetSuspension(0)->GetAxleSpeed(RIGHT);
        wheelomega_csv << time << omega_front_left << omega_front_right << std::endl;

        // Synchronize subsystems
        terrain.Synchronize(time);
        sedan.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for all subsystems
        terrain.Advance(step_size);
        sedan.Advance(step_size);
        vis->Advance(step_size);
    }

    wheelomega_csv.WriteToFile(out_dir + "/FrontWheelOmega_" + std::to_string(lock_diff) + ".csv");

    return 0;
}
