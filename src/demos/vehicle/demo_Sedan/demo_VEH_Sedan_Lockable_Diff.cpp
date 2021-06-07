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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

// =============================================================================

const double step_size = 1e-3;
const std::string out_dir = GetChronoOutputPath() + "SEDAN";

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    bool lock_diff = (argc > 1) ? true : false;

    if (lock_diff) {
        GetLog() << "The differential box is locked\n";
    } else {
        GetLog() << "The differential box is unlocked\n";
    }

    // Create the vehicle
    Sedan my_sedan;
    my_sedan.SetContactMethod(ChContactMethod::SMC);
    my_sedan.SetChassisCollisionType(CollisionType::NONE);
    my_sedan.SetChassisFixed(false);
    my_sedan.SetInitPosition(ChCoordsys<>(ChVector<>(-40, 0, 1.0)));
    my_sedan.SetTireType(TireModelType::TMEASY);
    my_sedan.SetTireStepSize(1e-3);
    my_sedan.Initialize();

    my_sedan.SetChassisVisualizationType(VisualizationType::NONE);
    my_sedan.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_sedan.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_sedan.SetWheelVisualizationType(VisualizationType::MESH);
    my_sedan.SetTireVisualizationType(VisualizationType::MESH);

    my_sedan.LockAxleDifferential(0, lock_diff);

    // Create the terrain
    RigidTerrain terrain(my_sedan.GetSystem());

    auto patch1_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch1_mat->SetFriction(0.1f);
    patch1_mat->SetRestitution(0.01f);
    patch1_mat->SetYoungModulus(2e7f);
    patch1_mat->SetPoissonRatio(0.3f);

    auto patch2_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch2_mat->SetFriction(0.9f);
    patch2_mat->SetRestitution(0.01f);
    patch2_mat->SetYoungModulus(2e7f);
    patch2_mat->SetPoissonRatio(0.3f);

    auto patch1 = terrain.AddPatch(patch1_mat, ChVector<>(0, -25, 0), ChVector<>(0, 0, 1), 100, 50);
    auto patch2 = terrain.AddPatch(patch2_mat, ChVector<>(0, +25, 0), ChVector<>(0, 0, 1), 100, 50);

    patch1->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch1->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 200, 50);

    patch2->SetColor(ChColor(0.5f, 0.5f, 0.8f));
    patch2->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 50);

    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_sedan.GetVehicle(), L"Sedan Demo Locked Diff");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.5), 4.0, 0.5);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    utils::CSV_writer wheelomega_csv("\t");

    // Simulation loop
    while (app.GetDevice()->run()) {
        double time = my_sedan.GetSystem()->GetChTime();

        if (time > 15 || my_sedan.GetVehicle().GetVehiclePos().x() > 49)
            break;

        // Render scene
        app.BeginScene();
        app.DrawAll();
        app.EndScene();

        // Driver inputs
        ChDriver::Inputs driver_inputs = {0, 0, 0};
        if (time > 2)
            driver_inputs.m_throttle = 0.6;
        else if (time > 1)
            driver_inputs.m_throttle = 0.6 * (time - 1);

        // Output
        double omega_front_left = my_sedan.GetVehicle().GetSuspension(0)->GetAxleSpeed(LEFT);
        double omega_front_right = my_sedan.GetVehicle().GetSuspension(0)->GetAxleSpeed(RIGHT);
        wheelomega_csv << time << omega_front_left << omega_front_right << std::endl;

        // Synchronize subsystems
        terrain.Synchronize(time);
        my_sedan.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        // Advance simulation for all subsystems
        terrain.Advance(step_size);
        my_sedan.Advance(step_size);
        app.Advance(step_size);
    }

    wheelomega_csv.write_to_file(out_dir + "/FrontWheelOmega_" + std::to_string(lock_diff) + ".csv");

    return 0;
}
