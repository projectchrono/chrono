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
// Demonstration of simulating two vehicles simultaneously.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Simulation step sizes
double step_size = 0.005;


// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------
    // Create systems
    // --------------

    // Chrono system
    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.SetSolverMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // Create the terrain
    RigidTerrain terrain(&sys);
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 100);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create and initialize the first vehicle
    HMMWV_Reduced hmmwv_1(&sys);
    hmmwv_1.SetInitPosition(ChCoordsys<>(ChVector<>(0, -1.5, 1.0), QUNIT));
    hmmwv_1.SetPowertrainType(PowertrainModelType::SIMPLE);
    hmmwv_1.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv_1.SetTireType(TireModelType::RIGID);
    hmmwv_1.Initialize();
    hmmwv_1.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_1.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    std::vector<ChDataDriver::Entry> driver_data_1;
    driver_data_1.push_back({0.0, 0, 0.0, 0});
    driver_data_1.push_back({0.5, 0, 0.0, 0});
    driver_data_1.push_back({0.7, 0.3, 0.7, 0});
    driver_data_1.push_back({1.0, 0.3, 0.7, 0});
    driver_data_1.push_back({3.0, 0.5, 0.1, 0});
    ChDataDriver driver_1(hmmwv_1.GetVehicle(), driver_data_1);
    driver_1.Initialize();

    // Create and initialize the second vehicle
    HMMWV_Reduced hmmwv_2(&sys);
    hmmwv_2.SetInitPosition(ChCoordsys<>(ChVector<>(7, 1.5, 1.0), QUNIT));
    hmmwv_2.SetPowertrainType(PowertrainModelType::SIMPLE);
    hmmwv_2.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv_2.SetTireType(TireModelType::RIGID);
    hmmwv_2.Initialize();
    hmmwv_2.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_2.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    std::vector<ChDataDriver::Entry> driver_data_2;
    driver_data_2.push_back({ 0.0, 0, 0.0, 0 });
    driver_data_2.push_back({ 0.5, 0, 0.0, 0 });
    driver_data_2.push_back({ 0.7, -0.3, 0.7, 0 });
    driver_data_2.push_back({ 1.0, -0.3, 0.7, 0 });
    driver_data_2.push_back({ 3.0, -0.5, 0.1, 0 });
    ChDataDriver driver_2(hmmwv_2.GetVehicle(), driver_data_2);
    driver_2.Initialize();

    // Create the vehicle Irrlicht interface (associated with 1st vehicle)
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Two cars demo");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    vis->SetChaseCameraState(utils::ChChaseCamera::Track);
    vis->SetChaseCameraPosition(ChVector<>(-15, 0, 2.0));
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddTypicalLights();
    vis->AttachVehicle(&hmmwv_1.GetVehicle());

    // ---------------
    // Simulation loop
    // ---------------

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();

        // Driver inputs
        DriverInputs driver_inputs_1 = driver_1.GetInputs();
        DriverInputs driver_inputs_2 = driver_2.GetInputs();

        // Update modules (process inputs from other modules)
        driver_1.Synchronize(time);
        driver_2.Synchronize(time);
        hmmwv_1.Synchronize(time, driver_inputs_1, terrain);
        hmmwv_2.Synchronize(time, driver_inputs_2, terrain);
        terrain.Synchronize(time);
        vis->Synchronize("", driver_inputs_1);

        // Advance simulation for one timestep for all modules.
        driver_1.Advance(step_size);
        driver_2.Advance(step_size);
        hmmwv_1.Advance(step_size);
        hmmwv_2.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);

        vis->EndScene();
    }

    return 0;
}
