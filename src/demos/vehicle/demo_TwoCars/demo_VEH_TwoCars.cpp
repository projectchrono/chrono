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

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

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
    sys.SetMaxItersSolverSpeed(150);
    sys.SetMaxItersSolverStab(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Create the terrain
    RigidTerrain terrain(&sys);
    auto patch = terrain.AddPatch(ChCoordsys<>(ChVector<>(0, 0, -5), QUNIT), ChVector<>(200, 100, 10));
    patch->SetContactFrictionCoefficient(0.9f);
    patch->SetContactRestitutionCoefficient(0.01f);
    patch->SetContactMaterialProperties(2e7f, 0.3f);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create and initialize the first vehicle
    HMMWV_Reduced hmmwv_1(&sys);
    hmmwv_1.SetInitPosition(ChCoordsys<>(ChVector<>(0, -1.5, 1.0), QUNIT));
    hmmwv_1.SetPowertrainType(PowertrainModelType::SIMPLE);
    hmmwv_1.SetDriveType(DrivelineType::RWD);
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
    hmmwv_2.SetDriveType(DrivelineType::RWD);
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
    ChWheeledVehicleIrrApp app(&hmmwv_1.GetVehicle(), &hmmwv_1.GetPowertrain(), L"Two cars demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
    app.SetChaseCameraState(utils::ChChaseCamera::Track);
    app.SetChaseCameraPosition(ChVector<>(-15, 0, 2.0));
    app.SetTimestep(step_size);

    // Bind all Irrlicht visualization assets
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------

    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        double time = sys.GetChTime();

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Collect output data from modules (for inter-module communication)
        double throttle_input_1 = driver_1.GetThrottle();
        double steering_input_1 = driver_1.GetSteering();
        double braking_input_1 = driver_1.GetBraking();

        double throttle_input_2 = driver_2.GetThrottle();
        double steering_input_2 = driver_2.GetSteering();
        double braking_input_2 = driver_2.GetBraking();

        // Update modules (process inputs from other modules)
        driver_1.Synchronize(time);
        driver_2.Synchronize(time);
        hmmwv_1.Synchronize(time, steering_input_1, braking_input_1, throttle_input_1, terrain);
        hmmwv_2.Synchronize(time, steering_input_2, braking_input_2, throttle_input_2, terrain);
        terrain.Synchronize(time);
        app.Synchronize("", steering_input_1, throttle_input_1, braking_input_1);

        // Advance simulation for one timestep for all modules.
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver_1.Advance(step);
        driver_2.Advance(step);
        hmmwv_1.Advance(step);
        hmmwv_2.Advance(step);
        terrain.Advance(step);
        app.Advance(step);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step);

        app.EndScene();
    }

    return 0;
}
