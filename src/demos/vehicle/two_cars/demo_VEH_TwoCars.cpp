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

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

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

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

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
    driver_data_2.push_back({0.0, 0, 0.0, 0});
    driver_data_2.push_back({0.5, 0, 0.0, 0});
    driver_data_2.push_back({0.7, -0.3, 0.7, 0});
    driver_data_2.push_back({1.0, -0.3, 0.7, 0});
    driver_data_2.push_back({3.0, -0.5, 0.1, 0});
    ChDataDriver driver_2(hmmwv_2.GetVehicle(), driver_data_2);
    driver_2.Initialize();

    // Create the vehicle run-time visualization interface and the interactive driver
    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Two cars demo");
            vis_irr->SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
            vis_irr->SetChaseCameraState(utils::ChChaseCamera::Track);
            vis_irr->SetChaseCameraPosition(ChVector<>(-15, 0, 2.0));
            vis_irr->Initialize();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AddLightDirectional();
            vis_irr->AttachVehicle(&hmmwv_1.GetVehicle());

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Two cars demo");
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 300));
            vis_vsg->SetChaseCamera(ChVector<>(0.0, 0.0, .75), 6.0, 0.5);
            vis_vsg->SetChaseCameraState(utils::ChChaseCamera::Track);
            vis_vsg->SetChaseCameraPosition(ChVector<>(-15, 0, 2.0));
            vis_vsg->AttachVehicle(&hmmwv_1.GetVehicle());
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

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
        vis->Synchronize(time, driver_inputs_1);

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
