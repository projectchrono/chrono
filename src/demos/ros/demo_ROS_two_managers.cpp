// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Harry Zhang, Aaron Young, Radu Serban
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

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::ros;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Simulation step sizes
double step_size = 0.005;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------
    // Create systems
    // --------------

    // Chrono system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // Create the terrain
    RigidTerrain terrain(&sys);
    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 100);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // define ROS handelers' rate
    auto driver_inputs_rate = 25;
    auto vehicle_state_rate = 25;
    // define ROS topics' name for driver inputs and vehicle state
    auto driver_inputs_topic_name = "~/input/driver_inputs";
    auto vehicle_state_topic_name = "~/output/vehicle/state";

    // Create and initialize the first vehicle
    HMMWV_Reduced hmmwv_1(&sys);
    hmmwv_1.SetInitPosition(ChCoordsys<>(ChVector3d(0, -1.5, 1.0), QUNIT));
    hmmwv_1.SetEngineType(EngineModelType::SIMPLE);
    hmmwv_1.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv_1.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv_1.SetTireType(TireModelType::RIGID);
    hmmwv_1.Initialize();
    hmmwv_1.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_1.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_1.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the basic driver for the first vehicle
    auto driver_1 = std::make_shared<ChDriver>(hmmwv_1.GetVehicle());
    // Create ROS manager for vehicle one
    auto ros_manager_1 = chrono_types::make_shared<ChROSManager>("hmmwv_1");
    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler_1 = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager_1->RegisterHandler(clock_handler_1);
    // Create a subscriber to the driver inputs
    auto driver_inputs_handler_1 =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver_1, driver_inputs_topic_name);
    ros_manager_1->RegisterHandler(driver_inputs_handler_1);
    // Create a publisher for the vehicle state
    auto vehicle_state_handler_1 = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, hmmwv_1.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_1->RegisterHandler(vehicle_state_handler_1);
    // Finally, initialize the ros manager
    ros_manager_1->Initialize();


    // Create and initialize the second vehicle
    HMMWV_Reduced hmmwv_2(&sys);
    hmmwv_2.SetInitPosition(ChCoordsys<>(ChVector3d(7, 1.5, 1.0), QUNIT));
    hmmwv_2.SetEngineType(EngineModelType::SIMPLE);
    hmmwv_2.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    hmmwv_2.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv_2.SetTireType(TireModelType::RIGID);
    hmmwv_2.Initialize();
    hmmwv_2.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv_2.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv_2.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // Create the basic driver for the first vehicle
    auto driver_2 = std::make_shared<ChDriver>(hmmwv_2.GetVehicle());
    // Create ROS manager for vehicle one
    auto ros_manager_2 = chrono_types::make_shared<ChROSManager>("hmmwv_2");
    // Create a subscriber to the driver inputs
    auto driver_inputs_handler_2 =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver_2, driver_inputs_topic_name);
    ros_manager_2->RegisterHandler(driver_inputs_handler_2);
    // Create a publisher for the vehicle state
    auto vehicle_state_handler_2 = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, hmmwv_2.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_2->RegisterHandler(vehicle_state_handler_2);
    // Finally, initialize the ros manager
    ros_manager_2->Initialize();

// Create the vehicle run-time visualization interface and the interactive driver
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
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Two cars demo");
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
            vis_irr->SetChaseCameraState(utils::ChChaseCamera::Track);
            vis_irr->SetChaseCameraPosition(ChVector3d(-15, 0, 2.0));
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
            vis_vsg->SetWindowSize(ChVector2i(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
            vis_vsg->SetChaseCameraState(utils::ChChaseCamera::Track);
            vis_vsg->SetChaseCameraPosition(ChVector3d(-15, 0, 2.0));
            vis_vsg->AttachVehicle(&hmmwv_1.GetVehicle());
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    hmmwv_1.GetVehicle().EnableRealtime(true);
    hmmwv_2.GetVehicle().EnableRealtime(true);

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Render scene
        vis->BeginScene();
        vis->Render();

        // Driver inputs
        DriverInputs driver_inputs_1 = driver_1->GetInputs();
        DriverInputs driver_inputs_2 = driver_2->GetInputs();

        // Update modules (process inputs from other modules)
        driver_1->Synchronize(time);
        driver_2->Synchronize(time);
        hmmwv_1.Synchronize(time, driver_inputs_1, terrain);
        hmmwv_2.Synchronize(time, driver_inputs_2, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs_1);

        // Advance simulation for one timestep for all modules.
        driver_1->Advance(step_size);
        driver_2->Advance(step_size);
        hmmwv_1.Advance(step_size);
        hmmwv_2.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);

        // Update ROS managers
        if (!ros_manager_1->Update(time, step_size) || !ros_manager_2->Update(time, step_size))
            break;

        vis->EndScene();
    }

    return 0;
}