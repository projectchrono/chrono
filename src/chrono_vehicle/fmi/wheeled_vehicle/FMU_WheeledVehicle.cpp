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
// Authors: Radu Serban
// =============================================================================
//
// Co-simulation FMU encapsulating a wheeled vehicle system with 4 wheels.
// The vehicle includes a powertrain and transmission, but no tires.
//
// =============================================================================

// #define FMI2_FUNCTION_PREFIX MyModel_
#include <cassert>
#include <map>
#include <algorithm>

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "FMU_WheeledVehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

FmuComponent::FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID)
    : FmuChronoComponentBase(_instanceName, _fmuType, _fmuGUID) {
    // Initialize FMU type
    initializeType(_fmuType);

    // Set initial/default values for FMU variables
    g_acc = {0, 0, -9.8};
    driver_inputs = {0, 0, 0, 0};
    init_loc = {0, 0, 0};
    init_yaw = 0;

    system_SMC = 1;
    vis = 0;
    step_size = 1e-3;

    // Set wheel identifier strings
    wheel_data[0].identifier = "FL";
    wheel_data[1].identifier = "FR";
    wheel_data[2].identifier = "RL";
    wheel_data[3].identifier = "RR";

    // Set configuration flags for this FMU
    AddFmuVariable(&vis, "vis", FmuVariable::Type::Boolean, "1", "enable visualization",         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&vehicle_JSON, "vehicle_JSON", FmuVariable::Type::String, "1", "vehicle JSON",                 //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&engine_JSON, "engine_JSON", FmuVariable::Type::String, "1", "engine JSON",                    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&transmission_JSON, "transmission_JSON", FmuVariable::Type::String, "1", "transmission JSON",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //

    AddFmuVariable(&system_SMC, "system_SMC", FmuVariable::Type::Boolean, "1", "use SMC system",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVecVariable(init_loc, "init_loc", "m", "initial location",                                //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&init_yaw, "init_yaw", FmuVariable::Type::Real, "rad", "initial location Z",     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    AddFmuVecVariable(g_acc, "g_acc", "m/s2", "gravitational acceleration",                         //
                      FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set CONTINOUS INPUTS for this FMU (driver inputs)
    AddFmuVariable(&driver_inputs.m_steering, "steering", FmuVariable::Type::Real, "1", "steering input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_throttle, "throttle", FmuVariable::Type::Real, "1", "throttle input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_braking, "braking", FmuVariable::Type::Real, "1", "braking input",     //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_clutch, "clutch", FmuVariable::Type::Real, "1", "clutch input",        //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

    // Set CONTINUOUS OUTPUTS for this FMU (vehicle reference frame)
    AddFmuFrameMovingVariable(ref_frame, "ref_frame", "m", "m/s", "reference frame",                          //
                              FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Set CONTINOUS INPUTS and OUTPUTS for this FMU (wheel state and forces for co-simulation)
    for (int iw = 0; iw < 4; iw++) {
        std::string prefix = "wheel_" + wheel_data[iw].identifier;

        AddFmuVecVariable(wheel_data[iw].load.point, prefix + ".point", "m", prefix + " load application point",  //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
        AddFmuVecVariable(wheel_data[iw].load.force, prefix + ".force", "N", prefix + " load applied force",      //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
        AddFmuVecVariable(wheel_data[iw].load.moment, prefix + ".moment", "Nm", prefix + " load applied moment",  //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

        AddFmuVecVariable(wheel_data[iw].state.pos, prefix + ".pos", "m", prefix + " position",                      //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);             //
        AddFmuQuatVariable(wheel_data[iw].state.rot, prefix + ".rot", "1", prefix + " rotation",                     //
                           FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);            //
        AddFmuVecVariable(wheel_data[iw].state.lin_vel, prefix + ".lin_vel", "m/s", prefix + " linear velocity",     //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);             //
        AddFmuVecVariable(wheel_data[iw].state.ang_vel, prefix + ".ang_vel", "rad/s", prefix + " angular velocity",  //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);             //
    }

    // Specify functions to process input variables (at beginning of step)
    preStepCallbacks.push_back([this]() { this->SynchronizeVehicle(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    postStepCallbacks.push_back([this]() { this->CalculateVehicleOutputs(); });
}

void FmuComponent::CreateVehicle() {
    std::cout << "Create vehicle FMU" << std::endl;
    std::cout << " Vehicle JSON:      " << vehicle_JSON << std::endl;
    std::cout << " Engine JSON:       " << engine_JSON << std::endl;
    std::cout << " Transmission JSON: " << transmission_JSON << std::endl;
    std::cout << " Initial location:  " << init_loc << std::endl;
    std::cout << " Initial yaw:       " << init_yaw << std::endl;

    // Create the vehicle system
    vehicle = chrono_types::make_shared<WheeledVehicle>(vehicle_JSON,
                                                        system_SMC ? ChContactMethod::SMC : ChContactMethod::NSC);
    vehicle->Initialize(ChCoordsys<>(init_loc + ChVector<>(0, 0, 0.5), Q_from_AngZ(init_yaw)));
    ////vehicle->GetChassis()->SetFixed(true);
    ////std::cout << "\n\nATTENTION: vehicle chassis fixed to ground!\n\n" << std::endl;

    // Initialize the vehicle reference frame
    ref_frame = vehicle->GetRefFrame();

    // Cache vehicle wheels
    wheel_data[0].wheel = vehicle->GetWheel(0, VehicleSide::LEFT);
    wheel_data[1].wheel = vehicle->GetWheel(0, VehicleSide::RIGHT);
    wheel_data[2].wheel = vehicle->GetWheel(1, VehicleSide::LEFT);
    wheel_data[3].wheel = vehicle->GetWheel(1, VehicleSide::RIGHT);

    //// TODO: allow setting through FMU variables
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(engine_JSON);
    auto transmission = ReadTransmissionJSON(transmission_JSON);
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);
}

void FmuComponent::ConfigureSystem() {
    // Containing system
    auto system = vehicle->GetSystem();

    system->Set_G_acc(g_acc);

    // Associate a collision system
    system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Modify solver settings if the vehicle model contains bushings
    if (vehicle->HasBushings()) {
        auto solver = chrono_types::make_shared<ChSolverMINRES>();
        system->SetSolver(solver);
        solver->SetMaxIterations(150);
        solver->SetTolerance(1e-10);
        solver->EnableDiagonalPreconditioner(true);
        solver->EnableWarmStart(true);
        solver->SetVerbose(false);

        step_size = std::min(step_size, 2e-4);
        system->SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    }
}

void FmuComponent::SynchronizeVehicle(double time) {
    // Apply tire forces (received from outside) to wheel bodies
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].wheel->Synchronize(wheel_data[iw].load);
    }

    vehicle->Synchronize(time, driver_inputs);

    if (vis) {
#ifdef CHRONO_IRRLICHT
        vis_sys->Synchronize(time, driver_inputs);
#endif
    }
}

void FmuComponent::CalculateVehicleOutputs() {
    // Extract wheel states
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].state = wheel_data[iw].wheel->GetState();
    }

    // Update the vehicle reference frame
    ref_frame = vehicle->GetRefFrame();

    //// TODO - other vehicle outputs...
}

void FmuComponent::_preModelDescriptionExport() {}

void FmuComponent::_postModelDescriptionExport() {}

void FmuComponent::_enterInitializationMode() {}

void FmuComponent::_exitInitializationMode() {
    // Create the vehicle system
    CreateVehicle();

    // Configure Chrono system
    ConfigureSystem();

    // Initialize runtime visualization (if requested and if available)
    if (vis) {
#ifdef CHRONO_IRRLICHT
        std::cout << " Enable run-time visualization" << std::endl;

        vis_sys = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        vis_sys->SetLogLevel(irr::ELL_NONE);
        vis_sys->SetWindowTitle("Wheeled Vehicle FMU");
        vis_sys->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
        vis_sys->AddGrid(0.5, 0.5, 400, 400, ChCoordsys<>(init_loc, Q_from_AngZ(init_yaw)),
                         ChColor(0.31f, 0.43f, 0.43f));
        vis_sys->Initialize();
        vis_sys->AddLightDirectional();
        vis_sys->AttachVehicle(vehicle.get());
#else
        std::cout << " Run-time visualization not available" << std::endl;
#endif
    }
}

fmi2Status FmuComponent::_doStep(fmi2Real currentCommunicationPoint,
                                 fmi2Real communicationStepSize,
                                 fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - time),
                              std::min(communicationStepSize, step_size));
        vehicle->Advance(h);

        if (vis) {
#ifdef CHRONO_IRRLICHT
            auto status = vis_sys->Run();
            if (!status)
                return fmi2Discard;
            vis_sys->BeginScene(true, true, ChColor(0.33f, 0.6f, 0.78f));
            vis_sys->Render();
            vis_sys->RenderFrame(ref_frame);
            vis_sys->EndScene();

            vis_sys->Advance(h);
#endif
        }

        ////sendToLog("time: " + std::to_string(time) + "\n", fmi2Status::fmi2OK, "logAll");

        time = time + h;
    }

    return fmi2Status::fmi2OK;
}
