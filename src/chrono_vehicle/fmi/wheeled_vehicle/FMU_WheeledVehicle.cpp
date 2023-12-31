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

    // Set initial values for FMU input variables
    driver_inputs = {0, 0, 0, 0};

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

    // Set CONTINOUS INPUTS and OUTPUTS for this FMU (wheel state and forces for co-simulation)
    for (int iw = 0; iw < 4; iw++) {
        std::string prefix = "wheel_" + wheel_data[iw].identifier;

        AddFmuVecVariable(wheel_data[iw].load.point, prefix + "_point", "m", prefix + " application point",  //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);      //
        AddFmuVecVariable(wheel_data[iw].load.force, prefix + "_frc", "N", prefix + " applied force",        //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);      //
        AddFmuVecVariable(wheel_data[iw].load.moment, prefix + "_trq", "Nm", prefix + " applied torque",     //
                          FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);      //

        AddFmuVecVariable(wheel_data[iw].state.pos, prefix + "_pos", "m", prefix + " position",                  //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);         //
        AddFmuQuatVariable(wheel_data[iw].state.rot, prefix + "_rot", "1", prefix + " rotation",                 //
                           FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);        //
        AddFmuVecVariable(wheel_data[iw].state.lin_vel, prefix + "_vel", "m/s", prefix + " linear velocity",     //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);         //
        AddFmuVecVariable(wheel_data[iw].state.ang_vel, prefix + "_omg", "rad/s", prefix + " angular velocity",  //
                          FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);         //
    }

    // Specify functions to process input variables (at beginning of step)
    preStepCallbacks.push_back([this]() { this->SynchronizeVehicle(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    postStepCallbacks.push_back([this]() { this->CalculateVehicleOutputs(); });
}

void FmuComponent::CreateVehicle() {
    // Create the vehicle system
    vehicle = chrono_types::make_shared<WheeledVehicle>(vehicle_JSON,
                                                        system_SMC ? ChContactMethod::SMC : ChContactMethod::NSC);
    vehicle->Initialize(ChCoordsys<>(init_loc, Q_from_AngZ(init_yaw)));
    vehicle->GetChassis()->SetFixed(false);

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
    for (int iw = 0; iw < 4; iw++) {
        wheel_data[iw].state = wheel_data[iw].wheel->GetState();
    }

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
        sendToLog("Enable run-time visualization", fmi2Status::fmi2OK, "logAll");

        vis_sys = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        vis_sys->SetWindowTitle("Wheeled Vehicle FMU");
        vis_sys->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
        vis_sys->Initialize();
        vis_sys->AddLightDirectional();
        vis_sys->AttachVehicle(vehicle.get());
#else
        sendToLog("Run-time visualization not available", fmi2Status::fmi2OK, "logAll");
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
            vis_sys->Run();
            vis_sys->BeginScene(true, true, ChColor(0.33f, 0.6f, 0.78f));
            vis_sys->Render();
            vis_sys->EndScene();

            vis_sys->Advance(h);
#endif
        }

        ////sendToLog("time: " + std::to_string(time) + "\n", fmi2Status::fmi2OK, "logAll");

        time = time + h;
    }

    return fmi2Status::fmi2OK;
}
