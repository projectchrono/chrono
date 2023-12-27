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

#include "wheeled_vehicle_complete_FMU.h"

using namespace chrono;
using namespace chrono::vehicle;

FmuComponent::FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID)
    : FmuChronoComponentBase(_instanceName, _fmuType, _fmuGUID) {
    // Initialize FMU type
    initializeType(_fmuType);

    // Set initial values for FMU input variables
    driver_inputs = {0, 0, 0, 0};
    terrain_height = {0.0, 0.0, 0.0, 0.0};
    terrain_normal = {ChVector<>(0, 0, 1), ChVector<>(0, 0, 1), ChVector<>(0, 0, 1), ChVector<>(0, 0, 1)};

    // Set configuration flags for this FMU
    AddFmuVariable(&vis, "vis", FmuVariable::Type::Boolean, "1", "enable visualization",         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&vehicle_JSON, "vehicle_JSON", FmuVariable::Type::String, "1", "vehicle JSON",                 //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&tire_JSON, "tire_JSON", FmuVariable::Type::String, "1", "tire JSON",                          //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&engine_JSON, "engine_JSON", FmuVariable::Type::String, "1", "engine JSON",                    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&transmission_JSON, "transmission_JSON", FmuVariable::Type::String, "1", "transmission JSON",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //

    AddFmuVariable(&system_SMC, "system_SMC", FmuVariable::Type::Boolean, "1", "use SMC system",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);   //

    AddFmuVariable(&init_loc.x(), "init_loc_x", FmuVariable::Type::Real, "m", "initial location X",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);      //
    AddFmuVariable(&init_loc.y(), "init_loc_y", FmuVariable::Type::Real, "m", "initial location Y",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);      //
    AddFmuVariable(&init_loc.z(), "init_loc_z", FmuVariable::Type::Real, "m", "initial location Z",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);      //
    AddFmuVariable(&init_yaw, "init_yaw", FmuVariable::Type::Real, "rad", "initial location Z",      //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);      //

    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set CONTINOUS INPUTS for this FMU
    AddFmuVariable(&driver_inputs.m_steering, "steering", FmuVariable::Type::Real, "1", "steering input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_throttle, "throttle", FmuVariable::Type::Real, "1", "throttle input",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_braking, "braking", FmuVariable::Type::Real, "1", "braking input",     //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //
    AddFmuVariable(&driver_inputs.m_clutch, "clutch", FmuVariable::Type::Real, "1", "clutch input",        //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);           //

    AddFmuVariable(&terrain_height[0], "height_FL", FmuVariable::Type::Real, "m", "terrain height FL",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);        //
    AddFmuVariable(&terrain_height[1], "height_FR", FmuVariable::Type::Real, "m", "terrain height FR",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);        //
    AddFmuVariable(&terrain_height[2], "height_RL", FmuVariable::Type::Real, "m", "terrain height RL",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);        //
    AddFmuVariable(&terrain_height[3], "height_RR", FmuVariable::Type::Real, "m", "terrain height RR",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);        //

    //// TODO terrain normals

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

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(tire_JSON);
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }
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
    //// TODO
    ////vehicle->Synchronize(time, driver_inputs, terrain);
    if (vis) {
#ifdef CHRONO_IRRLICHT
        vissys->Synchronize(time, driver_inputs);
#endif
    }
}

void FmuComponent::CalculateVehicleOutputs() {
    //// TODO
}

void FmuComponent::_preModelDescriptionExport() {}

void FmuComponent::_postModelDescriptionExport() {}

void FmuComponent::_enterInitializationMode() {}

void FmuComponent::_exitInitializationMode() {
    // Create the vehicle system
    CreateVehicle();

    // Configure Chrono system
    ////ConfigureSystem();

    // Initialize runtime visualization (if requested and if available)
    if (vis) {
#ifdef CHRONO_IRRLICHT
        sendToLog("Enable run-time visualization", fmi2Status::fmi2OK, "logAll");

        vissys = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        vissys->SetWindowTitle("Wheeled Vehicle FMU");
        vissys->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
        vissys->Initialize();
        vissys->AddLightDirectional();
        vissys->AttachVehicle(vehicle.get());
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
            vissys->Run();
            vissys->BeginScene(true, true, ChColor(0.33f, 0.6f, 0.78f));
            vissys->Render();
            vissys->EndScene();

            vissys->Advance(h);
#endif
        }

        ////sendToLog("time: " + std::to_string(time) + "\n", fmi2Status::fmi2OK, "logAll");

        time = time + h;
    }

    return fmi2Status::fmi2OK;
}
