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
// Co-simulation FMU encapsulating a powertrain system.
//
// =============================================================================

#include <cassert>
#include <algorithm>

#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "FMU_Powertrain.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

// Create an instance of this FMU
fmu_tools::fmi2::FmuComponentBase* fmu_tools::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
                                                                        fmi2Type fmuType,
                                                                        fmi2String fmuGUID,
                                                                        fmi2String fmuResourceLocation,
                                                                        const fmi2CallbackFunctions* functions,
                                                                        fmi2Boolean visible,
                                                                        fmi2Boolean loggingOn) {
    return new FmuComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}

// -----------------------------------------------------------------------------

FmuComponent::FmuComponent(fmi2String instanceName,
                           fmi2Type fmuType,
                           fmi2String fmuGUID,
                           fmi2String fmuResourceLocation,
                           const fmi2CallbackFunctions* functions,
                           fmi2Boolean visible,
                           fmi2Boolean loggingOn)
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    throttle = 0;
    clutch = 0;

    driveshaft_speed = 0;
    driveshaft_torque = 0;
    engine_reaction = 0;
    transmission_reaction = 0;

    step_size = 1e-3;

    out_path = ".";

    // Get default JSON file from FMU resources
    auto resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    engine_JSON = resources_dir + "/EngineShafts.json";
    transmission_JSON = resources_dir + "/AutomaticTransmissionShafts.json";

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&engine_JSON, "engine_JSON", FmuVariable::Type::String, "1", "engine JSON",                    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //
    AddFmuVariable(&transmission_JSON, "transmission_JSON", FmuVariable::Type::String, "1", "transmission JSON",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);                   //

    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set FIXED PARAMETERS for this FMU (I/O)
    AddFmuVariable(&out_path, "out_path", FmuVariable::Type::String, "1", "output directory",    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set CONTINOUS INPUTS for this FMU (driver inputs)
    AddFmuVariable(&throttle, "throttle", FmuVariable::Type::Real, "1", "throttle input",         //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&clutch, "clutch", FmuVariable::Type::Real, "1", "clutch input",               //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //

    // Set CONTINUOUS OUTPUTS for this FMU (reaction torques)
    AddFmuVariable(&engine_reaction, "engine_reaction", FmuVariable::Type::Real,                  //
                   "Nm", "engine reaction torque",                                                //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&transmission_reaction, "transmission_reaction", FmuVariable::Type::Real,      //
                   "Nm", "transmission reaction torque",                                          //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //

    // Set coupling CONTINUOUS INPUTS AND OUTPUTS for this FMU (vehicle side)
    AddFmuVariable(&driveshaft_speed, "driveshaft_speed", FmuVariable::Type::Real,                //
                   "rad/s", "driveshaft angular speed",                                           //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&driveshaft_torque, "driveshaft_torque", FmuVariable::Type::Real,              //
                   "Nm", "driveshaft motor torque",                                               //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->SynchronizePowertrain(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculatePowertrainOutputs(); });
}

class Chassis : public ChChassis {
  public:
    Chassis() : ChChassis("chassis") {}
    virtual std::string GetTemplateName() const override { return ""; }
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return ChCoordsysd(); }
    virtual void EnableCollision(bool state) override {}

    virtual double GetBodyMass() const override { return 1; }
    virtual ChFrame<> GetBodyCOMFrame() const override { return ChFramed(); }
    virtual ChMatrix33<> GetBodyInertia() const override { return ChMatrix33<>(1); }
};

void FmuComponent::CreatePowertrain() {
    std::cout << "Create powertrain FMU" << std::endl;
    std::cout << " Engine JSON: " << engine_JSON << std::endl;
    std::cout << " Transmission JSON: " << transmission_JSON << std::endl;

    engine = ReadEngineJSON(engine_JSON);
    transmission = ReadTransmissionJSON(transmission_JSON);
    powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);

    // Create a placeholder chassis (fixed)
    auto chassis = chrono_types::make_shared<Chassis>();
    chassis->Initialize(&sys, ChCoordsysd(), 0.0);
    chassis->SetFixed(true);

    powertrain->Initialize(chassis);
}

void FmuComponent::SynchronizePowertrain(double time) {
    // Synchronize the powertrain assembly given current driver inputs and driveshaft speed
    DriverInputs driver_inputs;
    driver_inputs.m_braking = 0;
    driver_inputs.m_clutch = clutch;
    driver_inputs.m_steering = 0;
    driver_inputs.m_throttle = throttle;

    powertrain->Synchronize(time, driver_inputs, driveshaft_speed);
}

void FmuComponent::CalculatePowertrainOutputs() {
    // Output torque to a driveline
    driveshaft_torque = transmission->GetOutputDriveshaftTorque();

    // Reaction torques on a chassis
    engine_reaction = engine->GetChassisReactionTorque();
    transmission_reaction = transmission->GetChassisReactionTorque();
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    CreatePowertrain();
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint,
                                    fmi2Real communicationStepSize,
                                    fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                              std::min(communicationStepSize, step_size));

        powertrain->Advance(h);

        ////sendToLog("time: " + std::to_string(m_time) + "\n", fmi2Status::fmi2OK, "logAll");

        m_time += h;
    }

    return fmi2Status::fmi2OK;
}
