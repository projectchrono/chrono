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

#include <cassert>
#include <map>
#include <algorithm>

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/serialization/ChArchive.h"

#include "actuatorFMU.h"

using namespace chrono;
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
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn), have_s0(false) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set start values for FMU input and output variables
    init_F = 0;
    s = 0;
    sd = 0;
    Uref = 0;
    F = 0;

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&init_F, "init_F", FmuVariable::Type::Real, "N", "initial load",              //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set CONTINOUS INPUTS for this FMU
    AddFmuVariable(&s, "s", FmuVariable::Type::Real, "m", "actuator length",                       //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //
    AddFmuVariable(&sd, "sd", FmuVariable::Type::Real, "m/s", "actuator length rate",              //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //
    AddFmuVariable(&Uref, "Uref", FmuVariable::Type::Real, "1", "input signal",                    //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //

    // Set CONTINUOUS OUTPUTS from this FMU
    AddFmuVariable(&F, "F", FmuVariable::Type::Real, "N", "actuator force",                        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&p1, "p1", FmuVariable::Type::Real, "N/m2", "piston pressure 1",                //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&p2, "p2", FmuVariable::Type::Real, "N/m2", "piston pressure 2",                //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Set CONTINUOUS LOCALS for this FMU
    AddFmuVariable(&U, "U", FmuVariable::Type::Real, "1", "valve position",                       //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Specify variable dependencies
    DeclareVariableDependencies("F", {"s", "sd", "Uref"});
    DeclareVariableDependencies("p1", {"s", "sd", "Uref"});
    DeclareVariableDependencies("p2", {"s", "sd", "Uref"});
    DeclareVariableDependencies("U", {"Uref"});

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->m_actuator->SetActuatorLength(s, sd); });
    AddPreStepFunction([this]() { this->m_actuation->SetSetpoint(Uref, this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateActuatorForce(); });
    AddPostStepFunction([this]() { this->CalculatePistonPressures(); });
    AddPostStepFunction([this]() { this->CalculateValvePosition(); });
}

void FmuComponent::CalculateActuatorForce() {
    F = m_actuator->GetActuatorForce();
}

void FmuComponent::CalculatePistonPressures() {
    auto p = m_actuator->GetCylinderPressures();
    p1 = p[0];
    p2 = p[1];
}

void FmuComponent::CalculateValvePosition() {
    U = m_actuator->GetValvePosition();
}

void FmuComponent::preModelDescriptionExport() {
    exitInitializationModeIMPL();
}

void FmuComponent::postModelDescriptionExport() {}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    // 1. Construct hydraulic actuator (must have parameters)

    // Set gravitational acceleration
    ChVector3d Gacc(0, 0, -9.8);
    sys.SetGravitationalAcceleration(Gacc);

    // Create the actuation object
    m_actuation = chrono_types::make_shared<ChFunctionSetpoint>();

    // Construct the hydraulic actuator
    m_actuator = chrono_types::make_shared<ChHydraulicActuator2>();
    m_actuator->SetInputFunction(m_actuation);
    m_actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
    m_actuator->Cylinder().SetInitialChamberPressures(3.3e6, 4.4e6);
    m_actuator->DirectionalValve().SetInitialSpoolPosition(0);
    sys.Add(m_actuator);

    // 2. Complete construction of the hydraulic actuator (must have s and init_F)
    m_actuator->SetActuatorInitialLength(s);
    m_actuator->SetInitialLoad(init_F);
    m_actuator->Initialize();

    // 3. Initialize FMU outputs (in case they are queried before the first step)
    CalculateActuatorForce();
    CalculatePistonPressures();
    CalculateValvePosition();

    sys.DoAssembly(AssemblyAnalysis::Level::FULL);

    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint,
                                    fmi2Real communicationStepSize,
                                    fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    // Set initial actuator length at t=0
    if (!have_s0) {
        std::cout << "Setting s0 = " << s << std::endl;
        m_actuator->SetActuatorInitialLength(s);
        have_s0 = true;
    }

    // Advance FMU state to next communication point
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real step_size = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                                      std::min(communicationStepSize, m_stepSize));

        sys.DoStepDynamics(step_size);
        ////sendToLog("time: " + std::to_string(m_time) + "\n", fmi2Status::fmi2OK, "logAll");

        m_time += step_size;
    }

    return fmi2Status::fmi2OK;
}
