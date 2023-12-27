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

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/serialization/ChArchive.h"

#include "actuatorFMU.h"

using namespace chrono;

FmuComponent::FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID)
    : FmuChronoComponentBase(_instanceName, _fmuType, _fmuGUID) {
    // Initialize FMU type
    initializeType(_fmuType);

    // Set initial values for FMU input variables
    init_F = 0;
    s = 0;
    sd = 0;
    Uref = 0;

    // Set CONTINOUS INPUTS and OUTPUTS for this FMU
    AddFmuVariable(&init_F, "init_F", FmuVariable::Type::Real, "N", "initial load",                //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //
    AddFmuVariable(&s, "s", FmuVariable::Type::Real, "m", "actuator length",                       //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //
    AddFmuVariable(&sd, "sd", FmuVariable::Type::Real, "m/s", "actuator length rate",              //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //
    AddFmuVariable(&F, "F", FmuVariable::Type::Real, "N", "actuator force",                        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&Uref, "Uref", FmuVariable::Type::Real, "1", "input signal",                    //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //

    // Set additional CONTINUOUS OUTPUTS from this FMU
    AddFmuVariable(&p1, "p1", FmuVariable::Type::Real, "N/m2", "piston pressure 1",                //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&p2, "p2", FmuVariable::Type::Real, "N/m2", "piston pressure 2",                //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&U, "U", FmuVariable::Type::Real, "1", "valve position",                        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Set gravitational acceleration
    ChVector<> Gacc(0, 0, -9.8);
    sys.Set_G_acc(Gacc);

    // Create the actuation object
    m_actuation = chrono_types::make_shared<ChFunction_Setpoint>();

    // Construct the hydraulic actuator
    m_actuator = chrono_types::make_shared<ChHydraulicActuator2>();
    m_actuator->SetInputFunction(m_actuation);
    m_actuator->Cylinder().SetInitialChamberLengths(0.221, 0.221);
    m_actuator->Cylinder().SetInitialChamberPressures(3.3e6, 4.4e6);
    m_actuator->DirectionalValve().SetInitialSpoolPosition(0);
    sys.Add(m_actuator);

    // Specify functions to process input variables (at beginning of step)
    preStepCallbacks.push_back([this]() { this->m_actuator->SetActuatorLength(s, sd); });
    preStepCallbacks.push_back([this]() { this->m_actuation->SetSetpoint(Uref, this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    postStepCallbacks.push_back([this]() { this->CalculateActuatorForce(); });
    postStepCallbacks.push_back([this]() { this->CalculatePistonPressures(); });
    postStepCallbacks.push_back([this]() { this->CalculateValvePosition(); });
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

void FmuComponent::_preModelDescriptionExport() {
    _exitInitializationMode();
    ////ChArchiveFmu archive_fmu(*this);
    ////archive_fmu << CHNVP(sys);
}

void FmuComponent::_postModelDescriptionExport() {}

void FmuComponent::_enterInitializationMode() {}

void FmuComponent::_exitInitializationMode() {
    // Complete construction of the hydraulic actuator (must have s and init_F)
    m_actuator->SetActuatorInitialLength(s);
    m_actuator->SetInitialLoad(init_F);
    m_actuator->Initialize();

    // Initialize FMU outputs (in case they are queried before the first step)
    CalculateActuatorForce();
    CalculatePistonPressures();
    CalculateValvePosition();

    sys.DoFullAssembly();
}

fmi2Status FmuComponent::_doStep(fmi2Real currentCommunicationPoint,
                                 fmi2Real communicationStepSize,
                                 fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real _stepSize = std::min((currentCommunicationPoint + communicationStepSize - time),
                                      std::min(communicationStepSize, stepSize));

        sys.DoStepDynamics(_stepSize);
        sendToLog("time: " + std::to_string(time) + "\n", fmi2Status::fmi2OK, "logAll");

        time = time + _stepSize;
    }

    return fmi2Status::fmi2OK;
}
