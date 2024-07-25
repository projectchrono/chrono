// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Example FMU for model exchange (FMI 2.0 standard)
// Implements the Van der Pol ODE:
//   x'' = mu * (1 - x^2) * x' - x + u(t)
//   x(0) = 2
//   x'(0) = 0
// =============================================================================

#include "vdpFMU.h"

using namespace fmu_tools::fmi2;

// -----------------------------------------------------------------------------

// Create an instance of this FMU
FmuComponentBase* fmu_tools::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
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
    : FmuComponentBase(
          instanceName,
          fmuType,
          fmuGUID,
          fmuResourceLocation,
          functions,
          visible,
          loggingOn,
          {{"logEvents", true},
           {"logSingularLinearSystems", true},
           {"logNonlinearSystems", true},
           {"logStatusWarning", true},
           {"logStatusError", true},
           {"logStatusPending", true},
           {"logDynamicStateSelection", true},
           {"logStatusDiscard", true},
           {"logStatusFatal", true},
           {"logAll", true}},
          {"logStatusWarning", "logStatusDiscard", "logStatusError", "logStatusFatal", "logStatusPending"}) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial conditions for underlying ODE
    q = {2.0, 0.0};

    // Default parameter value
    mu = 1.0;

    // Default forcing term value
    u = 0;

    // Declare relevant variables
    AddFmuVariable(&mu, "mu", FmuVariable::Type::Real, "1", "VDP parameter",                     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&q[0], "x", FmuVariable::Type::Real, "1", "state",                             //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&q[1], "der(x)", FmuVariable::Type::Real, "1", "1st state derivative",         //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,   //
                   FmuVariable::InitialType::calculated);                                         //

    AddFmuVariable(&q[1], "v", FmuVariable::Type::Real, "1", "derivative",                        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&a, "der(v)", FmuVariable::Type::Real, "1", "2nd state derivative",            //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,   //
                   FmuVariable::InitialType::calculated);                                         //

    AddFmuVariable(&u, "u", FmuVariable::Type::Real, "1", "forcing term",                        //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::none);                                              //

    // Specify state derivatives
    DeclareStateDerivative("der(x)", "x", {"v"});
    DeclareStateDerivative("der(v)", "v", {"x", "v", "mu"});

    // Variable dependencies must be specified for:
    // - variables with causality 'output' for which 'initial' is 'approx' or 'calculated'
    // - variables with causality 'calculatedParameter'
    DeclareVariableDependencies("der(x)", {"v"});
    DeclareVariableDependencies("der(v)", {"x", "v", "mu"});

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->calcAcceleration(); });
}

// -----------------------------------------------------------------------------

// A function added as a post-step callback can be used to prepare (post-process)
// other output or local FMI variables (here, the cart and pendulum accelerations)
void FmuComponent::calcAcceleration() {
    a = mu * (1 - q[0] * q[0]) * q[1] - q[0];
}

// -----------------------------------------------------------------------------

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::getContinuousStatesIMPL(fmi2Real x[], size_t nx) {
    for (size_t i = 0; i < nx; i++) {
        x[i] = q[i];
    }

    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::setContinuousStatesIMPL(const fmi2Real x[], size_t nx) {
    for (size_t i = 0; i < nx; i++) {
        q[i] = x[i];
    }

    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::getDerivativesIMPL(fmi2Real derivatives[], size_t nx) {
    derivatives[0] = q[1];
    derivatives[1] = mu * (1 - q[0] * q[0]) * q[1] - q[0] + u;

    return fmi2Status::fmi2OK;
}
