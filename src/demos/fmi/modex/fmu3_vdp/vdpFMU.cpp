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
// Example FMU for model exchange (FMI 3.0 standard)
// Implements the Van der Pol ODE:
//   x'' = mu * (1 - x^2) * x' - x + u(t)
//   x(0) = 2
//   x'(0) = 0
// =============================================================================

#include "vdpFMU.h"

using namespace fmu_tools::fmi3;

// -----------------------------------------------------------------------------

// Create an instance of this FMU
FmuComponentBase* fmu_tools::fmi3::fmi3InstantiateIMPL(FmuType fmiInterfaceType,
                                                       fmi3String instanceName,
                                                       fmi3String instantiationToken,
                                                       fmi3String resourcePath,
                                                       fmi3Boolean visible,
                                                       fmi3Boolean loggingOn,
                                                       fmi3InstanceEnvironment instanceEnvironment,
                                                       fmi3LogMessageCallback logMessage) {
    return new FmuComponent(fmiInterfaceType, instanceName, instantiationToken, resourcePath, visible, loggingOn,
                            instanceEnvironment, logMessage);
}

// -----------------------------------------------------------------------------

FmuComponent::FmuComponent(FmuType fmiInterfaceType,
                           fmi3String instanceName,
                           fmi3String instantiationToken,
                           fmi3String resourcePath,
                           fmi3Boolean visible,
                           fmi3Boolean loggingOn,
                           fmi3InstanceEnvironment instanceEnvironment,
                           fmi3LogMessageCallback logMessage)
    : FmuComponentBase(
          fmiInterfaceType,
          instanceName,
          instantiationToken,
          resourcePath,
          visible,
          loggingOn,
          instanceEnvironment,
          logMessage,
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
    initializeType(fmiInterfaceType);

    // Set initial conditions for underlying ODE
    q = {2.0, 0.0};

    // Default parameter value
    mu = 1.0;

    // Default forcing term value
    u = 0;

    // Declare relevant variables
    AddFmuVariable(&mu, "mu", FmuVariable::Type::Float64, "1", "VDP parameter",                  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&q[0], "x", FmuVariable::Type::Float64, "1", "state",                          //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&q[1], "der(x)", FmuVariable::Type::Float64, "1", "1st state derivative",      //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,   //
                   FmuVariable::InitialType::calculated);                                         //

    AddFmuVariable(&q[1], "v", FmuVariable::Type::Float64, "1", "derivative",                     //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&a, "der(v)", FmuVariable::Type::Float64, "1", "2nd state derivative",         //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,   //
                   FmuVariable::InitialType::calculated);                                         //

    AddFmuVariable(&u, "u", FmuVariable::Type::Float64, "1", "forcing term",                     //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::automatic);                                         //

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

fmi3Status FmuComponent::enterInitializationModeIMPL() {
    return fmi3Status::fmi3OK;
}

fmi3Status FmuComponent::exitInitializationModeIMPL() {
    return fmi3Status::fmi3OK;
}

fmi3Status FmuComponent::getContinuousStatesIMPL(fmi3Float64 continuousStates[], size_t nContinuousStates) {
    for (size_t i = 0; i < nContinuousStates; i++) {
        continuousStates[i] = q[i];
    }

    return fmi3Status::fmi3OK;
}

fmi3Status FmuComponent::setContinuousStatesIMPL(const fmi3Float64 continuousStates[], size_t nContinuousStates) {
    for (size_t i = 0; i < nContinuousStates; i++) {
        q[i] = continuousStates[i];
    }

    return fmi3Status::fmi3OK;
}

fmi3Status FmuComponent::getDerivativesIMPL(fmi3Float64 derivatives[], size_t nContinuousStates) {
    derivatives[0] = q[1];
    derivatives[1] = mu * (1 - q[0] * q[0]) * q[1] - q[0] + u;

    return fmi3Status::fmi3OK;
}
