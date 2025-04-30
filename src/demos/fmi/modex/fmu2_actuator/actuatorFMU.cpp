// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
fmu_forge::fmi2::FmuComponentBase* fmu_forge::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
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

    //// TODO - provide FMU variables to set these
    // Hydraulic actuator parameters
    hose1V = 3.14e-5;
    hose2V = 7.85e-5;
    Bo = 1500e6;
    Bh = 150e6;
    Bc = 31500e6;
    pP = 7.6e6;
    pT = 0.1e6;

    // Set start values for FMU input and output variables
    init_s = 0;
    init_F = 0;
    s = 0;
    sd = 0;
    Uref = 0;
    F = 0;

    //// TODO - should be set elsewhere (enterInitializationModeIMPL or exitInitializationModeIMPL)
    // Set initial conditions for underlying ODE
    q = {0.0, 4.4e6, 3.3e6};
    dvalve.SetInitialSpoolPosition(0);
    cyl.SetInitialChamberLengths(0.221, 0.221);
    cyl.SetInitialChamberPressures(4.4e6, 3.3e6);

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&init_s, "init_s", FmuVariable::Type::Real, "m", "initial length",            //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
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

    // Set states and derivatives

    AddFmuVariable(&q[0], "U", FmuVariable::Type::Real, "1", "valve position",                    //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&q[1], "p1", FmuVariable::Type::Real, "N/m2", "piston pressure 1",             //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&q[2], "p2", FmuVariable::Type::Real, "N/m2", "piston pressure 2",             //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //

    AddFmuVariable(&qd[0], "Ud", FmuVariable::Type::Real, "1", "valve position derivative",         //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,     //
                   FmuVariable::InitialType::calculated);                                           //
    AddFmuVariable(&qd[1], "p1d", FmuVariable::Type::Real, "N/m2", "piston pressure 1 derivative",  //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,     //
                   FmuVariable::InitialType::calculated);                                           //
    AddFmuVariable(&qd[2], "p2d", FmuVariable::Type::Real, "N/m2", "piston pressure 2 derivative",  //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,     //
                   FmuVariable::InitialType::calculated);                                           //

    // Specify state derivatives
    DeclareStateDerivative("Ud", "U", {"U", "Uref"});
    DeclareStateDerivative("p1d", "p1", {"U", "p1", "p2"});
    DeclareStateDerivative("p2d", "p2", {"U", "p1", "p2"});

    // Specify variable dependencies
    DeclareVariableDependencies("Ud", {"U", "Uref"});
    DeclareVariableDependencies("p1d", {"U", "p1", "p2"});
    DeclareVariableDependencies("p2d", {"U", "p1", "p2"});

    DeclareVariableDependencies("F", {"s", "sd", "Uref"});
    DeclareVariableDependencies("p1", {"s", "sd", "Uref"});
    DeclareVariableDependencies("p2", {"s", "sd", "Uref"});
    DeclareVariableDependencies("U", {"Uref"});

    // Specify functions to calculate FMU outputs (called after getDerivatives)
    AddPostStepFunction([this]() { this->calcForce(); });
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {}

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
    // Extract state
    double U = q(0);
    Vec2 p = q.segment(1, 2);

    //// TODO - time not needed here, but how do we get it in general?
    double time = 0;

    ////std::cout << "      t=" << time << "  U=" << U << "  Uref=" << Uref << std::endl;

    // Evaluate state derivatives
    auto Ud = dvalve.EvaluateSpoolPositionRate(time, U, Uref);
    auto pd = EvaluatePressureRates(time, p, U);

    // Load right-hand side
    derivatives[0] = Ud;
    derivatives[1] = pd(0);
    derivatives[2] = pd(1);

    return fmi2Status::fmi2OK;
}

Vec2 FmuComponent::EvaluatePressureRates(double t, const Vec2& p, double U) {
    const auto& Ac = cyl.GetAreas();
    auto Lc = cyl.ComputeChamberLengths(s - init_s);
    auto Vc = cyl.ComputeChamberVolumes(Lc);

    // Compute volumes
    Vec2 V(hose1V + Vc(0), hose2V + Vc(1));

    // Compute bulk modulus
    double ooBo = 1.0 / Bo;
    double ooBc = 1.0 / Bc;
    double ooBh = 1.0 / Bh;
    double ooBe1 = ooBo + (Vc(0) / V(0)) * ooBc + (hose1V / V(0)) * ooBh;
    double ooBe2 = ooBo + (Vc(1) / V(1)) * ooBc + (hose2V / V(1)) * ooBh;
    Vec2 Be(1.0 / ooBe1, 1.0 / ooBe2);

    // Compute volume flows
    auto Q = dvalve.ComputeVolumeFlows(U, p.segment(0, 2), pP, pT);

    // Compute pressure rates
    Vec2 pd;
    pd(0) = (Be(0) / V(0)) * (Q(0) - Ac(0) * sd);
    pd(1) = (Be(1) / V(1)) * (Ac(1) * sd - Q(1));
    return pd;
}

void FmuComponent::calcForce() {
    Vec2 p = q.segment(1, 2);
    F = cyl.EvalForce(p, s - init_s, sd);
}
