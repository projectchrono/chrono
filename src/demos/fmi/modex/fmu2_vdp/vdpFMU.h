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

#pragma once

#include <array>

#include "fmi2/FmuToolsExport.h"

class FmuComponent : public fmu_forge::fmi2::FmuComponentBase {
  public:
    FmuComponent(fmi2String instanceName,
                 fmi2Type fmuType,
                 fmi2String fmuGUID,
                 fmi2String fmuResourceLocation,
                 const fmi2CallbackFunctions* functions,
                 fmi2Boolean visible,
                 fmi2Boolean loggingOn);

    ~FmuComponent() {}

  private:
    // FMU implementation overrides
    virtual bool is_cosimulation_available() const override { return false; }
    virtual bool is_modelexchange_available() const override { return true; }

    virtual fmi2Status enterInitializationModeIMPL() override;
    virtual fmi2Status exitInitializationModeIMPL() override;

    virtual fmi2Status getContinuousStatesIMPL(fmi2Real x[], size_t nx) override;
    virtual fmi2Status setContinuousStatesIMPL(const fmi2Real x[], size_t nx) override;
    virtual fmi2Status getDerivativesIMPL(fmi2Real derivatives[], size_t nx) override;

    void calcAcceleration();

    // Problem states
    typedef std::array<double, 2> vec2;

    vec2 q;     // states
    double a;   // acceleration
    double u;   // forcing term
    double mu;  // problem parameter
};
