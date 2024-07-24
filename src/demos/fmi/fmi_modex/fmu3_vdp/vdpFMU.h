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

#pragma once

#include <array>

#include "fmi3/FmuToolsExport.h"

class FmuComponent : public fmu_tools::fmi3::FmuComponentBase {
  public:
    FmuComponent(fmu_tools::fmi3::FmuType fmiInterfaceType,
                 fmi3String instanceName,
                 fmi3String instantiationToken,
                 fmi3String resourcePath,
                 fmi3Boolean visible,
                 fmi3Boolean loggingOn,
                 fmi3InstanceEnvironment instanceEnvironment,
                 fmi3LogMessageCallback logMessage);

    ~FmuComponent() {}

  private:
    // FMU implementation overrides
    virtual bool is_cosimulation_available() const override { return false; }
    virtual bool is_modelexchange_available() const override { return true; }

    virtual fmi3Status enterInitializationModeIMPL() override;
    virtual fmi3Status exitInitializationModeIMPL() override;

    virtual fmi3Status getContinuousStatesIMPL(fmi3Float64 continuousStates[], size_t nContinuousStates) override;
    virtual fmi3Status setContinuousStatesIMPL(const fmi3Float64 continuousStates[], size_t nContinuousStates) override;
    virtual fmi3Status getDerivativesIMPL(fmi3Float64 derivatives[], size_t nContinuousStates) override;

    void calcAcceleration();

    // Problem states
    typedef std::array<double, 2> vec2;

    vec2 q;     // states
    double a;   // acceleration
    double u;   // forcing term
    double mu;  // problem parameter
};
