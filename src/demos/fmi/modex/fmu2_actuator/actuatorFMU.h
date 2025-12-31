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

#pragma once

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChHydraulicCircuit.h"

#include "chrono_fmi/fmi2/ChFmuToolsExport.h"

class FmuComponent : public chrono::fmi2::FmuChronoComponentBase {
  public:
    FmuComponent(fmi2String instanceName,
                 fmi2Type fmuType,
                 fmi2String fmuGUID,
                 fmi2String fmuResourceLocation,
                 const fmi2CallbackFunctions* functions,
                 fmi2Boolean visible,
                 fmi2Boolean loggingOn);
    virtual ~FmuComponent() {}

  private:
    virtual bool is_cosimulation_available() const override { return false; }
    virtual bool is_modelexchange_available() const override { return true; }

    virtual fmi2Status enterInitializationModeIMPL() override;
    virtual fmi2Status exitInitializationModeIMPL() override;

    virtual fmi2Status getContinuousStatesIMPL(fmi2Real x[], size_t nx) override;
    virtual fmi2Status setContinuousStatesIMPL(const fmi2Real x[], size_t nx) override;
    virtual fmi2Status getDerivativesIMPL(fmi2Real derivatives[], size_t nx) override;

    virtual void preModelDescriptionExport() override;
    virtual void postModelDescriptionExport() override;

    /// Evaluate pressure rates at curent state.
    chrono::Vec2 EvaluatePressureRates(double t, const chrono::Vec2& p, double U);

    /// Calculate current actuator force.
    void calcForce();

    // FMU states
    chrono::ChVectorN<double, 3> q;   ///< states (U, p1, p2)
    chrono::ChVectorN<double, 3> qd;  ///< state derivatives (Ud, p1d, p2d)

    // Hydraulic circuit components    
    chrono::ChHydraulicCylinder cyl;                ///< hydraulic cylinder
    chrono::ChHydraulicDirectionalValve4x3 dvalve;  ///< directional valve

    // Actuator parameters
    double hose1V;  ///< hose 1 volume [m^3]
    double hose2V;  ///< hose 2 volume [m^3]
    double Bo;      ///< oil bulk modulus [Pa]
    double Bh;      ///< hose bulk modulus [Pa]
    double Bc;      ///< cylinder bulk modulus [Pa]
    double pP;      ///< pump pressure [Pa]
    double pT;      ///< tank pressure [Pa]

    double init_s;  ///< initial actuator length
    double init_F;  ///< initial load (FMU initialization input)

    double s;     ///< actuator length (FMU input)
    double sd;    ///< actuator length rate (FMU input)
    double Uref;  ///< input signal (FMU input)

    double F;     ///< actuator force (FMU output)
};
