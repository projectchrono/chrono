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

#pragma once

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChHydraulicActuator.h"
#include "chrono/functions/ChFunction.h"

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

    /// Advance dynamics
    virtual fmi2Status doStepIMPL(fmi2Real currentCommunicationPoint,
                                  fmi2Real communicationStepSize,
                                  fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  protected:
    virtual fmi2Status enterInitializationModeIMPL() override;
    virtual fmi2Status exitInitializationModeIMPL() override;

    virtual void preModelDescriptionExport() override;
    virtual void postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    void CalculateActuatorForce();
    void CalculatePistonPressures();
    void CalculateValvePosition();

    chrono::ChSystemSMC sys;

    double s;       // actuator length (FMU input)
    double sd;      // actuator length rate (FMU input)
    double F;       // actuator force (FMU output)
    double Uref;    // input signal (FMU input)
    double init_F;  // initial load (FMU initialization input)

    double p1;  // piston pressure 1
    double p2;  // piston pressure 2
    double U;   // valve position

    bool have_s0;

    std::shared_ptr<chrono::ChHydraulicActuator2> m_actuator;
    std::shared_ptr<chrono::ChFunctionSetpoint> m_actuation;
};
