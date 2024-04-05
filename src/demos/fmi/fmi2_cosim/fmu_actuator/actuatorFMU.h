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

class FmuComponent : public chrono::FmuChronoComponentBase {
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
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  protected:
    virtual void _enterInitializationMode() override;
    virtual void _exitInitializationMode() override;

    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

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

    std::shared_ptr<chrono::ChHydraulicActuator2> m_actuator;
    std::shared_ptr<chrono::ChFunctionSetpoint> m_actuation;
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName,
                                             fmi2Type fmuType,
                                             fmi2String fmuGUID,
                                             fmi2String fmuResourceLocation,
                                             const fmi2CallbackFunctions* functions,
                                             fmi2Boolean visible,
                                             fmi2Boolean loggingOn) {
    return new FmuComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}
