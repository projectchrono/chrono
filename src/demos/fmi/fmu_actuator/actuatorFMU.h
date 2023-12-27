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
#include "chrono/motion_functions/ChFunction.h"

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "chrono_fmi/ChFmuToolsExport.h"

class FmuComponent : public chrono::FmuChronoComponentBase {
  public:
    FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID);
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
    std::shared_ptr<chrono::ChFunction_Setpoint> m_actuation;
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
