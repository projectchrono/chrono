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
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadsBody.h"

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "chrono_fmi/ChFmuToolsExport.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

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

    void ProcessActuatorForce();
    void CalculateActuatorLength();

    chrono::ChSystemSMC sys;

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vissys;
#endif

    // Enable/disable run-time visualization
    fmi2Boolean vis = false;

    // Body properties (with default values)
    double crane_mass = 500;
    double crane_length = 1.0;
    double pend_mass = 100;
    double pend_length = 0.3;

    double init_crane_angle = chrono::CH_C_PI / 6;  // Initial crane angle (default value)
    double init_F;                                  // initial load

    double s;   // actuator length (FMU output)
    double sd;  // actuator length rate (FMU output)
    double F;   // actuator force (FMU input)

    // Mount points
    chrono::ChVector<> m_point_ground;
    chrono::ChVector<> m_point_crane;

    std::shared_ptr<chrono::ChBody> m_crane;
    std::shared_ptr<chrono::ChLoadBodyForce> m_external_load;
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
