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

#include "chrono_fmi/fmi2/ChFmuToolsExport.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

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

    void ProcessActuatorForce();
    void CalculateActuatorLength();

    chrono::ChSystemSMC sys;

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vis_sys;
#endif

    // Body properties (with default values)
    double crane_mass = 500;
    double crane_length = 1.0;
    double pend_mass = 100;
    double pend_length = 0.3;

    double init_crane_angle = chrono::CH_PI / 6;  // initial crane angle (default value)
    double init_F;                                // initial load

    double s;   // actuator length (FMU output)
    double sd;  // actuator length rate (FMU output)
    double F;   // actuator force (FMU input)

    // Mount points
    chrono::ChVector3d m_point_ground;
    chrono::ChVector3d m_point_crane;

    std::shared_ptr<chrono::ChBody> m_crane;
    std::shared_ptr<chrono::ChLoadBodyForce> m_external_load;
};
