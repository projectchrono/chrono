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

#pragma once

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadsBody.h"

#include "chrono_fmi/fmi3/ChFmuToolsExport.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

class FmuComponent : public chrono::fmi3::FmuChronoComponentBase {
  public:
    FmuComponent(fmu_forge::fmi3::FmuType fmiInterfaceType,
                 fmi3String instanceName,
                 fmi3String instantiationToken,
                 fmi3String resourcePath,
                 fmi3Boolean visible,
                 fmi3Boolean loggingOn,
                 fmi3InstanceEnvironment instanceEnvironment,
                 fmi3LogMessageCallback logMessage);
    virtual ~FmuComponent() {}

    /// Advance dynamics
    virtual fmi3Status doStepIMPL(fmi3Float64 currentCommunicationPoint,
                                  fmi3Float64 communicationStepSize,
                                  fmi3Boolean noSetFMUStatePriorToCurrentPoint,
                                  fmi3Boolean* eventHandlingNeeded,
                                  fmi3Boolean* terminateSimulation,
                                  fmi3Boolean* earlyReturn,
                                  fmi3Float64* lastSuccessfulTime) override;

  protected:
    virtual fmi3Status enterInitializationModeIMPL() override;
    virtual fmi3Status exitInitializationModeIMPL() override;

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
