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
// External project template for building a Chrono co-simulation FMU.
// =============================================================================

#pragma once

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_fmi/ChFmuToolsExport.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

class FmuComponent : public chrono::FmuChronoComponentBase {
  public:
    FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID);

    virtual ~FmuComponent() {}

    /// FMU_ACTION: override _doStep of the base class with the problem-specific implementation
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  protected:
    virtual void _exitInitializationMode() override;

    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    // Problem-specific data members
    chrono::ChSystemNSC sys;
    chrono::ChRealtimeStepTimer realtime_timer;

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vis;
#endif

    double x_tt;
    double x_t;
    double x;
    double theta_tt;
    double theta_t;
    double theta;
    double pendulum_length = 0.5;
    double pendulum_mass = 1.0;
    double cart_mass = 1.0;
};

// FMU_ACTION:: implement the following functions
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
