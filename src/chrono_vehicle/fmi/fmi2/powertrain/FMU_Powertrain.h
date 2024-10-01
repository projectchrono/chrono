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
//
// Co-simulation FMU encapsulating a powertrain system.
//
// The wrapped Chrono::Vehicle powertrain model is defined through JSON specification
// files for the engine and the transmission.
//
// This powertrain FMU must be co-simulated with a vehicle system which provides
// the current chassis state (of type WheelState) and the driveshaft angular speed.
//
// This powertrain FMU defines continuous output variables for:
//   - motor torque on the driveshaft
//
// =============================================================================

#pragma once

#include <string>
#include <vector>
#include <array>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChEngine.h"
#include "chrono_vehicle/ChTransmission.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"

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
    ~FmuComponent() {}

    /// Advance dynamics.
    virtual fmi2Status doStepIMPL(fmi2Real currentCommunicationPoint,
                                  fmi2Real communicationStepSize,
                                  fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  private:
    virtual fmi2Status enterInitializationModeIMPL() override;
    virtual fmi2Status exitInitializationModeIMPL() override;

    virtual void preModelDescriptionExport() override;
    virtual void postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    void CreatePowertrain();
    void SynchronizePowertrain(double time);
    void CalculatePowertrainOutputs();

    std::shared_ptr<chrono::vehicle::ChEngine> engine;                  ///< underlying engine
    std::shared_ptr<chrono::vehicle::ChTransmission> transmission;      ///< underlying transmission
    std::shared_ptr<chrono::vehicle::ChPowertrainAssembly> powertrain;  ///< underlying powertrain
    chrono::ChSystemSMC sys;                                            ///< containing system

    // FMU I/O parameters
    std::string out_path;  ///< output directory

    // FMU parameters
    std::string engine_JSON;        ///< JSON engine specification file
    std::string transmission_JSON;  ///< JSON transmission specification file
    double step_size;               ///< integration step size

    // FMU inputs and outputs
    double throttle;               ///< throttle driver command, in [0,1] (input)
    double clutch;                 ///< clutch driver command, in [0,1] (input)
    double driveshaft_speed;       ///< driveshaft angular speed (input)
    double driveshaft_torque;      ///< driveshaft motor torque (output)
    double engine_reaction;        ///< engine reaction torque on chassis (output)
    double transmission_reaction;  ///< transmission reaction torque on chassis (output)
};
