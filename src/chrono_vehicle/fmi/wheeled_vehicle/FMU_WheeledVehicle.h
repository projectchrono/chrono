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

#include <string>
#include <vector>
#include <array>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "chrono_fmi/ChFmuToolsExport.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
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

    void CreateVehicle();
    void ConfigureSystem();
    void SynchronizeVehicle(double time);
    void CalculateVehicleOutputs();

    /// Exchange data for vehicle wheels.
    struct WheelData {
        std::shared_ptr<chrono::vehicle::ChWheel> wheel;
        std::string identifier;
        chrono::vehicle::WheelState state;
        chrono::vehicle::TerrainForce load;
    };

    std::shared_ptr<chrono::vehicle::WheeledVehicle> vehicle;  ///< underlying wheeled vehicle

#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> vis_sys;
#endif

    // FMU parameters
    std::string vehicle_JSON;       ///< JSON vehicle specification file
    std::string engine_JSON;        ///< JSON engine specification file
    std::string transmission_JSON;  ///< JSON transmission specification file
    fmi2Boolean system_SMC = true;  ///< use SMC contact formulation (NSC otherwise)
    fmi2Boolean vis = false;        ///< enable/disable run-time visualization
    chrono::ChVector<> init_loc;    ///< initial vehicle location
    double init_yaw;                ///< initial vehicle orientation
    double step_size;               ///< integration step size

    // FMU outputs and inputs for co-simulation
    chrono::vehicle::DriverInputs driver_inputs;  ///< vehicle control inputs
    std::array<WheelData, 4> wheel_data;          ///< wheel state and applied forces

    //// TODO - more outputs
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
