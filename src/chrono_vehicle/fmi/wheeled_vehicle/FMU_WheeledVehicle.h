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
// Co-simulation FMU encapsulating a wheeled vehicle system with 4 wheels.
// The vehicle includes a powertrain and transmission, but no tires.
//
// The wrapped Chrono::Vehicle model is defined through JSON specification files
// for the vehicle, engine, and transmission.
//
// This vehicle FMU must be co-simulated with a driver system which provides
// vehicle commands and 4 tire systems which provide tire loads for each of the
// 4 wheels (of type TerrainForce).
//
// This vehicle FMU defines continuous output variables for:
//   - vehicle reference frame (of type ChFrameMoving)
//   - wheel states (of type WheelState)
//
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
    ~FmuComponent() {}

    /// Advance dynamics.
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  private:
    virtual void _enterInitializationMode() override;
    virtual void _exitInitializationMode() override;

    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    /// Create the vehicle system.
    /// This function is invoked in _exitInitializationMode(), once FMU parameters are set.
    void CreateVehicle();

    /// Configure the underlying Chrono system.
    /// This function is invoked in _exitInitializationMode(), once FMU parameters are set.
    void ConfigureSystem();

    /// Update vehicle system with current FMU continuous inputs.
    /// This function is called before advancing dynamics of the vehicle.
    void SynchronizeVehicle(double time);

    /// Extract FMU continuous outputs from the vehicle system.
    /// This function is called after advancing dynamics of the vehicle.
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
    fmi2Boolean system_SMC;         ///< use SMC contact formulation (NSC otherwise)
    fmi2Boolean vis;                ///< enable/disable run-time visualization
    chrono::ChVector<> init_loc;    ///< initial vehicle location
    double init_yaw;                ///< initial vehicle orientation
    chrono::ChVector<> g_acc;       ///< gravitational acceleration
    double step_size;               ///< integration step size

    // FMU continuous inputs and outputs for co-simulation
    chrono::vehicle::DriverInputs driver_inputs;  ///< vehicle control inputs (input)
    std::array<WheelData, 4> wheel_data;          ///< wheel state and applied forces (output/input)
    chrono::ChFrameMoving<> ref_frame;            ///< vehicle reference frame (output)

    //// TODO - more outputs
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
