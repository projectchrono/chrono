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
// The vehicle does not include an engine, transmission, or tires.
//
// The wrapped Chrono::Vehicle model is defined through a JSON specification file
// for the vehicle.
//
// This vehicle FMU must be co-simulated with a powertrain which provides the
// torque at the driveshaft, a driver system which provides vehicle commands, and
// 4 tire systems which provide tire loads for each of the 4 wheels (of type
// TerrainForce).
//
// This vehicle FMU defines continuous output variables for:
//   - vehicle reference frame (of type ChFrameMoving)
//   - wheel states (of type WheelState)
//   - driveshaft speed
//
// =============================================================================

#pragma once

#include <string>
#include <vector>
#include <array>

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include "chrono_fmi/fmi2/ChFmuToolsExport.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
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

    /// Create the vehicle system.
    /// This function is invoked in exitInitializationModeIMPL(), once FMU parameters are set.
    void CreateVehicle();

    /// Configure the underlying Chrono system.
    /// This function is invoked in exitInitializationModeIMPL(), once FMU parameters are set.
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

    // FMU I/O parameters
    std::string out_path;  ///< output directory
    bool save_img;         ///< enable/disable saving of visualization snapshots
    double fps;            ///< snapshot saving frequency (in FPS)

    // FMU parameters
    std::string data_path;                     ///< path to vehicle data
    std::string vehicle_JSON;                  ///< JSON vehicle specification file
    fmi2Boolean system_SMC;                    ///< use SMC contact formulation (NSC otherwise)
    chrono::ChVector3d init_loc;               ///< initial vehicle location
    double init_yaw;                           ///< initial vehicle orientation
    chrono::ChVector3d engineblock_dir;        ///< engine block mounting direction
    chrono::ChVector3d transmissionblock_dir;  ///< transmission block mounting direction
    chrono::ChVector3d g_acc;                  ///< gravitational acceleration
    double step_size;                          ///< integration step size

    // FMU continuous inputs and outputs for co-simulation (vehicle-terrain)
    chrono::vehicle::DriverInputs driver_inputs;  ///< vehicle control inputs (input)
    std::array<WheelData, 4> wheel_data;          ///< wheel state and applied forces (output/input)
    chrono::ChFrameMoving<> ref_frame;            ///< vehicle reference frame (output)

    // FMU continuous inputs and outputs for co-simulation (vehicle-powertrain)
    double driveshaft_speed;       ///< driveshaft angular speed (output)
    double driveshaft_torque;      ///< driveshaft motor torque (input)
    double engine_reaction;        ///< engine reaction torque on chassis (intput)
    double transmission_reaction;  ///< transmission reaction torque on chassis (intput)

    //// TODO - more outputs

    int render_frame;  ///< counter for rendered frames
};
