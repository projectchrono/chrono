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
// Automatic transmission model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_SHAFTS_AUTOMATIC_TRANSMISSION_H
#define CH_SHAFTS_AUTOMATIC_TRANSMISSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTransmission.h"

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsTorqueConverter.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

// Forward reference
class ChVehicle;

/// Template for an automatic transmission model using shaft elements.
/// This transmission template includes a torque converter and a manumatic gearbox.
class CH_VEHICLE_API ChAutomaticTransmissionShafts : public ChTransmission {
  public:
    /// Construct a shafts-based automatic transmission model.
    ChAutomaticTransmissionShafts(const std::string& name);

    virtual ~ChAutomaticTransmissionShafts();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "AutomaticTransmissionShafts"; }

    /// Return the value of slippage in the torque converter.
    virtual double GetTorqueConverterSlippage() const override { return m_torqueconverter->GetSlippage(); }

    /// Return the input torque to the torque converter.
    virtual double GetTorqueConverterInputTorque() const override {
        return -m_torqueconverter->GetTorqueReactionOnInput();
    }

    /// Return the output torque from the torque converter.
    virtual double GetTorqueConverterOutputTorque() const override {
        return m_torqueconverter->GetTorqueReactionOnOutput();
    }

    /// Return the torque converter output shaft speed.
    virtual double GetTorqueConverterOutputSpeed() const override { return m_shaft_ingear->GetPos_dt(); }

    /// Use this to define the gear shift latency, in seconds.
    void SetGearShiftLatency(double ml) { m_gear_shift_latency = ml; }

    /// Use this to get the gear shift latency, in seconds.
    double GetGearShiftLatency(double ml) { return m_gear_shift_latency; }

    /// Return the transmission output torque on the driveshaft.
    /// This is the torque that is passed to the driveline subsystem, thus providing the interface between the
    /// powertrain and vehicle systems.
    virtual double GetOutputDriveshaftTorque() const override;

    /// Return the transmission output speed of the motorshaft.
    /// This represents the output from the transmision subsystem that is passed to the engine subsystem.
    virtual double GetOutputMotorshaftSpeed() const override;

  protected:
    /// Set inertia of the transmission block.
    virtual double GetTransmissionBlockInertia() const = 0;

    /// Inertias of the component ChShaft objects.
    virtual double GetIngearShaftInertia() const = 0;

    /// Inertia of the motorshaft (connection to engine).
    virtual double GetMotorshaftInertia() const = 0;

    /// Inertia of the driveshaft (connection to driveline).
    virtual double GetDriveshaftInertia() const = 0;

    /// Upshift and downshift rotation speeds (in RPM)
    virtual double GetUpshiftRPM() const = 0;
    virtual double GetDownshiftRPM() const = 0;

    /// Set the capacity factor map.
    /// Specify the capacity factor as a function of the speed ratio.
    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

    /// Set the torque ratio map.
    /// Specify torque ratio as a function of the speed ratio.
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

  private:
    /// Initialize this transmission system by attaching it to an existing vehicle chassis and connecting the provided
    /// engine and driveline subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Synchronize the state of this transmission system at the current time.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_torque,           ///< input engine torque
                             double driveshaft_speed             ///< input driveline speed
                             ) override;

    /// Advance the state of this transmission system by the specified time step.
    /// No-op, since the state of a AutomaticTransmissionShafts is advanced as part of the vehicle state.
    virtual void Advance(double step) override {}

    /// Perform any action required on a gear shift (the new gear and gear ratio are available).
    virtual void OnGearShift() override;

    /// Perform any action required on placing the transmission in neutral.
    virtual void OnNeutralShift() override;

    std::shared_ptr<ChShaft> m_motorshaft;  ///< shaft connection to the transmission
    std::shared_ptr<ChShaft> m_driveshaft;  ///< shaft connection to driveline

    std::shared_ptr<ChShaft> m_transmissionblock;
    std::shared_ptr<ChShaftsBody> m_transmissionblock_to_body;
    std::shared_ptr<ChShaftsTorqueConverter> m_torqueconverter;
    std::shared_ptr<ChShaft> m_shaft_ingear;
    std::shared_ptr<ChShaftsGearbox> m_gears;

    double m_last_time_gearshift;
    double m_gear_shift_latency;
    double m_upshift_speed;
    double m_downshift_speed;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
