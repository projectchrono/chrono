// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Powertrain model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_SHAFTS_POWERTRAIN_H
#define CH_SHAFTS_POWERTRAIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"

#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChShaftsTorqueConverter.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChShaftsTorque.h"
#include "chrono/physics/ChShaftsThermalEngine.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

// Forward reference
class ChVehicle;

/// Template for a powertrain model using shaft elements.
/// This powertrain template includes a torque converter and a manumatic transmission.
class CH_VEHICLE_API ChShaftsPowertrain : public ChPowertrain {
  public:
    /// Construct a shafts-based powertrain model.
    ChShaftsPowertrain(const std::string& name,
                       const ChVector<>& dir_motor_block = ChVector<>(1, 0, 0));

    virtual ~ChShaftsPowertrain();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ShaftsPowertrain"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_crankshaft->GetPos_dt(); }

    /// Return the current engine torque.
    virtual double GetMotorTorque() const override { return m_engine->GetTorqueReactionOn1(); }

    /// Return the value of slippage in the torque converter.
    virtual double GetTorqueConverterSlippage() const override { return m_torqueconverter->GetSlippage(); }

    /// Return the input torque to the torque converter.
    virtual double GetTorqueConverterInputTorque() const override { return -m_torqueconverter->GetTorqueReactionOnInput(); }

    /// Return the output torque from the torque converter.
    virtual double GetTorqueConverterOutputTorque() const override { return m_torqueconverter->GetTorqueReactionOnOutput(); }

    /// Return the torque converter output shaft speed.
    virtual double GetTorqueConverterOutputSpeed() const override { return m_shaft_ingear->GetPos_dt(); }

    /// Return the output torque from the powertrain.
    /// This is the torque that is passed to a vehicle system, thus providing the interface between the powertrain and
    /// vehicle co-simulation modules.
    virtual double GetOutputTorque() const override;

    /// Use this to define the gear shift latency, in seconds.
    void SetGearShiftLatency(double ml) { m_gear_shift_latency = ml; }

    /// Use this to get the gear shift latency, in seconds.
    double GetGearShiftLatency(double ml) { return m_gear_shift_latency; }

  protected:
    /// Inertias of the component ChShaft objects.
    virtual double GetMotorBlockInertia() const = 0;
    virtual double GetCrankshaftInertia() const = 0;
    virtual double GetIngearShaftInertia() const = 0;
    virtual double GetPowershaftInertia() const = 0;

    /// Upshift and downshift rotation speeds (in RPM)
    virtual double GetUpshiftRPM() const = 0;
    virtual double GetDownshiftRPM() const = 0;

    /// Engine speed-torque map.
    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;
    /// Engine speed-torque braking effect because of losses.
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

    /// Set the capacity factor map.
    /// Specify the capacity factor as a function of the speed ratio.
    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;
 
    /// Set the torque ratio map.
    /// Specify torque ratio as a function of the speed ratio.
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

  private:
    /// Initialize this powertrain system.
    /// This creates all the wrapped ChShaft objects and their constraints, torques etc.
    /// and connects the powertrain to the vehicle.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this powertrain system at the current time.
    /// The powertrain system is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                            ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double shaft_speed                      ///< [in] driveshaft speed
                             ) override;

    /// Advance the state of this powertrain system by the specified time step.
    /// Since the state of a ShaftsPowertrain is advanced as part of the vehicle
    /// state, this function does nothing.
    virtual void Advance(double step) override {}

    /// Perform any action required on a gear shift (the new gear and gear ratio are available).
    virtual void OnGearShift() override;

    /// Perform any action required on placing the transmission in neutral.
    virtual void OnNeutralShift() override;

    std::shared_ptr<ChShaftsBody> m_motorblock_to_body;
    std::shared_ptr<ChShaft> m_motorblock;
    std::shared_ptr<ChShaftsThermalEngine> m_engine;
    std::shared_ptr<ChShaftsThermalEngine> m_engine_losses;
    std::shared_ptr<ChShaft> m_crankshaft;
    std::shared_ptr<ChShaftsTorqueConverter> m_torqueconverter;
    std::shared_ptr<ChShaft> m_shaft_ingear;
    std::shared_ptr<ChShaftsGearbox> m_gears;
    std::shared_ptr<ChShaft> m_shaft;  ///< connection to driveline

    ChVector<> m_dir_motor_block;

    double m_last_time_gearshift;
    double m_gear_shift_latency;
    double m_upshift_speed;
    double m_downshift_speed;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
