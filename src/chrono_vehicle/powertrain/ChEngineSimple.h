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
// Simple engine model template
//
// =============================================================================

#ifndef CH_SIMPLE_ENGINE_H
#define CH_SIMPLE_ENGINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChEngine.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Template for simplified engine model.
/// This model uses a trivial speed-torque curve.
class CH_VEHICLE_API ChEngineSimple : public ChEngine {
  public:
    virtual ~ChEngineSimple() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "EngineSimple"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motor_speed; }

    /// Return the output engine torque.
    /// This is the torque passed to a transmission subsystem.
    virtual double GetOutputMotorshaftTorque() const override { return m_motor_torque; }

  protected:
    ChEngineSimple(const std::string& name);

    /// Return the maximum motor torque.
    virtual double GetMaxTorque() const = 0;

    /// Return the maximum motor power.
    virtual double GetMaxPower() const = 0;

    /// Return the maximum motor speed (above which torque is 0).
    virtual double GetMaxSpeed() const = 0;

  private:
    /// Initialize the engine system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this engine system at the current time.
    /// The engine system is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_speed             ///< input transmission speed
                             ) override;

    /// Advance the state of this engine system by the specified time step.
    /// This function does nothing for this simplified engine model.
    virtual void Advance(double step) override {}

    double m_motor_speed;
    double m_motor_torque;
    double m_critical_speed;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
