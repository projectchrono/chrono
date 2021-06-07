// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple powertrain model template.
// - hyperbolical speed-torque curve like a CVT gearbox
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef CH_SIMPLE_CVT_POWERTRAIN_H
#define CH_SIMPLE_CVT_POWERTRAIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Template for simplified powertrain model.
/// This model uses a trivial speed-torque curve, has no torque converter, and no transmission box.
/// As such, in automatic mode, it always uses the 1st forward gear ratio.
class CH_VEHICLE_API ChSimpleCVTPowertrain : public ChPowertrain {
  public:
    ChSimpleCVTPowertrain(const std::string& name);

    virtual ~ChSimpleCVTPowertrain() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SimpleCVTPowertrain"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motorSpeed; }

    /// Return the current engine torque.
    virtual double GetMotorTorque() const override { return m_motorTorque; }

    /// Return the value of slippage in the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterSlippage() const override { return 0; }

    /// Return the input torque to the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterInputTorque() const override { return 0; }

    /// Return the output torque from the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterOutputTorque() const override { return 0; }

    /// Return the torque converter output shaft speed.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterOutputSpeed() const override { return 0; }

    /// Return the output torque from the powertrain.
    /// This is the torque that is passed to a vehicle system, thus providing the
    /// interface between the powertrain and vehicle co-simulation modules.
    virtual double GetOutputTorque() const override { return m_shaftTorque; }

  protected:
    /// Return the maximum motor torque.
    virtual double GetMaxTorque() const = 0;

    /// Return the maximum motor power.
    virtual double GetMaxPower() const = 0;

    /// Return the maximum motor speed (above which torque is 0).
    virtual double GetMaxSpeed() const = 0;

  private:
    /// Initialize the powertrain system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,     ///< [in] chassis of the associated vehicle
                            std::shared_ptr<ChDriveline> driveline  ///< [in] driveline of the associated vehicle
                            ) override;

    /// Update the state of this powertrain system at the current time.
    /// The powertrain system is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,     ///< [in] current time
                             double throttle  ///< [in] current throttle input [0,1]
                             ) override;

    /// Advance the state of this powertrain system by the specified time step.
    /// This function does nothing for this simplified powertrain model.
    virtual void Advance(double step) override {}

    double m_motorSpeed;
    double m_motorTorque;
    double m_shaftTorque;
    double m_critical_speed;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
