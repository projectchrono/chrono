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
/// This model uses a trivial speed-torque curve, has no torque converter
/// and no transmission box.
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

    /// Return the current transmission gear.
    /// This simplified model does not have a transmission box.
    virtual int GetCurrentTransmissionGear() const override { return 1; }

    /// Return the output torque from the powertrain.
    /// This is the torque that is passed to a vehicle system, thus providing the
    /// interface between the powertrain and vehicle co-simulation modules.
    virtual double GetOutputTorque() const override { return m_shaftTorque; }

    /// Use this function to set the mode of automatic transmission.
    /// This simplified model does not have a transmission box.
    virtual void SetDriveMode(ChPowertrain::DriveMode mmode) override;

    /// Initialize the powertrain system.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,     ///< [in] chassis o the associated vehicle
                            std::shared_ptr<ChShaft> driveshaft  ///< [in] shaft connection to the vehicle driveline
                            ) override;

    /// Update the state of this powertrain system at the current time.
    /// The powertrain system is provided the current driver throttle input, a
    /// value in the range [0,1], and the current angular speed of the transmission
    /// shaft (from the driveline).
    virtual void Synchronize(double time,        ///< [in] current time
                             double throttle,    ///< [in] current throttle input [0,1]
                             double shaft_speed  ///< [in] current angular speed of the transmission shaft
                             ) override;

    /// Advance the state of this powertrain system by the specified time step.
    /// This function does nothing for this simplified powertrain model.
    virtual void Advance(double step) override {}

  protected:
    /// Return the forward gear ratio (single gear transmission).
    virtual double GetForwardGearRatio() const = 0;

    /// Return the reverse gear ratio.
    virtual double GetReverseGearRatio() const = 0;

    /// Return the maximum motor torque.
    virtual double GetMaxTorque() const = 0;

    /// Return the maximum motor power.
    virtual double GetMaxPower() const = 0;

    /// Return the critical motor speed (torque cutoff).
    virtual double GetCriticalSpeed() const = 0;

  private:
    double m_current_gear_ratio;
    double m_motorSpeed;
    double m_motorTorque;
    double m_shaftTorque;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
