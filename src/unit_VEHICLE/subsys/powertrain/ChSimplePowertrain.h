// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model template.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef CH_SIMPLE_POWERTRAIN_H
#define CH_SIMPLE_POWERTRAIN_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChPowertrain.h"
#include "subsys/ChSuspension.h"

namespace chrono {

class CH_SUBSYS_API ChSimplePowertrain : public ChPowertrain
{
public:

  ChSimplePowertrain();

  ~ChSimplePowertrain() {}

  /// Initialize the powertrain system.
  void Initialize();

  /// Return the current engine speed.
  virtual double GetMotorSpeed() const { return m_motorSpeed; }

  /// Return the current engine torque.
  virtual double GetMotorTorque() const { return m_motorTorque; }

  /// Return the value of slippage in the torque converter.
  /// This simplified model does not have a torque converter.
  virtual double GetTorqueConverterSlippage() const { return 0; }

  /// Return the input torque to the torque converter.
  /// This simplified model does not have a torque converter.
  virtual double GetTorqueConverterInputTorque() const { return 0; }

  /// Return the output torque from the torque converter.
  /// This simplified model does not have a torque converter.
  virtual double GetTorqueConverterOutputTorque() const { return 0; }

  /// Return the current transmission gear
  /// This simplified model does not have a transmission box.
  virtual int GetCurrentTransmissionGear() const { return 1; }

  /// Return the ouput torque from the powertrain.
  /// This is the torque that is passed to a vehicle system, thus providing the
  /// interface between the powertrain and vehcicle cosimulation modules.
  /// Since a ShaftsPowertrain is directly connected to the vehicle's driveline,
  /// this function returns 0.
  virtual double GetOutputTorque() const { return m_shaftTorque; }

  /// Use this function to set the mode of automatic transmission.
  /// This simplified model does not have a transmission box.
  virtual void SetDriveMode(ChPowertrain::DriveMode mmode);

  /// Update the state of this powertrain system at the current time.
  /// The powertrain system is provided the current driver throttle input, a
  /// value in the range [0,1], and the current angular speed of the transmission
  /// shaft (from the driveline).
  virtual void Update(
    double time,       ///< [in] current time
    double throttle,   ///< [in] current throttle input [0,1]
    double shaft_speed ///< [in] current angular speed of the transmission shaft
    );

  /// Advance the state of this powertrain system by the specified time step.
  /// This function does nothing for this simplified powertrain model.
  virtual void Advance(double step) {}

protected:

  /// Return the forward gear ratio (single gear transmission)
  virtual double GetForwardGearRatio() const = 0;

  /// Return the reverse gear ratio
  virtual double GetReverseGearRatio() const = 0;

  /// Return the maximum motor torque
  virtual double GetMaxTorque() const = 0;

  /// Return the maximum motor speed
  virtual double GetMaxSpeed() const = 0;

private:
  double  m_current_gear_ratio;
  double  m_motorSpeed;
  double  m_motorTorque;
  double  m_shaftTorque;
};


} // end namespace chrono


#endif
