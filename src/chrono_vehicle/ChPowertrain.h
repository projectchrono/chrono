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
// Base class for a vehicle powertrain.
//
// =============================================================================

#ifndef CH_POWERTRAIN_H
#define CH_POWERTRAIN_H

#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChBody.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {

///
/// Base class for a powertrain system.
///
class CH_VEHICLE_API ChPowertrain : public ChShared
{
public:

  enum DriveMode {
    FORWARD,
    NEUTRAL,
    REVERSE
  };

  ChPowertrain();

  virtual ~ChPowertrain() {}

  /// Return the current engine speed.
  virtual double GetMotorSpeed() const = 0;

  /// Return the current engine torque.
  virtual double GetMotorTorque() const = 0;

  /// Return the value of slippage in the torque converter.
  virtual double GetTorqueConverterSlippage() const = 0;

  /// Return the input torque to the torque converter.
  virtual double GetTorqueConverterInputTorque() const = 0;

  /// Return the output torque from the torque converter.
  virtual double GetTorqueConverterOutputTorque() const = 0;

  /// Return the current transmission gear.
  virtual int GetCurrentTransmissionGear() const = 0;

  /// Return the ouput torque from the powertrain.
  /// This is the torque that is passed to a vehicle system, thus providing the
  /// interface between the powertrain and vehcicle cosimulation modules.
  virtual double GetOutputTorque() const = 0;

  /// Return the current mode of the transmission.
  DriveMode GetDriveMode() { return m_drive_mode; }

  /// Set the mode of the transmission.
  virtual void SetDriveMode(DriveMode mmode) = 0;

  /// Update the state of this powertrain system at the current time.
  /// The powertrain system is provided the current driver throttle input, a
  /// value in the range [0,1], and the current angular speed of the transmission
  /// shaft (from the driveline).
  virtual void Update(
    double time,       ///< [in] current time
    double throttle,   ///< [in] current throttle input [0,1]
    double shaft_speed ///< [in] current angular speed of the transmission shaft
    ) = 0;

  /// Advance the state of this powertrain system by the specified time step.
  virtual void Advance(double step) = 0;

protected:
  DriveMode m_drive_mode;
};


} // end namespace chrono


#endif
