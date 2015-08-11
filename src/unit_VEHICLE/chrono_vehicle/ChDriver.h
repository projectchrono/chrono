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
// Base class for a vehicle driver. A driver system must be able to report the
// current values of the inputs (throttle, steering, braking).
//
// =============================================================================

#ifndef CH_DRIVER_H
#define CH_DRIVER_H

#include <string>

#include "core/ChShared.h"
#include "physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"

namespace chrono {

///
/// Base class for a vehicle driver system.
/// A driver system must be able to report the current values of the inputs
/// (throttle, steering, braking). A concrete driver class must set the member
/// variables m_throttle, m_steering, and m_braking.
///
class CH_VEHICLE_API ChDriver : public ChShared
{
public:

  ChDriver();
  virtual ~ChDriver() {}

  /// Get the driver throttle input (in the range [0,1])
  double GetThrottle() const { return m_throttle; }

  /// Get the driver steering input (in the range [-1,+1])
  double GetSteering() const { return m_steering; }

  /// Get the driver braking input (in the range [0,1])
  double GetBraking() const  { return m_braking; }

  /// Update the state of this driver system at the current time.
  virtual void Update(double time) {}

  /// Advance the state of this driver system by the specified time step.
  virtual void Advance(double step) {}

  /// Initialize output file for recording driver inputs.
  bool LogInit(const std::string& filename);

  /// Record the current driver inputs to the log file.
  bool Log(double time);

protected:
  /// clamp to interval
  double clamp(double val, double min_val, double max_val);

  /// Set the value for the driver steering input.
  void SetSteering(double val, double min_val = -1, double max_val = 1);

  /// Set the value for the driver throttle input.
  void SetThrottle(double val, double min_val = 0, double max_val = 1);

  /// Set the value for the driver braking input.
  void SetBraking(double val, double min_val = 0, double max_val = 1);

  double m_throttle;   ///< current value of throttle input
  double m_steering;   ///< current value of steering input
  double m_braking;    ///< current value of braking input

private:
  std::string  m_log_filename;  // name of output file for recording driver inputs

};


} // end namespace chrono


#endif