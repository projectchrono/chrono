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

#include "subsys/ChApiSubsys.h"
#include "subsys/ChVehicle.h"

namespace chrono {

///
/// Base class for a powertrain system.
///
class CH_SUBSYS_API ChPowertrain : public ChShared
{
public:

  enum DriveMode {
    FORWARD,
    NEUTRAL,
    REVERSE
  };

  ChPowertrain(ChVehicle* car);

  virtual ~ChPowertrain() {}

  /// Get the engine speed.
  virtual double GetMotorSpeed() const = 0;

  /// Get the engine torque.
  virtual double GetMotorTorque() const = 0;

  /// Update the state of this powertrain system at the current time.
  /// The powertrain system is provided the current driver throttle input, a
  /// value in the range [0,1].
  virtual void Update(
    double time,       ///< [in] current time
    double throttle    ///< [in] current throttle input [0,1]
    ) = 0;
};


} // end namespace chrono


#endif
