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
// Authors: Alessandro Tasora
// =============================================================================
//
// Base class for a wheel brake.
//
// =============================================================================

#ifndef CH_BRAKE_H
#define CH_BRAKE_H

#include "core/ChShared.h"
#include "physics/ChLinkLock.h"

#include "subsys/ChApiSubsys.h"

namespace chrono {

class CH_SUBSYS_API ChBrake : public ChShared {
public:

  ChBrake();
  virtual ~ChBrake() {}

  /// Set the brake modulation, in 0..1 range, 
  /// when = 0 it is completely free,
  /// when = 1 it should provide the max braking torque
  /// This function can be called to modulate braking in realtime simulation loops.
  virtual void ApplyBrakeModulation(double modulation) = 0;

  /// Get the current brake torque, as a result of simulation,
  /// so it might change from time to time
  virtual double GetBrakeTorque() = 0;

protected:
};


} // end namespace chrono


#endif
