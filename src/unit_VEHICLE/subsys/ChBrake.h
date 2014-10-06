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

#include <vector>

#include "core/ChShared.h"
#include "physics/ChLinkLock.h"

#include "subsys/ChApiSubsys.h"

namespace chrono {

///
/// Base class for a brake subsystem
///
class CH_SUBSYS_API ChBrake : public ChShared
{
public:

  ChBrake();
  virtual ~ChBrake() {}

  /// Initialize the brake by providing the wheel's revolute link.
  virtual void Initialize(ChSharedPtr<ChLinkLockRevolute> hub) = 0;

  /// Set the brake modulation.
  /// The input value is in the range [0,1].<br>
  ///   modulation = 0 indicates no braking<br>
  ///   modulation = 1 indicates that the subsystem should provide maximum braking torque
  virtual void ApplyBrakeModulation(double modulation) = 0;

  /// Get the current brake torque.
  virtual double GetBrakeTorque() = 0;
};


/// Vector of handles to brake subsystems.
typedef std::vector<ChSharedPtr<ChBrake> >  ChBrakeList;


} // end namespace chrono


#endif
