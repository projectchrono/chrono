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
// Base class for a vehicle driver. A driver object must be able to report the
// current values of the inputs (throttle, steering, braking).
//
// =============================================================================

#ifndef CH_DRIVER_H
#define CH_DRIVER_H

#include "core/ChShared.h"
#include "physics/ChSystem.h"

#include "subsys/ChApiSubsys.h"

namespace chrono {

///
/// Base class for a vehicle driver system.
/// A driver object must be able to report the current values of the inputs
/// (throttle, steering, braking). A concrete driver class must set the member
/// variables m_throttle, m_steering, and m_braking.
///
class CH_SUBSYS_API ChDriver : public ChShared
{
public:
  ChDriver();
  virtual ~ChDriver() {}

  double GetThrottle() const { return m_throttle; }
  double GetSteering() const { return m_steering; }
  double GetBraking() const  { return m_braking; }

  virtual void Update(double time) {}
  virtual void Advance(double step) {}

protected:
  void SetSteering(double val, double min_val = -1, double max_val = 1);
  void SetThrottle(double val, double min_val = 0, double max_val = 1);
  void SetBraking(double val, double min_val = 0, double max_val = 1);

  double m_throttle;
  double m_steering;
  double m_braking;

};


} // end namespace chrono


#endif