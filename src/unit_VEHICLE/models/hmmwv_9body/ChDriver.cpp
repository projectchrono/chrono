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
// current values of the inputs (throttle, steering, braking). To set these
// values, a concrete driver class can implement the virtual method Update()
// which will be invoked at each time step.
//
// =============================================================================

#include "ChDriver.h"


namespace chrono {


// -----------------------------------------------------------------------------
// Clamp a specified input value to appropriate interval.
// -----------------------------------------------------------------------------
void ChDriver::setSteering(double val)
{
  if (val < -1)     m_steering = -1;
  else if (val > 1) m_steering = 1;
  else              m_steering = val;
}

void ChDriver::setThrottle(double val)
{
  if (val < -1)     m_throttle = -1;
  else if (val > 1) m_throttle = 1;
  else              m_throttle = val;
}


}  // end namespace chrono
