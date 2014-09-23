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

#include "subsys/ChDriver.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDriver::ChDriver()
: m_throttle(0), m_steering(0), m_braking(0)
{
}

// -----------------------------------------------------------------------------
// Clamp a specified input value to appropriate interval.
// -----------------------------------------------------------------------------
double clamp(double val, double min_val, double max_val)
{
  if (val <= min_val)
    return min_val;
  if (val >= max_val)
    return max_val;
  return val;
}

void ChDriver::SetSteering(double val, double min_val, double max_val)
{
  m_steering = clamp(val, min_val, max_val);
}

void ChDriver::SetThrottle(double val, double min_val, double max_val)
{
  m_throttle = clamp(val, min_val, max_val);
}

void ChDriver::SetBraking(double val, double min_val, double max_val)
{
  m_braking = clamp(val, min_val, max_val);
}


}  // end namespace chrono
