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
// - RWD only
// - trivial speed-torque curve
// - no differential
//
// =============================================================================

#include "subsys/powertrain/ChSimplePowertrain.h"

namespace chrono {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSimplePowertrain::ChSimplePowertrain()
: ChPowertrain(),
  m_motorSpeed(0),
  m_motorTorque(0),
  m_shaftTorque(0)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimplePowertrain::Update(double time,
                                double throttle,
                                double shaft_speed)
{
  // The motorspeed is the shaft speed multiplied by gear ratio inversed:
  m_motorSpeed = shaft_speed * (1.0 / GetGearRatio());

  // The torque depends on speed-torque curve of the motor: here we assume a 
  // very simplified model a bit like in DC motors:
  m_motorTorque = GetMaxTorque() - m_motorSpeed * (GetMaxTorque() / GetMaxSpeed());

  // Motor torque is linearly modulated by throttle gas value:
  m_motorTorque *= throttle;

  // The torque at motor shaft:
  m_shaftTorque = m_motorTorque * (1.0 / GetGearRatio());
}


} // end namespace chrono
