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
// Simple powertrain model for the HMMWV vehicle.
// - RWD only
// - trivial speed-torque curve
// - no differential
//
// =============================================================================

#include "HMMWV9_Powertrain.h"

using namespace chrono;

namespace hmmwv9 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV9_Powertrain::m_max_torque = 2400 / 8.851;
const double HMMWV9_Powertrain::m_max_speed  = 2000;
const double HMMWV9_Powertrain::m_conic_tau = 0.2;
const double HMMWV9_Powertrain::m_gear_tau = 0.3;


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_Powertrain::HMMWV9_Powertrain(HMMWV9_Vehicle* car)
: ChPowertrain(car, RWD),
  m_wheelTorque(0),
  m_motorSpeed(0),
  m_motorTorque(0)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV9_Powertrain::GetWheelTorque(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return 0;
  case FRONT_RIGHT:
    return 0;
  case REAR_LEFT:
    return m_wheelTorque;
  case REAR_RIGHT:
    return m_wheelTorque;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_Powertrain::Update(double time,
                               double throttle)
{
  // Get wheel angular speeds.
  double wheelSpeedL = m_car->GetWheelAngSpeed(REAR_LEFT);
  double wheelSpeedR = m_car->GetWheelAngSpeed(REAR_RIGHT);

  // Assume clutch is never used. Given the kinematics of differential, the
  // speed of the engine transmission shaft is the average of the two wheel
  // speeds, multiplied the conic gear transmission ratio inversed:
  double shaftSpeed = (1.0 / m_conic_tau) * 0.5 * (wheelSpeedL + wheelSpeedR);

  // The motorspeed is the shaft speed multiplied by gear ratio inversed:
  m_motorSpeed = shaftSpeed * (1.0 / m_gear_tau);

  // The torque depends on speed-torque curve of the motor: here we assume a 
  // very simplified model a bit like in DC motors:
  m_motorTorque = m_max_torque - m_motorSpeed*(m_max_torque / m_max_speed);

  // Motor torque is linearly modulated by throttle gas value:
  m_motorTorque *= throttle;

  // The torque at motor shaft:
  double shaftTorque = m_motorTorque * (1.0 / m_gear_tau);

  // The torque at wheels - for each wheel, given the differential transmission, 
  // it is half of the shaft torque  (multiplied the conic gear transmission ratio)
  m_wheelTorque = 0.5 * shaftTorque * (1.0 / m_conic_tau);

}


} // end namespace hmmwv9
