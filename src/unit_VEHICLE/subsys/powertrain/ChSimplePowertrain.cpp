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
ChSimplePowertrain::ChSimplePowertrain(ChVehicle* car)
: ChPowertrain(car),
  m_wheelTorque(0),
  m_motorSpeed(0),
  m_motorTorque(0)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimplePowertrain::Initialize(ChSharedPtr<ChBody>  chassis,
                                    ChSharedPtr<ChShaft> axle_L,
                                    ChSharedPtr<ChShaft> axle_R)
{
  m_axle_L = axle_L;
  m_axle_R = axle_R;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSimplePowertrain::GetWheelTorque(ChWheelId which) const
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
  default:
    return -1;  // should not happen
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSimplePowertrain::Update(double time,
                                double throttle)
{
  // Get wheel angular speeds.
  double wheelSpeedL = m_axle_L->GetPos_dt();
  double wheelSpeedR = m_axle_R->GetPos_dt();

  // Assume clutch is never used. Given the kinematics of differential, the
  // speed of the engine transmission shaft is the average of the two wheel
  // speeds, multiplied the conic gear transmission ratio inversed:
  double shaftSpeed = (1.0 / GetConicTau()) * 0.5 * (wheelSpeedL + wheelSpeedR);

  // The motorspeed is the shaft speed multiplied by gear ratio inversed:
  m_motorSpeed = shaftSpeed * (1.0 / GetGearTau());

  // The torque depends on speed-torque curve of the motor: here we assume a 
  // very simplified model a bit like in DC motors:
  m_motorTorque = GetMaxTorque() - m_motorSpeed*(GetMaxTorque() / GetMaxSpeed());

  // Motor torque is linearly modulated by throttle gas value:
  m_motorTorque *= throttle;

  // The torque at motor shaft:
  double shaftTorque = m_motorTorque * (1.0 / GetGearTau());

  // The torque at wheels - for each wheel, given the differential transmission, 
  // it is half of the shaft torque  (multiplied the conic gear transmission ratio)
  m_wheelTorque = 0.5 * shaftTorque * (1.0 / GetConicTau());

  // Apply torques directly to the rear suspension subsystems.
  m_axle_L->SetAppliedTorque(-m_wheelTorque);
  m_axle_R->SetAppliedTorque(-m_wheelTorque);
}


} // end namespace chrono
