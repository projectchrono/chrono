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

#ifndef HMMWV9_POWERTRAIN_H
#define HMMWV9_POWERTRAIN_H

#include "subsys/ChPowertrain.h"

#include "HMMWV9_Vehicle.h"

namespace hmmwv9 {

// Forward reference
class HMMWV9_Vehicle;

class HMMWV9_Powertrain : public chrono::ChPowertrain {
public:

  HMMWV9_Powertrain(HMMWV9_Vehicle* car);

  ~HMMWV9_Powertrain() {}

  virtual double GetWheelTorque(chrono::ChWheelId which) const;
  virtual void Update(double time, double throttle);

  double GetMotorSpeed() const { return m_motorSpeed; }
  double GetMotorTorque() const { return m_motorTorque; }

private:

  double  m_motorSpeed;
  double  m_motorTorque;
  double  m_wheelTorque;

  static const double m_conic_tau;   // the transmission ratio of the conic gears at the rear axle
  static const double m_gear_tau;    // the actual tau of the gear
  static const double m_max_torque;  // the max torque of the motor [Nm];
  static const double m_max_speed;   // the max rotation speed of the motor [rads/s]
};


} // end namespace hmmwv9


#endif
