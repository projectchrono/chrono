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

#ifndef HMMWV_SIMPLEPOWERTRAIN_H
#define HMMWV_SIMPLEPOWERTRAIN_H

#include "subsys/powertrain/ChSimplePowertrain.h"
#include "subsys/ChVehicle.h"

namespace hmmwv {

// Forward reference
class chrono::ChVehicle;

class HMMWV_SimplePowertrain : public chrono::ChSimplePowertrain {
public:

  HMMWV_SimplePowertrain(chrono::ChVehicle* car);

  ~HMMWV_SimplePowertrain() {}

  virtual double GetConicTau() const  { return m_conic_tau; }
  virtual double GetGearTau() const   { return m_gear_tau; }
  virtual double GetMaxTorque() const { return m_max_torque; }
  virtual double GetMaxSpeed() const  { return m_max_speed; }

private:

  static const double m_conic_tau;   // the transmission ratio of the conic gears at the rear axle
  static const double m_gear_tau;    // the actual tau of the gear
  static const double m_max_torque;  // the max torque of the motor [Nm];
  static const double m_max_speed;   // the max rotation speed of the motor [rads/s]
};


} // end namespace hmmwv


#endif
