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

#ifndef CH_SIMPLE_POWERTRAIN_H
#define CH_SIMPLE_POWERTRAIN_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChPowertrain.h"
#include "subsys/ChVehicle.h"
#include "subsys/ChSuspension.h"

namespace chrono {

// Forward reference
class ChVehicle;

class CH_SUBSYS_API ChSimplePowertrain : public ChPowertrain
{
public:

  ChSimplePowertrain(ChVehicle* car);

  ~ChSimplePowertrain() {}

  void Initialize(ChSharedPtr<ChBody> chassis,
                  ChSharedPtr<ChSuspension> rear_L_susp,
                  ChSharedPtr<ChSuspension> rear_R_susp);

  virtual double GetMotorSpeed() const { return m_motorSpeed; }
  virtual double GetMotorTorque() const { return m_motorTorque; }
  virtual double GetWheelTorque(ChWheelId which) const;

  virtual void Update(double time, double throttle);

protected:

  virtual double GetConicTau() const = 0;   // the transmission ratio of the conic gears at the rear axle
  virtual double GetGearTau() const = 0;    // the actual tau of the gear
  virtual double GetMaxTorque() const = 0;  // the max torque of the motor [Nm];
  virtual double GetMaxSpeed() const = 0;   // the max rotation speed of the motor [rads/s]

  ChSharedPtr<ChSuspension> m_rear_L_susp;
  ChSharedPtr<ChSuspension> m_rear_R_susp;

  double  m_motorSpeed;
  double  m_motorTorque;
  double  m_wheelTorque;

};


} // end namespace chrono


#endif
