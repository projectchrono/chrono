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
// Authors: Radu Serban
// =============================================================================
//
// Generic simple powertrain model for a vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef GENERIC_SIMPLEPOWERTRAIN_H
#define GENERIC_SIMPLEPOWERTRAIN_H

#include "subsys/powertrain/ChSimplePowertrain.h"

class Generic_SimplePowertrain : public chrono::ChSimplePowertrain
{
public:

  Generic_SimplePowertrain() {}
  ~Generic_SimplePowertrain() {}

  virtual double GetForwardGearRatio() const { return 0.3; }
  virtual double GetReverseGearRatio() const { return -0.3; }
  virtual double GetMaxTorque() const        { return 270.0; }
  virtual double GetMaxSpeed() const         { return 2000; }
};


#endif
