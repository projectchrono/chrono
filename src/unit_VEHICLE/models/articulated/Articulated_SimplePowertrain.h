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
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================
//
// Articulated simple powertrain model for a vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef ARTICULATED_SIMPLEPOWERTRAIN_H
#define ARTICULATED_SIMPLEPOWERTRAIN_H

#include "subsys/powertrain/ChSimplePowertrain.h"

class Articulated_SimplePowertrain : public chrono::ChSimplePowertrain
{
public:

  Articulated_SimplePowertrain() {}
  ~Articulated_SimplePowertrain() {}

  virtual double GetForwardGearRatio() const { return 0.3; }
  virtual double GetReverseGearRatio() const { return -0.3; }
  virtual double GetMaxTorque() const        { return 670.0; }
  virtual double GetMaxSpeed() const         { return 2000; }
};


#endif
