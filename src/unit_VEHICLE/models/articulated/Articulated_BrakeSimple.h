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
// Authors: Alessandro Tasora
// =============================================================================
//
// Articulated simple brake model
//
// =============================================================================

#ifndef ARTICULATED_BRAKESIMPLE_H
#define ARTICULATED_BRAKESIMPLE_H

#include "subsys/brake/ChBrakeSimple.h"

class  Articulated_BrakeSimple : public chrono::ChBrakeSimple
{
public:

  Articulated_BrakeSimple() {}
  ~Articulated_BrakeSimple() {}

  virtual double GetMaxBrakingTorque() { return 4000.0; }
};


#endif
