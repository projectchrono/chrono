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
// Generic simple brake model
//
// =============================================================================

#ifndef GENERIC_BRAKESIMPLE_H
#define GENERIC_BRAKESIMPLE_H

#include "subsys/brake/ChBrakeSimple.h"

class  Generic_BrakeSimple : public chrono::ChBrakeSimple
{
public:

  Generic_BrakeSimple() {}
  ~Generic_BrakeSimple() {}

  virtual double GetMaxBrakingTorque() { return 4000.0; }
};


#endif
