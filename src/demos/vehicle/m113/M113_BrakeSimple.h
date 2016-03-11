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
// M113 simple brake model
//
// =============================================================================

#ifndef M113_BRAKESIMPLE_H
#define M113_BRAKESIMPLE_H

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

class M113_BrakeSimple : public chrono::vehicle::ChTrackBrakeSimple {
  public:
    M113_BrakeSimple() {}
    ~M113_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return 10000.0; }
};

#endif
