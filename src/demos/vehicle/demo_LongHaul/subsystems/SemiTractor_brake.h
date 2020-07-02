// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Rainer Gericke
// =============================================================================
//
// Kraz 64431 simple brake models (front and rear).
//
// =============================================================================

#ifndef SEMITRACTOR_BRAKESIMPLE_H
#define SEMITRACTOR_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

class SemiTractor_brake : public chrono::vehicle::ChBrakeSimple {
  public:
    SemiTractor_brake(const std::string& name);
    virtual ~SemiTractor_brake() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

#endif
