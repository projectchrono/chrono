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
// Krone ProfiLiner SP5 simple brake models (all axles).
//
// =============================================================================

#ifndef SEMITRAILOR_BRAKESIMPLE_H
#define SEMITRAILOR_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

class SemiTrailer_brake : public chrono::vehicle::ChBrakeSimple {
  public:
    SemiTrailer_brake(const std::string& name);
    virtual ~SemiTrailer_brake() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

#endif
