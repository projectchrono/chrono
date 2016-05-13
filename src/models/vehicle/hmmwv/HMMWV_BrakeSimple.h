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
// HMMWV simple brake models (front and rear).
//
// =============================================================================

#ifndef HMMWV_BRAKESIMPLE_H
#define HMMWV_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "models/ChApiModels.h"

namespace hmmwv {

class CH_MODELS_API HMMWV_BrakeSimple : public chrono::vehicle::ChBrakeSimple {
  public:
    HMMWV_BrakeSimple();
    virtual ~HMMWV_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

}  // end namespace chrono

#endif
