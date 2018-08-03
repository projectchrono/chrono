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
// Authors: Alessandro Tasora, Asher Elmquist
// =============================================================================
//
// Sedan simple brake models (front and rear).
//
// =============================================================================

#ifndef SEDAN_BRAKESIMPLE_H
#define SEDAN_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace sedan {

/// @addtogroup vehicle_models_sedan
/// @{

/// Simple Sedan brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API Sedan_BrakeSimple : public ChBrakeSimple {
  public:
    Sedan_BrakeSimple(const std::string& name);
    virtual ~Sedan_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// @} vehicle_models_sedan

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono

#endif
