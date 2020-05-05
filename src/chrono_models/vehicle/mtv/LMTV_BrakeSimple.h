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
// LMTV simple brake models (front and rear).
//
// =============================================================================

#ifndef LMTV_BRAKESIMPLE_H
#define LMTV_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace mtv {

/// @addtogroup vehicle_models_mtv
/// @{

/// Simple MAN brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API LMTV_BrakeSimple : public ChBrakeSimple {
  public:
    LMTV_BrakeSimple(const std::string& name);
    virtual ~LMTV_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// @} vehicle_models_mtv

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
