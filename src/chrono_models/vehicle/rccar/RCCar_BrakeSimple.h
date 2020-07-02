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
// Authors: Alessandro Tasora, Jayne Henry 
// =============================================================================
//
// RCCar simple brake models (front and rear).
//
// =============================================================================

#ifndef RCCAR_BRAKESIMPLE_H
#define RCCAR_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace rccar {

/// @addtogroup vehicle_models_rccar
/// @{

/// Simple RCCar brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API RCCar_BrakeSimple : public chrono::vehicle::ChBrakeSimple {
  public:
    RCCar_BrakeSimple(const std::string& name);
    virtual ~RCCar_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// @} vehicle_models_rccar

}  // namespace rccar
}  // namespace vehicle
}  // namespace chrono

#endif
