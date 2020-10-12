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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Kraz 64431 simple brake models (front and rear).
//
// =============================================================================

#ifndef KRAZ_TRACTOR_BRAKE_H
#define KRAZ_TRACTOR_BRAKE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// @addtogroup vehicle_models_kraz
/// @{

/// Simple Kraz tractor brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API Kraz_tractor_Brake : public ChBrakeSimple {
  public:
    Kraz_tractor_Brake(const std::string& name);
    ~Kraz_tractor_Brake() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// @} vehicle_models_kraz

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
