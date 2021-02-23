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
// Authors: Radu Serban
// =============================================================================
//
// FED-alpha shafts-based brake model.
//
// =============================================================================

#ifndef FEDA_BRAKE_SHAFTS_H
#define FEDA_BRAKE_SHAFTS_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace feda {

/// @addtogroup vehicle_models_feda
/// @{

/// Shafts-based FED-alpha brake subsystem (uses a clutch between two shafts).
class CH_MODELS_API FEDA_BrakeShafts : public ChBrakeShafts {
  public:
    FEDA_BrakeShafts(const std::string& name);
    ~FEDA_BrakeShafts() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }
    virtual double GetShaftInertia() override { return m_shaft_inertia; }

  private:
    static const double m_maxtorque;
    static const double m_shaft_inertia;
};

/// @} vehicle_models_feda

}  // end namespace feda
}  // end namespace vehicle
}  // end namespace chrono

#endif
