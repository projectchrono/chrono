// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Generic simple powertrain model for a vehicle.
// - trivial speed-torque curve
//
// =============================================================================

#ifndef GENERIC_ENGINESIMPLE_H
#define GENERIC_ENGINESIMPLE_H

#include "chrono_vehicle/powertrain/ChEngineSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Simple engine model for the generic vehicle (purely kinematic).
class CH_MODELS_API Generic_EngineSimple : public ChEngineSimple {
  public:
    Generic_EngineSimple(const std::string& name);

    double GetMaxTorque() const override { return 365.0; }
    double GetMaxSpeed() const override { return 5000.0; }
    double GetMaxPower() const override { return 60000.0; } // TODO verify (~80bhp)
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
