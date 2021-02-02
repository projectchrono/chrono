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
// Authors: Rainer Gericke
// =============================================================================
//
// Marder shafts-based brake model
//
// =============================================================================

#ifndef MARDER_BRAKE_SHAFTS_H
#define MARDER_BRAKE_SHAFTS_H

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Shafts-based Marder brake subsystem (uses a clutch between two shafts).
class CH_MODELS_API Marder_BrakeShafts : public ChTrackBrakeShafts {
  public:
    Marder_BrakeShafts(const std::string& name) : ChTrackBrakeShafts(name) {}
    ~Marder_BrakeShafts() {}

    virtual double GetMaxBrakingTorque() override { return 30000.0; }
    virtual double GetShaftInertia() override { return 0.4; }
};

/// @} vehicle_models_marder

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
