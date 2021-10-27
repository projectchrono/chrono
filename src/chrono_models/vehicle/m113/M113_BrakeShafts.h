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
// M113 shafts-based brake model
//
// =============================================================================

#ifndef M113_BRAKE_SHAFTS_H
#define M113_BRAKE_SHAFTS_H

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeShafts.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Shafts-based M113 brake subsystem (uses a clutch between two shafts).
class CH_MODELS_API M113_BrakeShafts : public ChTrackBrakeShafts {
  public:
    M113_BrakeShafts(const std::string& name) : ChTrackBrakeShafts(name) {}
    ~M113_BrakeShafts() {}

    virtual double GetMaxBrakingTorque() override { return 10000.0; }
    virtual double GetShaftInertia() override { return 0.4; }
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
