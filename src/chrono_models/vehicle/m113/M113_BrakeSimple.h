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
// M113 simple brake model
//
// =============================================================================

#ifndef M113_BRAKESIMPLE_H
#define M113_BRAKESIMPLE_H

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Simple M113 brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API M113_BrakeSimple : public ChTrackBrakeSimple {
  public:
    M113_BrakeSimple(const std::string& name) : ChTrackBrakeSimple(name) {}
    ~M113_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return 10000.0; }
};

/// @} vehicle_models_m113

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
