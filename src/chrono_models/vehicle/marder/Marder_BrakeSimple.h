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
// Marder simple brake model
//
// =============================================================================

#ifndef MARDER_BRAKE_SIMPLE_H
#define MARDER_BRAKE_SIMPLE_H

#include "chrono_vehicle/tracked_vehicle/brake/ChTrackBrakeSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Simple M113 brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API Marder_BrakeSimple : public ChTrackBrakeSimple {
  public:
    Marder_BrakeSimple(const std::string& name) : ChTrackBrakeSimple(name) {}
    ~Marder_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return 30000.0; }
};

/// @} vehicle_models_marder

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
