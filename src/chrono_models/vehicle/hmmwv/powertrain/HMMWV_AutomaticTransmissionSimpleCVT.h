// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Automatic Continuous Variable Transmission (CVT) model for the HMMWV vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-varying model (linearly dependend on driveshaft angular velocity)
// - simple optional consideration of efficiency
//
// =============================================================================

#ifndef HMMWV_AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H
#define HMMWV_AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleCVT.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV_AutomaticTransmissionSimpleCVT : public ChAutomaticTransmissionSimpleCVT {
  public:
    HMMWV_AutomaticTransmissionSimpleCVT(const std::string& name);
    ~HMMWV_AutomaticTransmissionSimpleCVT() {}
};

}  // namespace hmmwv
}  // namespace vehicle
}  // namespace chrono

#endif  // HMMWV_AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H
