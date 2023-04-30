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
// - no torque converter
// - no transmission box
//
// =============================================================================

#ifndef GENERIC_AUTOMATICTRANSMISSIONSIMPLE_H
#define GENERIC_AUTOMATICTRANSMISSIONSIMPLE_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Simple transmission model for the generic vehicle.
class CH_MODELS_API Generic_AutomaticTransmissionSimple : public ChAutomaticTransmissionSimpleMap {
  public:
    Generic_AutomaticTransmissionSimple(const std::string& name);
    void SetGearRatios(std::vector<double>& fwd, double& rev) override;
    void SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) override;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
