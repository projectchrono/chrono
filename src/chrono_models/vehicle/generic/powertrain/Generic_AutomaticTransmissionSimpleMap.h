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
// Authors: Radu Serban, Mike Taylor, Marcel Offermans
// =============================================================================
//
// Simple automatic gearbox model for the generic vehicle.
//
// =============================================================================

#ifndef GENERIC_AUTOMATIC_TRANSMISSION_SIMPLEMAP_H
#define GENERIC_AUTOMATIC_TRANSMISSION_SIMPLEMAP_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono/core/ChCubicSpline.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Custom automatic transmission model for a generic vehicle.
class CH_MODELS_API Generic_AutomaticTransmissionSimpleMap : public ChAutomaticTransmissionSimpleMap {
  public:
    Generic_AutomaticTransmissionSimpleMap(const std::string& name);

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    void SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) override;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
