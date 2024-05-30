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
// Authors: Radu Serban
// =============================================================================
//
// Automatic transmssion model for the HMMWV vehicle.
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#ifndef CHRONO_CHEROKEE_AUTOMATICTRANSMISSIONSIMPLEMAP_H
#define CHRONO_CHEROKEE_AUTOMATICTRANSMISSIONSIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_jeep
/// @{

/// HMMWV automatic transmission model template based on a simple gear-shifting model.
class CH_MODELS_API Cherokee_AutomaticTransmissionSimpleMap : public ChAutomaticTransmissionSimpleMap {
  public:
    Cherokee_AutomaticTransmissionSimpleMap(const std::string& name);
    ~Cherokee_AutomaticTransmissionSimpleMap() {}

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify the min and max engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;
};

/// @} vehicle_models_hmmwv

}  // namespace jeep
}  // end namespace vehicle
}  // end namespace chrono

#endif
