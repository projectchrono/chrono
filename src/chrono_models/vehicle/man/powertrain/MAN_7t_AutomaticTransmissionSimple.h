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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple automatic transmission to use EngineSimpleMap.
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#ifndef MAN_7T_AUTOMATIC_TRANSMISSION_SIMPLE_H
#define MAN_7T_AUTOMATIC_TRANSMISSION_SIMPLE_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

/// Simple MAN_7t powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API MAN_7t_AutomaticTransmissionSimple : public ChAutomaticTransmissionSimpleMap {
  public:
    MAN_7t_AutomaticTransmissionSimple(const std::string& name);

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
