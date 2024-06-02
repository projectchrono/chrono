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
// Authors: Radu Serban, Asher Elmquist, Marcel Offermans, Rainer Gericke
// data from http://www.treffseiten.de/bmw/info/daten_320i_325i_330i_320d_limousine_05_09.pdf
// Gear ratios from automatic gearbox version
// =============================================================================
//
// Simple automatic transmission model for the BMW E90 (330i 2006) vehicle.
// - no torque converter
// - simple gear-shifting model (in automatic mode)
// data from http://www.treffseiten.de/bmw/info/daten_320i_325i_330i_320d_limousine_05_09.pdf
// Gear ratios from automatic gearbox version
//
// =============================================================================

#ifndef BMW_E90_AUTOMATICTRANSMISSIONSIMPLEMAP_H
#define BMW_E90_AUTOMATICTRANSMISSIONSIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Simple Sedan automatic transmission subsystem.
class CH_MODELS_API BMW_E90_AutomaticTransmissionSimpleMap : public ChAutomaticTransmissionSimpleMap {
  public:
    BMW_E90_AutomaticTransmissionSimpleMap(const std::string& name);

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    void SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
                        ) override;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
