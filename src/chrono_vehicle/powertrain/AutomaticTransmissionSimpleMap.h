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
// JSON specification of an automatic transmission model template based on a
// simple gear-shifting model.
//
// =============================================================================

#ifndef SIMPLEMAP_AUTOMATIC_TRANSMISSION_H
#define SIMPLEMAP_AUTOMATIC_TRANSMISSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Automatic transmission model template based on a simple gear-shifting model (specified through JSON file).
class CH_VEHICLE_API AutomaticTransmissionSimpleMap : public ChAutomaticTransmissionSimpleMap {
  public:
    AutomaticTransmissionSimpleMap(const std::string& filename);
    AutomaticTransmissionSimpleMap(const rapidjson::Document& d);
    ~AutomaticTransmissionSimpleMap() {}

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify the min and max engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) override;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_rev_gear;
    std::vector<double> m_fwd_gear;

    ChMapData m_shift_bands;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
