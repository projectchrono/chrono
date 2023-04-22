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
// JSON specification of a simple engine model based on torque-speed engine maps
//
// =============================================================================

#ifndef SIMPLEMAP_ENGINE_H
#define SIMPLEMAP_ENGINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Simple speed-torque engine map subsystem (specified through JSON file).
class CH_VEHICLE_API EngineSimpleMap : public ChEngineSimpleMap {
  public:
    EngineSimpleMap(const std::string& filename);
    EngineSimpleMap(const rapidjson::Document& d);
    ~EngineSimpleMap() {}

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override { return m_max_engine_speed; }

    /// Set the maps for engine characteristics
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                                     ) override;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_max_engine_speed;

    ChMapData m_engine_map_full;
    ChMapData m_engine_map_zero;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
