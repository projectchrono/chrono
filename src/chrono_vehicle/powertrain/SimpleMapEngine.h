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
#include "chrono_vehicle/powertrain/ChSimpleMapEngine.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Simple speed-torque engine map subsystem (specified through JSON file).
class CH_VEHICLE_API SimpleMapEngine : public ChSimpleMapEngine {
  public:
    SimpleMapEngine(const std::string& filename);
    SimpleMapEngine(const rapidjson::Document& d);
    ~SimpleMapEngine() {}

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override { return m_max_engine_speed; }

    /// Set the maps for engine characteristics
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                                     ) override;

  private:
    struct MapData {
        unsigned int m_n;
        std::vector<double> m_x;
        std::vector<double> m_y;
    };

    virtual void Create(const rapidjson::Document& d) override;

    void ReadMapData(const rapidjson::Value& a, MapData& map_data);
    void SetMapData(const MapData& map_data, std::shared_ptr<ChFunction_Recorder>& map);

    double m_max_engine_speed;

    MapData m_engine_map_full;
    MapData m_engine_map_zero;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
