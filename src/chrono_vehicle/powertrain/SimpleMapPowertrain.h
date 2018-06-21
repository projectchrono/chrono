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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// JSON specification of a simple powertrain model.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
// - configured via JSON file
//
// =============================================================================

#ifndef SIMPLEMAP_POWERTRAIN_H
#define SIMPLEMAP_POWERTRAIN_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/powertrain/ChSimpleMapPowertrain.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Simple powertrain subsystem (based on engine speed-torque maps via JSON file).
class CH_VEHICLE_API SimpleMapPowertrain : public ChSimpleMapPowertrain {
  public:
    SimpleMapPowertrain(const std::string& filename);
    SimpleMapPowertrain(const rapidjson::Document& d);
    ~SimpleMapPowertrain() {}

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() override { return m_max_engine_speed; }

    /// Set the gears, i.e. the transmission ratios of the various gears.
    /// A concrete class must populate the vector of forward gear ratios, ordered as 1st, 2nd, etc.
    /// and provide a value for the single reverse gear ratio.
    virtual void SetGearRatios(std::vector<double>& fwd_gear_ratios,  ///< [out] list of forward gear ratios
                               double& reverse_gear_ratio             ///< [out] single reverse gear ratio
                               ) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;

    /// Set the Maps for engine characteristics
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

    double m_rev_gear;
    std::vector<double> m_fwd_gear;

    MapData m_engine_torque;
    MapData m_engine_losses;
    MapData m_shift_bands;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
