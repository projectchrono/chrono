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
// ChShaft-based engine model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef SHAFTS_ENGINE_H
#define SHAFTS_ENGINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/powertrain/ChShaftsEngine.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Shafts-based engine subsystem (specified through JSON file).
class CH_VEHICLE_API ShaftsEngine : public ChShaftsEngine {
  public:
    ShaftsEngine(const std::string& filename);
    ShaftsEngine(const rapidjson::Document& d);
    ~ShaftsEngine() {}

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetCrankshaftInertia() const override { return m_crankshaft_inertia; }

    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) override;

  private:
    struct MapData {
        unsigned int m_n;
        std::vector<double> m_x;
        std::vector<double> m_y;
    };

    virtual void Create(const rapidjson::Document& d) override;

    void ReadMapData(const rapidjson::Value& a, MapData& map_data);
    void SetMapData(const MapData& map_data,
                    double x_factor,
                    double y_factor,
                    std::shared_ptr<ChFunction_Recorder>& map);

    double m_motorblock_inertia;
    double m_crankshaft_inertia;

    MapData m_engine_torque;
    MapData m_engine_losses;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
