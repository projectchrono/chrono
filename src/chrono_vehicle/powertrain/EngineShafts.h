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
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/powertrain/ChEngineShafts.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Shafts-based engine subsystem (specified through JSON file).
class CH_VEHICLE_API EngineShafts : public ChEngineShafts {
  public:
    EngineShafts(const std::string& filename);
    EngineShafts(const rapidjson::Document& d);
    ~EngineShafts() {}

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }

    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) override;

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_motorblock_inertia;
    double m_motorshaft_inertia;

    ChMapData m_engine_torque;
    ChMapData m_engine_losses;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
