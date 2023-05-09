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
// Simplified engine model constructed with data from file (JSON format).
//
// =============================================================================

#ifndef SIMPLE_ENGINE_H
#define SIMPLE_ENGINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/powertrain/ChEngineSimple.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Simple engine subsystem (specified through JSON file).
class CH_VEHICLE_API EngineSimple : public ChEngineSimple {
  public:
    EngineSimple(const std::string& filename);
    EngineSimple(const rapidjson::Document& d);
    ~EngineSimple() {}

    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_max_torque;  ///< maximum motor torque
    double m_max_power;   ///< maximum motor power
    double m_max_speed;   ///< maximum engine speed
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
