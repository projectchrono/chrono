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
// M113 simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#ifndef M113_ENGINE_SIMPLE_H
#define M113_ENGINE_SIMPLE_H

#include "chrono_vehicle/powertrain/EngineSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// M113 simple engine model based on hyperbolical speed-torque curve (CVT).
class CH_MODELS_API M113_EngineSimple : public ChEngineSimple {
  public:
    M113_EngineSimple(const std::string& name);

    ~M113_EngineSimple() {}

    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_max_torque;  ///< maximum motor torque
    static const double m_max_power;   ///< maximum motor power
    static const double m_max_speed;   ///< maximum engine speed
};

/// @} vehicle_models_m113

}  // end namespace M113
}  // end namespace vehicle
}  // end namespace chrono

#endif

