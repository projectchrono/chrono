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
// FMTV simple engine model based on hyperbolical speed-torque curve (CVT)
//
// =============================================================================

#ifndef FMTV_ENGINE_SIMPLE_H
#define FMTV_ENGINE_SIMPLE_H

#include "chrono_vehicle/powertrain/EngineSimple.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// FMTV simple engine model based on hyperbolical speed-torque curve (CVT).
class CH_MODELS_API FMTV_EngineSimple : public ChEngineSimple {
  public:
    FMTV_EngineSimple(const std::string& name);

    ~FMTV_EngineSimple() {}

    virtual double GetMaxTorque() const override { return m_max_torque; }
    virtual double GetMaxPower() const override { return m_max_power; }
    virtual double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_max_torque;  ///< maximum motor torque
    static const double m_max_power;   ///< maximum motor power
    static const double m_max_speed;   ///< maximum engine speed
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
