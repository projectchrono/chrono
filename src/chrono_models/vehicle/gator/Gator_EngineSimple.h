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
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Simple engine model for the Gator vehicle.
// - linear speed-torque curve
//
// =============================================================================

#ifndef GATOR_ENGINESIMPLE_H
#define GATOR_ENGINESIMPLE_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChEngineSimple.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gator {

/// @addtogroup vehicle_models_gator
/// @{

/// Simple Gator powertrain subsystem (DC motor linear torque-speed characteristic).
class CH_MODELS_API Gator_EngineSimple : public ChEngineSimple {
  public:
    Gator_EngineSimple(const std::string& name);
    ~Gator_EngineSimple() {}

  protected:
    double GetMaxTorque() const override { return m_max_torque; }
    double GetMaxPower() const override { return m_max_power; }
    double GetMaxSpeed() const override { return m_max_speed; }

  private:
    static const double m_max_torque;  ///< maximum motor torque
    static const double m_max_power;   ///< maximum motor power
    static const double m_max_speed;   ///< maximum engine speed
};

/// @} vehicle_models_gator

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono

#endif
