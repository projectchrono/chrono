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
// Authors: Radu Serban, Jayne Henry, Luning Fang, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/artcar/ARTcar_EngineSimpleMap.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace artcar {

const double rpm2rads = CH_C_PI / 30;

ARTcar_EngineSimpleMap::ARTcar_EngineSimpleMap(const std::string& name)
    : ChEngineSimpleMap(name),
      m_voltage_ratio(1.0f),
      m_Kv_rating(1300),
      m_supply_voltage(7.4),
      m_stall_torque(0.7)  // TODO, currently a guess
{}

double ARTcar_EngineSimpleMap::GetMaxEngineSpeed() {
    return m_Kv_rating * m_supply_voltage * m_voltage_ratio * rpm2rads;
}

void ARTcar_EngineSimpleMap::SetEngineTorqueMaps(ChFunction_Recorder& map0, ChFunction_Recorder& mapF) {
    double max_rpm = m_Kv_rating * m_supply_voltage * m_voltage_ratio;

    // since this is a model of motor and ESC combination, we assume a linear relationship.
    // while brushless motors don't follow linear torque-speed relationship, most hobby electronic
    // speed controllers control the motor such that it approximately follows a linear relationship
    mapF.AddPoint(0, m_stall_torque);      // stall torque //TODO
    mapF.AddPoint(max_rpm * rpm2rads, 0);  // no load speed

    // N-m and rad/s
    map0.AddPoint(0, 0);
    // map0.AddPoint(.1 * max_rpm * rpm2rads, 0);
    // map0.AddPoint(max_rpm * rpm2rads, -stallTorque);  // TODO, currently a guess
    // map0.AddPoint(.1 * max_rpm * rpm2rads, 0);
    map0.AddPoint(max_rpm * rpm2rads, 0);  // TODO, currently a guess
}

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono
