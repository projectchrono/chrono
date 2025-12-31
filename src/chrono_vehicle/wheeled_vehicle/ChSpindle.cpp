// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Definition of a wheeled vehicle spindle.
// Each suspension subsystem creates and maintains two spindles, one for each
// side. A spindle object is a ChBody with an associated force/torque accumulator
// for applying tire/terrain forces.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/ChSpindle.h"

namespace chrono {
namespace vehicle {

ChSpindle::ChSpindle() : ChBody() {
    m_tire_accumulator_index = AddAccumulator();
}

void ChSpindle::AddTireAccumulator() {
    m_tire_accumulator_index = AddAccumulator();
}

void ChSpindle::EmptyTireAccumulator() {
    EmptyAccumulator(m_tire_accumulator_index);
}

void ChSpindle::AccumulateTireForce(const TerrainForce& tire_force) {
    AccumulateForce(m_tire_accumulator_index, tire_force.force, tire_force.point, false);
    AccumulateTorque(m_tire_accumulator_index, tire_force.moment, false);
}

}  // end namespace vehicle
}  // end namespace chrono
