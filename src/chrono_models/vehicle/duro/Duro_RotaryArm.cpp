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
// Authors: Rainer Gericke
// =============================================================================
//
// Duro rotary arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/duro/Duro_RotaryArm.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Duro_RotaryArm::m_pitmanArmMass = 1.605;

const double Duro_RotaryArm::m_pitmanArmRadius = 0.02;

const double Duro_RotaryArm::m_maxAngle = 12.5 * (CH_C_PI / 180);

const ChVector<> Duro_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector<> Duro_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Duro_RotaryArm::Duro_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> Duro_RotaryArm::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector<>(0.22, 0.635 - 0.2 + 0.155, 0.1);
        case ARM_C:
            return ChVector<>(0.22, 0.635 - 0.2 + 0.155, 0.3);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> Duro_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(-1, 0, 0);
        default:
            return ChVector<>(-1, 0, 0);
    }
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
