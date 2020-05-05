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
// LMTV rotary arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/LMTV_RotaryArm.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double LMTV_RotaryArm::m_pitmanArmMass = 1.605;

const double LMTV_RotaryArm::m_pitmanArmRadius = 0.02;

const double LMTV_RotaryArm::m_maxAngle = 22.7 * (CH_C_PI / 180);

const ChVector<> LMTV_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector<> LMTV_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LMTV_RotaryArm::LMTV_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> LMTV_RotaryArm::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector<>(1.0, 0.708341392, 0.1);
        case ARM_C:
            return ChVector<>(1.0, 0.708341392, 0.3);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> LMTV_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(0, 1, 0);
        default:
            return ChVector<>(0, 1, 0);
    }
}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
