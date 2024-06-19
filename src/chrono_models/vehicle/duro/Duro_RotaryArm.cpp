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

const double Duro_RotaryArm::m_pitmanArmRadius = 0.03;

const double Duro_RotaryArm::m_maxAngle = 12.5 * (CH_PI / 180);

const ChVector3d Duro_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector3d Duro_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Duro_RotaryArm::Duro_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector3d Duro_RotaryArm::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector3d(0.22, 0.59, 0.1);
        case ARM_C:
            return ChVector3d(0.22, 0.59, 0.3);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d Duro_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector3d(-1, 0, 0);
        default:
            return ChVector3d(-1, 0, 0);
    }
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
