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
// MAN 5t rotary arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_5t_RotaryArm.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_5t_RotaryArm::m_pitmanArmMass = 1.605;

const double MAN_5t_RotaryArm::m_pitmanArmRadius = 0.02;

const double MAN_5t_RotaryArm::m_maxAngle = 39.0 * (CH_PI / 180);

const ChVector3d MAN_5t_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector3d MAN_5t_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_RotaryArm::MAN_5t_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector3d MAN_5t_RotaryArm::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector3d(0.741, -0.217, 0.089);
        case ARM_C:
            return ChVector3d(0.850, -0.028, 0.166);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d MAN_5t_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector3d(0, 0, 1);
        default:
            return ChVector3d(0, 0, 1);
    }
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
