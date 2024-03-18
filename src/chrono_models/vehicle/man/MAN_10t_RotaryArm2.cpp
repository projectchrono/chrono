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
// MAN 10t 2nd axle rotary arm steering model.
// The wheels are running on a greater radius compared to the wheels on the
// first axle, m_maxAngle must be reduced
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_10t_RotaryArm2.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_10t_RotaryArm2::m_pitmanArmMass = 1.605;

const double MAN_10t_RotaryArm2::m_pitmanArmRadius = 0.02;

const double MAN_10t_RotaryArm2::m_maxAngle = 28.37 * (CH_PI / 180);

const ChVector3d MAN_10t_RotaryArm2::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector3d MAN_10t_RotaryArm2::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_10t_RotaryArm2::MAN_10t_RotaryArm2(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector3d MAN_10t_RotaryArm2::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector3d(0.741, -0.217, 0.089);
        case ARM_C:
            return ChVector3d(0.850, -0.028, 0.166);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d MAN_10t_RotaryArm2::getDirection(DirectionId which) {
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
