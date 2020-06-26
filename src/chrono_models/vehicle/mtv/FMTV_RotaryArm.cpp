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
// FMTV rotary arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/FMTV_RotaryArm.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FMTV_RotaryArm::m_pitmanArmMass = 1.605;
const double FMTV_RotaryArm::m_pitmanArmRadius = 0.02;
const double FMTV_RotaryArm::m_maxAngle = 22.7 * (CH_C_PI / 180);
const ChVector<> FMTV_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector<> FMTV_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

FMTV_RotaryArm::FMTV_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

const ChVector<> FMTV_RotaryArm::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector<>(1.0, 0.708341392, 0.1);
        case ARM_C:
            return ChVector<>(1.0, 0.708341392, 0.3);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> FMTV_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(0, 1, 0);
        default:
            return ChVector<>(0, 1, 0);
    }
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
