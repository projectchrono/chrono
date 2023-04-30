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
// Authors: Radu Serban
// =============================================================================
//
// HMMWV Pitman arm steering model with compliant steering column.
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/steering/HMMWV_PitmanArmShafts.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_PitmanArmShafts::m_steeringLinkMass = 3.6814844109;
const double HMMWV_PitmanArmShafts::m_pitmanArmMass = 1.605;

const double HMMWV_PitmanArmShafts::m_steeringLinkRadius = 0.03;
const double HMMWV_PitmanArmShafts::m_pitmanArmRadius = 0.02;

//// NOTE: maxAngle (at steering wheel) and gear ratio set such that max angle at revolute = 30
const double HMMWV_PitmanArmShafts::m_maxAngle = 780.0 * (CH_C_PI / 180);
const double HMMWV_PitmanArmShafts::m_gearRatio = 780.0 / 30.0;
//// NOTE: steering compliance 2000 N.mm/deg (too low?)
const double HMMWV_PitmanArmShafts::m_steeringCompliance = 2 * CH_C_RAD_TO_DEG;
//// NOTE: shaft inertia approximated (likely too high)
const double HMMWV_PitmanArmShafts::m_columnInertia = 5e-2;

const ChVector<> HMMWV_PitmanArmShafts::m_steeringLinkInertiaMoments(0.252, 0.00233, 0.254);
const ChVector<> HMMWV_PitmanArmShafts::m_steeringLinkInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> HMMWV_PitmanArmShafts::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector<> HMMWV_PitmanArmShafts::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_PitmanArmShafts::HMMWV_PitmanArmShafts(const std::string& name, bool rigid_connection)
    : ChPitmanArmShafts(name, false, rigid_connection) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> HMMWV_PitmanArmShafts::getLocation(PointId which) {
    switch (which) {
        case STEERINGLINK:
            return ChVector<>(0.129, 0, 0);
        case PITMANARM:
            return ChVector<>(0.064, 0.249, 0);
        case REV:
            return ChVector<>(0, 0.249, 0);
        case UNIV:
            return ChVector<>(0.129, 0.249, 0);
        case REVSPH_R:
            return ChVector<>(0, -0.325, 0);
        case REVSPH_S:
            return ChVector<>(0.129, -0.325, 0);
        case TIEROD_PA:
            return ChVector<>(0.195, 0.448, 0.035);
        case TIEROD_IA:
            return ChVector<>(0.195, -0.448, 0.035);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> HMMWV_PitmanArmShafts::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(0, 0, 1);
        case UNIV_AXIS_ARM:
            return ChVector<>(0, 0, 1);
        case UNIV_AXIS_LINK:
            return ChVector<>(1, 0, 0);
        case REVSPH_AXIS:
            return ChVector<>(0, 0, 1);
        default:
            return ChVector<>(0, 0, 1);
    }
}

}  // namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
