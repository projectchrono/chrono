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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// RCCar Pitman arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/rccar/RCCar_PitmanArm.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace rccar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;

const double RCCar_PitmanArm::m_steeringLinkMass = 0.01101740;
const double RCCar_PitmanArm::m_pitmanArmMass = 0.00524114;

const double RCCar_PitmanArm::m_steeringLinkRadius = 0.005;
const double RCCar_PitmanArm::m_pitmanArmRadius = 0.005;

const double RCCar_PitmanArm::m_maxAngle = 0.512; 

const ChVector<> RCCar_PitmanArm::m_steeringLinkInertiaMoments(0.00000560, 0.00000021, 0.00000569);
const ChVector<> RCCar_PitmanArm::m_steeringLinkInertiaProducts(0.0, 0.00000002, 0.0);

const ChVector<> RCCar_PitmanArm::m_pitmanArmInertiaMoments(0.00000041, 0.00000045, 0.00000032); 
const ChVector<> RCCar_PitmanArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RCCar_PitmanArm::RCCar_PitmanArm(const std::string& name) : ChPitmanArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> RCCar_PitmanArm::getLocation(PointId which) {
    switch (which) {
        case STEERINGLINK: //steering link COM
            return ChVector<>(.10163,-.01632,-.03162);
        case PITMANARM: // pitman arm COM
            return ChVector<>(.10163,.01632,-.03162);
        case REV: // pitman arm fixed point
            return ChVector<>(.09163,.01632,-.03162);
        case UNIV: //pitman arm moving end
            return ChVector<>(.1128,.01632,-.03162);
        case REVSPH_R: //idle arm fixed end
            return ChVector<>(.09163,-.01632,-.03162);
        case REVSPH_S: //idle arm moving end
            return ChVector<>(.1128,-.01632,-.03162);
        case TIEROD_PA:
            return ChVector<>(.1155, .033, -.027);
        case TIEROD_IA:
            return ChVector<>(.1155, -.033, -.027);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> RCCar_PitmanArm::getDirection(DirectionId which) {
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

}  // namespace rccar
}  // namespace vehicle
}  // namespace chrono
