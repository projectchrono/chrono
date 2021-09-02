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

const double RCCar_PitmanArm::m_steeringLinkMass = lb2kg * 0.25; //TODO
const double RCCar_PitmanArm::m_pitmanArmMass = lb2kg * 0.15;//TODO

const double RCCar_PitmanArm::m_steeringLinkRadius = in2m * 0.1;//TODO
const double RCCar_PitmanArm::m_pitmanArmRadius = in2m * 0.2;//TODO

const double RCCar_PitmanArm::m_maxAngle = 0.3448;//TODO

const ChVector<> RCCar_PitmanArm::m_steeringLinkInertiaMoments(0.00022, 0.00022, 0.0000029);//TODO
const ChVector<> RCCar_PitmanArm::m_steeringLinkInertiaProducts(0.0, 0.0, 0.0);//TODO
const ChVector<> RCCar_PitmanArm::m_pitmanArmInertiaMoments(0.00002, 0.00002, 0.00002);//TODO
const ChVector<> RCCar_PitmanArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);//TODO

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RCCar_PitmanArm::RCCar_PitmanArm(const std::string& name) : ChPitmanArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> RCCar_PitmanArm::getLocation(PointId which) {
    switch (which) {
        case STEERINGLINK:
            return in2m * ChVector<>(-1.2015, 0, -.9225);//TODO
        case PITMANARM:
            return in2m * ChVector<>(-1.6460, -0.6395, -.9225);//TODO
        case REV:
            return in2m * ChVector<>(-2.0985, -0.6395, -.4205);//TODO
        case UNIV:
            return in2m * ChVector<>(-1.2015, -0.6395, -.8345);//TODO
        case REVSPH_R:
            return in2m * ChVector<>(-2.0985, 0.6395, -.4205);//TODO
        case REVSPH_S:
            return in2m * ChVector<>(-1.2015, 0.6395, -.8345);//TODO
        case TIEROD_PA:
            return in2m * ChVector<>(-1.0770, -1.2760, -.6855);//TODO
        case TIEROD_IA:
            return in2m * ChVector<>(-1.0770, 1.2760, -.6855);//TODO
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> RCCar_PitmanArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(0, 0, 1);//TODO
        case UNIV_AXIS_ARM:
            return ChVector<>(0, 0, 1);//TODO
        case UNIV_AXIS_LINK:
            return ChVector<>(1, 0, 0);//TODO
        case REVSPH_AXIS:
            return ChVector<>(0, 0, 1);//TODO
        default:
            return ChVector<>(0, 0, 1);//TODO
    }
}

}  // namespace rccar
}  // namespace vehicle
}  // namespace chrono
