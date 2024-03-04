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
// Authors: Daniel Melanz, Radu Serban, Asher Elmquist
// =============================================================================
//
// Sedan multi-link suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up and origin
// at the midpoint between the wheel centers.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_MultiLink.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Sedan_MultiLink::m_spindleMass = 1.103;
const double Sedan_MultiLink::m_upperArmMass = 4.744;
const double Sedan_MultiLink::m_lateralMass = 1.910;
const double Sedan_MultiLink::m_trailingLinkMass = 15.204;
const double Sedan_MultiLink::m_uprightMass = 3.201;

const double Sedan_MultiLink::m_spindleRadius = 0.1;
const double Sedan_MultiLink::m_spindleWidth = 0.03;
const double Sedan_MultiLink::m_upperArmRadius = 0.02;
const double Sedan_MultiLink::m_lateralRadius = 0.02;
const double Sedan_MultiLink::m_trailingLinkRadius = 0.03;
const double Sedan_MultiLink::m_uprightRadius = 0.02;

const ChVector<> Sedan_MultiLink::m_spindleInertia(0.000478, 0.000478, 0.000496);
const ChVector<> Sedan_MultiLink::m_upperArmInertia(0.0237, 0.0294, 0.00612);
const ChVector<> Sedan_MultiLink::m_lateralInertia(0.0543, 0.0541, 0.000279);
const ChVector<> Sedan_MultiLink::m_trailingLinkInertia(0.0762, 0.527, 0.567);
const ChVector<> Sedan_MultiLink::m_uprightInertia(0.0250, 0.00653, 0.0284);

const double Sedan_MultiLink::m_axleInertia = 0.166;

const double Sedan_MultiLink::m_springCoefficient = 167062.000;
const double Sedan_MultiLink::m_dampingCoefficient = 15000.0;//60068.000;
const double Sedan_MultiLink::m_springRestLength = 0.339;

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Sedan_MultiLink::Sedan_MultiLink(const std::string& name) : ChMultiLink(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient);
    m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
Sedan_MultiLink::~Sedan_MultiLink() {}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Sedan_MultiLink::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0.000, 0.8, 0.000);
        case UPRIGHT:
            return ChVector<>(0.000, 0.76, 0.000);
        case UA_F:
            return ChVector<>(0.060, 0.397, 0.082);
        case UA_B:
            return ChVector<>(-0.157, 0.358, 0.062);
        case UA_U:
            return ChVector<>(0.056, 0.714, 0.151);
        case UA_CM:
            return ChVector<>(-0.014, 0.490, 0.098);
        case LAT_C:
            return ChVector<>(0.036, 0.188, -0.133);
        case LAT_U:
            return ChVector<>(0.029, 0.692, -0.093);
        case LAT_CM:
            return ChVector<>(0.033, 0.440, -0.113);
        case TL_C:
            return ChVector<>(0.723, 0.449, -0.072);
        case TL_U:
            return ChVector<>(-0.000, 0.714, -0.156);
        case TL_CM:
            return ChVector<>(0.279, 0.543, -0.132);
        case SHOCK_C:
            return ChVector<>(0.171, 0.478, 0.315);
        case SHOCK_L:
            return ChVector<>(0.181, 0.519, -0.162);
        case SPRING_C:
            return ChVector<>(0.181, 0.491, 0.110);
        case SPRING_L:
            return ChVector<>(0.181, 0.419, -0.164);
        case TIEROD_C:
            return ChVector<>(-0.257, 0.170, -0.116);
        case TIEROD_U:
            return ChVector<>(-0.144, 0.712, -0.056);
        default:
            return ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// Implementation of the getDirection() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Sedan_MultiLink::getDirection(DirectionId which) {
    switch (which) {
        case UNIV_AXIS_LINK_TL:
            return ChVector<>(0, 0, 1);
        case UNIV_AXIS_CHASSIS_TL:
            return ChVector<>(0.272, 0.962, 0);
        case UNIV_AXIS_LINK_LAT:
            return ChVector<>(-0.978950, 0.204099, 0);
        case UNIV_AXIS_CHASSIS_LAT:
            return ChVector<>(-0.021990, -0.105472, 0.994179);
        default:
            return ChVector<>(0, 0, 1);
    }
}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
