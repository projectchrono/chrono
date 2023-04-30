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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Generic solid axle suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origin at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/generic/suspension/Generic_SolidAxle.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Generic_SolidAxle::m_ULMass = 1.446;
const double Generic_SolidAxle::m_LLMass = 2.892;
const double Generic_SolidAxle::m_knuckleMass = 1.356;
const double Generic_SolidAxle::m_spindleMass = 0.248;
const double Generic_SolidAxle::m_axleTubeMass = 44.958;
const double Generic_SolidAxle::m_tierodMass = 1.633;
const double Generic_SolidAxle::m_draglinkMass = 0.464;
const double Generic_SolidAxle::m_bellCrankMass = 0.218;

const double Generic_SolidAxle::m_spindleRadius = 0.06;
const double Generic_SolidAxle::m_spindleWidth = 0.04;
const double Generic_SolidAxle::m_ULRadius = 0.02;
const double Generic_SolidAxle::m_LLRadius = 0.02;
const double Generic_SolidAxle::m_axleTubeRadius = 0.03;
const double Generic_SolidAxle::m_knuckleRadius = 0.01;
const double Generic_SolidAxle::m_tierodRadius = 0.007;
const double Generic_SolidAxle::m_draglinkRadius = 0.007;
const double Generic_SolidAxle::m_bellCrankRadius = 0.007;

const ChVector<> Generic_SolidAxle::m_axleTubeCOM(0, 0, 0);

const ChVector<> Generic_SolidAxle::m_axleTubeInertia(7.744, 0.045, 7.744);
const ChVector<> Generic_SolidAxle::m_spindleInertia(0.0000558, 0.0000279, 0.0000558);
const ChVector<> Generic_SolidAxle::m_ULInertia(0.011, 0.011, 0.000142);
const ChVector<> Generic_SolidAxle::m_LLInertia(0.0514, 0.0514, 0.00037);
const ChVector<> Generic_SolidAxle::m_knuckleInertia(0.00255, 0.00134, 0.00196);
const ChVector<> Generic_SolidAxle::m_tierodInertia(0.252, 0.001, 0.252);
const ChVector<> Generic_SolidAxle::m_draglinkInertia(0.005, 0.005, 0.001);
const ChVector<> Generic_SolidAxle::m_bellCrankInertia(0.001, 0.001, 0.001);

const double Generic_SolidAxle::m_axleInertia = 0.4;

const double Generic_SolidAxle::m_springCoefficient = 26706.20;
const double Generic_SolidAxle::m_dampingCoefficient = 22459.0;
const double Generic_SolidAxle::m_springRestLength = 0.3948;

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Generic_SolidAxle::Generic_SolidAxle(const std::string& name) : ChSolidAxle(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient);
    m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
Generic_SolidAxle::~Generic_SolidAxle() {}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Generic_SolidAxle::getLocation(PointId which) {
    switch (which) {
        case SHOCK_A:
            return ChVector<>(-0.065, 0.575, -0.025);
        case SHOCK_C:
            return ChVector<>(-0.080, 0.56, 0.3);
        case KNUCKLE_L:
            return ChVector<>(0.005, 0.7, -0.05);
        case KNUCKLE_U:
            return ChVector<>(-0.015, 0.675, 0.075);
        case LL_A:
            return ChVector<>(0.01, 0.6, -0.075);
        case LL_C:
            return ChVector<>(0.45, 0.35, -0.045);
        case UL_A:
            return ChVector<>(-0.055, 0.475, 0.15);
        case UL_C:
            return ChVector<>(0.355, 0.5, 0.15);
        case SPRING_A:
            return ChVector<>(-0.065, 0.575, -0.025);
        case SPRING_C:
            return ChVector<>(-0.080, 0.56, 0.3);
        case TIEROD_K:
            return ChVector<>(-0.075, 0.68, -0.065);
        case SPINDLE:
            return ChVector<>(0, 0.910, 0);
        case KNUCKLE_CM:
            return ChVector<>(0, 0.7, 0);
        case LL_CM:
            return ChVector<>(0.23, 0.475, -0.06);
        case UL_CM:
            return ChVector<>(0.15, 0.4875, 0.15);
        case BELLCRANK_TIEROD:
            return ChVector<>(-0.075, 0.325, -0.065);
        case BELLCRANK_AXLE:
            return ChVector<>(0, 0.325, -0.05);
        case BELLCRANK_DRAGLINK:
            return ChVector<>(0, 0.425, -0.05);
        case DRAGLINK_C:
            return ChVector<>(0.385, 0.45, -0.02);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
