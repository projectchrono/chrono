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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear mrole suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/mrole_DoubleWishbone.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double mrole_DoubleWishboneFront::m_UCAMass = 150.0;
const double mrole_DoubleWishboneFront::m_LCAMass = 150.0;
const double mrole_DoubleWishboneFront::m_uprightMass = 145.0;
const double mrole_DoubleWishboneFront::m_spindleMass = 40.0;

const double mrole_DoubleWishboneFront::m_spindleRadius = 0.10;
const double mrole_DoubleWishboneFront::m_spindleWidth = 0.06;
const double mrole_DoubleWishboneFront::m_LCARadius = 0.03;
const double mrole_DoubleWishboneFront::m_UCARadius = 0.02;
const double mrole_DoubleWishboneFront::m_uprightRadius = 0.04;

// TODO: Fix these values
const ChVector<> mrole_DoubleWishboneFront::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> mrole_DoubleWishboneFront::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> mrole_DoubleWishboneFront::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> mrole_DoubleWishboneFront::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> mrole_DoubleWishboneFront::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> mrole_DoubleWishboneFront::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> mrole_DoubleWishboneFront::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double mrole_DoubleWishboneFront::m_axleInertia = 0.4;

const double mrole_DoubleWishboneFront::m_springCoefficient = 455359.86;
const double mrole_DoubleWishboneFront::m_springRestLength = 0.62;
const double mrole_DoubleWishboneFront::m_springMaxLength = 0.640449438;
const double mrole_DoubleWishboneFront::m_springMinLength = 0.359550562;

const double mrole_DoubleWishboneFront::m_damperCoefExpansion = 152029.7673;
const double mrole_DoubleWishboneFront::m_damperDegresExpansion = 4.0;
const double mrole_DoubleWishboneFront::m_damperCoefCompression =
    0.6 * mrole_DoubleWishboneFront::m_damperCoefExpansion;
const double mrole_DoubleWishboneFront::m_damperDegresCompression = 4.0;

// -----------------------------------------------------------------------------

const double mrole_DoubleWishboneRear::m_UCAMass = 150.0;
const double mrole_DoubleWishboneRear::m_LCAMass = 150.0;
const double mrole_DoubleWishboneRear::m_uprightMass = 145.0;
const double mrole_DoubleWishboneRear::m_spindleMass = 40.0;

const double mrole_DoubleWishboneRear::m_spindleRadius = 0.10;
const double mrole_DoubleWishboneRear::m_spindleWidth = 0.06;
const double mrole_DoubleWishboneRear::m_LCARadius = 0.03;
const double mrole_DoubleWishboneRear::m_UCARadius = 0.02;
const double mrole_DoubleWishboneRear::m_uprightRadius = 0.04;

// TODO: Fix these values
const ChVector<> mrole_DoubleWishboneRear::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> mrole_DoubleWishboneRear::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector<> mrole_DoubleWishboneRear::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> mrole_DoubleWishboneRear::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector<> mrole_DoubleWishboneRear::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> mrole_DoubleWishboneRear::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector<> mrole_DoubleWishboneRear::m_uprightInertiaProducts(0.0, 0.0, 0.0);

const double mrole_DoubleWishboneRear::m_axleInertia = 0.4;

const double mrole_DoubleWishboneRear::m_springCoefficient = 455359.86;
const double mrole_DoubleWishboneRear::m_springRestLength = 0.62;
const double mrole_DoubleWishboneRear::m_springMaxLength = 0.640449438;
const double mrole_DoubleWishboneRear::m_springMinLength = 0.359550562;

const double mrole_DoubleWishboneRear::m_damperCoefExpansion = 152029.7673;
const double mrole_DoubleWishboneRear::m_damperDegresExpansion = 4.0;
const double mrole_DoubleWishboneRear::m_damperCoefCompression = 0.6 * mrole_DoubleWishboneRear::m_damperCoefExpansion;
const double mrole_DoubleWishboneRear::m_damperDegresCompression = 4.0;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------

mrole_DoubleWishboneFront::mrole_DoubleWishboneFront(const std::string& name) : ChDoubleWishbone(name) {
    auto springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient);
    springForceCB->enable_stops(m_springMinLength, m_springMaxLength);
    m_springForceCB = springForceCB;
    m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(
        m_damperCoefCompression, m_damperDegresCompression, m_damperCoefExpansion, m_damperDegresExpansion);
}

mrole_DoubleWishboneRear::mrole_DoubleWishboneRear(const std::string& name) : ChDoubleWishbone(name) {
    auto springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient);
    springForceCB->enable_stops(m_springMinLength, m_springMaxLength);
    m_springForceCB = springForceCB;
    m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(
        m_damperCoefCompression, m_damperDegresCompression, m_damperCoefExpansion, m_damperDegresExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------

mrole_DoubleWishboneFront::~mrole_DoubleWishboneFront() {}

mrole_DoubleWishboneRear::~mrole_DoubleWishboneRear() {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> mrole_DoubleWishboneFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            // return in2m * ChVector<>(-1.59, 35.815, -1.035);
            return ChVector<>(0.0, 1.29, 0.0);
        case UPRIGHT:
            // return in2m * ChVector<>(-1.59, 29.5675, -1.0350);
            return ChVector<>(0.0, 1.09, 0.0);
        case UCA_F:
            // return in2m * ChVector<>(-1.8864, 17.5575, 9.6308);
            return ChVector<>(0.1, 0.4, 0.245);
        case UCA_B:
            // return in2m * ChVector<>(-10.5596, 18.8085, 7.6992);
            return ChVector<>(-0.1, 0.4, 0.245);
        case UCA_U:
            // return in2m * ChVector<>(-2.088, 28.17, 8.484);
            return ChVector<>(0.0, 1.0, 0.245);
        case UCA_CM:
            // return in2m * ChVector<>(-4.155, 23.176, 8.575);
            return ChVector<>(0.0, 0.7, 0.245);
        case LCA_F:
            // return in2m * ChVector<>(8.7900, 12.09, 0);
            return ChVector<>(0.1, 0.4, -0.1);
        case LCA_B:
            // return in2m * ChVector<>(-8.7900, 12.09, 0);
            return ChVector<>(-0.1, 0.4, -0.1);
        case LCA_U:
            // return in2m * ChVector<>(-1.40, 30.965, -4.65);
            return ChVector<>(0.0, 1.1, -0.1);
        case LCA_CM:
            // return in2m * ChVector<>(0, 21.528, -2.325);
            return ChVector<>(0, 0.8, -0.1);
        case SHOCK_C:
            // return in2m * ChVector<>(4.095, 19.598, 12.722);
            return ChVector<>(0.0, 0.8, 0.4);
        case SHOCK_A:
            // return in2m * ChVector<>(3.827, 21.385, -1.835);
            return ChVector<>(0.0, 0.9, -0.1);
        case SPRING_C:
            return ChVector<>(0.0, 0.8, 0.4);
        case SPRING_A:
            return ChVector<>(0.0, 0.9, -0.1);
        case TIEROD_C:
            // return in2m * ChVector<>(-9.855, 17.655, 2.135);
            return ChVector<>(-0.45, 0.5, 0.0);
        case TIEROD_U:
            // return in2m * ChVector<>(-6.922, 32.327, -0.643);
            return ChVector<>(-0.45, 0.93852071, 0.0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> mrole_DoubleWishboneRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            // return in2m * ChVector<>(-1.59, 35.815, -1.035);
            return ChVector<>(0.0, 1.29, 0.0);
        case UPRIGHT:
            // return in2m * ChVector<>(-1.59, 29.5675, -1.0350);
            return ChVector<>(0.0, 1.09, 0.0);
        case UCA_F:
            // return in2m * ChVector<>(-1.8864, 17.5575, 9.6308);
            return ChVector<>(0.1, 0.4, 0.245);
        case UCA_B:
            // return in2m * ChVector<>(-10.5596, 18.8085, 7.6992);
            return ChVector<>(-0.1, 0.4, 0.245);
        case UCA_U:
            // return in2m * ChVector<>(-2.088, 28.17, 8.484);
            return ChVector<>(0.0, 1.0, 0.245);
        case UCA_CM:
            // return in2m * ChVector<>(-4.155, 23.176, 8.575);
            return ChVector<>(0.0, 0.7, 0.245);
        case LCA_F:
            // return in2m * ChVector<>(8.7900, 12.09, 0);
            return ChVector<>(0.1, 0.4, -0.1);
        case LCA_B:
            // return in2m * ChVector<>(-8.7900, 12.09, 0);
            return ChVector<>(-0.1, 0.4, -0.1);
        case LCA_U:
            // return in2m * ChVector<>(-1.40, 30.965, -4.65);
            return ChVector<>(0.0, 1.1, -0.1);
        case LCA_CM:
            // return in2m * ChVector<>(0, 21.528, -2.325);
            return ChVector<>(0, 0.8, -0.1);
        case SHOCK_C:
            // return in2m * ChVector<>(4.095, 19.598, 12.722);
            return ChVector<>(0.0, 0.8, 0.4);
        case SHOCK_A:
            // return in2m * ChVector<>(3.827, 21.385, -1.835);
            return ChVector<>(0.0, 0.9, -0.1);
        case SPRING_C:
            return ChVector<>(0.0, 0.8, 0.4);
        case SPRING_A:
            return ChVector<>(0.0, 0.9, -0.1);
        case TIEROD_C:
            // return in2m * ChVector<>(-9.855, 17.655, 2.135);
            return ChVector<>(-0.45, 0.5, 0.0);
        case TIEROD_U:
            // return in2m * ChVector<>(-6.922, 32.327, -0.643);
            return ChVector<>(-0.45, 0.93852071, 0.0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
