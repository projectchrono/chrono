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
// Front and Rear generic vehicle suspension subsystems (reduced double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishboneReduced) and origins at the midpoint
// between the lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/generic/suspension/Generic_DoubleWishboneReduced.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbfpin2Npm = 175.12677;

const double Generic_DoubleWishboneReducedFront::m_uprightMass = 19.450;
const double Generic_DoubleWishboneReducedFront::m_spindleMass = 14.705;

const double Generic_DoubleWishboneReducedFront::m_spindleRadius = 0.10;
const double Generic_DoubleWishboneReducedFront::m_spindleWidth = 0.06;
const double Generic_DoubleWishboneReducedFront::m_uprightRadius = 0.04;

const ChVector<> Generic_DoubleWishboneReducedFront::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> Generic_DoubleWishboneReducedFront::m_uprightInertia(0.1656, 0.1934, 0.04367);

const double Generic_DoubleWishboneReducedFront::m_axleInertia = 0.4;

const double Generic_DoubleWishboneReducedFront::m_springCoefficient = 369149.000;
const double Generic_DoubleWishboneReducedFront::m_dampingCoefficient = 22459.000;
const double Generic_DoubleWishboneReducedFront::m_springRestLength = 0.339;

// -----------------------------------------------------------------------------

const double Generic_DoubleWishboneReducedRear::m_uprightMass = 19.450;
const double Generic_DoubleWishboneReducedRear::m_spindleMass = 14.705;

const double Generic_DoubleWishboneReducedRear::m_spindleRadius = 0.10;
const double Generic_DoubleWishboneReducedRear::m_spindleWidth = 0.06;
const double Generic_DoubleWishboneReducedRear::m_uprightRadius = 0.04;

const ChVector<> Generic_DoubleWishboneReducedRear::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector<> Generic_DoubleWishboneReducedRear::m_uprightInertia(0.1656, 0.1934, 0.04367);

const double Generic_DoubleWishboneReducedRear::m_axleInertia = 0.4;

const double Generic_DoubleWishboneReducedRear::m_springCoefficient = 369149.000;
const double Generic_DoubleWishboneReducedRear::m_dampingCoefficient = 22459.000;
const double Generic_DoubleWishboneReducedRear::m_springRestLength = 0.382;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Generic_DoubleWishboneReducedFront::Generic_DoubleWishboneReducedFront(const std::string& name)
    : ChDoubleWishboneReduced(name) {
    m_shockForceCB = chrono_types::make_shared<LinearSpringDamperForce>(m_springCoefficient, m_dampingCoefficient);
}

Generic_DoubleWishboneReducedRear::Generic_DoubleWishboneReducedRear(const std::string& name)
    : ChDoubleWishboneReduced(name) {
    m_shockForceCB = chrono_types::make_shared<LinearSpringDamperForce>(m_springCoefficient, m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Generic_DoubleWishboneReducedFront::~Generic_DoubleWishboneReducedFront() {}

Generic_DoubleWishboneReducedRear::~Generic_DoubleWishboneReducedRear() {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Generic_DoubleWishboneReducedFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(-1.59, 35.815, -1.0350);
        case UPRIGHT:
            return in2m * ChVector<>(-1.59, 31.81, -1.0350);
        case UCA_F:
            return in2m * ChVector<>(-1.89, 17.55, 9.63);
        case UCA_B:
            return in2m * ChVector<>(-10.56, 18.81, 7.69);
        case UCA_U:
            return in2m * ChVector<>(-2.09, 28.16, 8.48);
        case LCA_F:
            return in2m * ChVector<>(8.79, 12.09, 0);
        case LCA_B:
            return in2m * ChVector<>(-8.79, 12.09, 0);
        case LCA_U:
            return in2m * ChVector<>(-1.40, 30.96, -4.65);
        case SHOCK_C:
            return in2m * ChVector<>(4.10, 27.86, 12.72);
        case SHOCK_U:
            return in2m * ChVector<>(3.83, 30.96, -1.52);
        case TIEROD_C:
            return in2m * ChVector<>(-9.855, 17.655, 2.135);
        case TIEROD_U:
            return in2m * ChVector<>(-6.922, 32.327, -0.643);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> Generic_DoubleWishboneReducedRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(1.40, 35.815, -1.035);
        case UPRIGHT:
            return in2m * ChVector<>(1.40, 31.81, -1.035);
        case UCA_F:
            return in2m * ChVector<>(13.78, 18.19, 8.88);
        case UCA_B:
            return in2m * ChVector<>(3.07, 18.19, 8.88);
        case UCA_U:
            return in2m * ChVector<>(1.40, 28.16, 8.50);
        case LCA_F:
            return in2m * ChVector<>(8.79, 12.09, 0);
        case LCA_B:
            return in2m * ChVector<>(-8.79, 12.09, 0);
        case LCA_U:
            return in2m * ChVector<>(1.40, 30.96, -4.65);
        case SHOCK_C:
            return in2m * ChVector<>(-4.09, 28.19, 12.72);
        case SHOCK_U:
            return in2m * ChVector<>(-4.09, 30.96, -1.51);
        case TIEROD_C:
            return in2m * ChVector<>(8.790, 16.38, 2.310);
        case TIEROD_U:
            return in2m * ChVector<>(6.704, 32.327, -0.365);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
