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
// Gator concrete single wishbone suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include <vector>
#include <algorithm>

#include "chrono_models/vehicle/gator/Gator_SingleWishbone.h"
#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double Gator_SingleWishbone::m_spindleMass = 3.0;
const double Gator_SingleWishbone::m_uprightMass = 1.5;
const double Gator_SingleWishbone::m_CAMass = 4.0;

const double Gator_SingleWishbone::m_spindleRadius = 0.06;
const double Gator_SingleWishbone::m_spindleWidth = 0.025;
const double Gator_SingleWishbone::m_uprightRadius = 0.01;
const double Gator_SingleWishbone::m_CARadius = 0.03;

const ChVector<> Gator_SingleWishbone::m_spindleInertia(0.001, 0.0018, 0.001);
const ChVector<> Gator_SingleWishbone::m_uprightInertiaMoments(0.0138, 0.0146, 0.00283);
const ChVector<> Gator_SingleWishbone::m_uprightInertiaProducts(0.0, 0.0, 0.0);
const ChVector<> Gator_SingleWishbone::m_CAInertiaMoments(0.0151, 0.0207, 0.0355);
const ChVector<> Gator_SingleWishbone::m_CAInertiaProducts(0.0, 0.0, 0.0);

const double Gator_SingleWishbone::m_axleInertia = 0.4;

const double Gator_SingleWishbone::m_springCoefficient = 300000.0;
const double Gator_SingleWishbone::m_dampingCoefficient = 20000.0;
const double Gator_SingleWishbone::m_shockRestLength = 0.3717;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Gator_SingleWishbone::Gator_SingleWishbone(const std::string& name) : ChSingleWishbone(name) {
    m_shockForceCB = chrono_types::make_shared<LinearSpringDamperForce>(m_springCoefficient, m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Gator_SingleWishbone::~Gator_SingleWishbone() {}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector<> Gator_SingleWishbone::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0, 0.56, -0.02);  // spindle location
        case UPRIGHT:
            return ChVector<>(0, 0.5, -0.02);  // upright location
        case CA_C:
            return ChVector<>(0, 0.06, 0);  // control arm, chassis
        case CA_U:
            return ChVector<>(0, 0.48, -0.02);  // control arm, upright
        case CA_CM:
            return ChVector<>(0, 0.21, -0.01);  // control arm, center of mass
        case STRUT_C:
            return ChVector<>(0, 0.28, 0.34);  // strut, chassis
        case STRUT_A:
            return ChVector<>(0, 0.39, -0.015);  // strut, control arm
        case TIEROD_C:
            return ChVector<>(-0.05, 0.21, 0.08);  // tierod connection point to chassis
        case TIEROD_U:
            return ChVector<>(-0.05, 0.48, 0.08);  // tierod connection point to upright
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
