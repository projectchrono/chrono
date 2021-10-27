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
// Kraz 64431 rotary arm steering model.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_tractor_Steering.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

using namespace chrono;
using namespace chrono::vehicle;

const double Kraz_tractor_Steering::m_pitmanArmMass = 1.605;

const double Kraz_tractor_Steering::m_pitmanArmRadius = 0.02;

const double Kraz_tractor_Steering::m_maxAngle = 22.7 * (CH_C_PI / 180);

const ChVector<> Kraz_tractor_Steering::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector<> Kraz_tractor_Steering::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Kraz_tractor_Steering::Kraz_tractor_Steering(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> Kraz_tractor_Steering::getLocation(PointId which) {
    const double ofs = 0.081;
    switch (which) {
        case ARM_L:
            return ChVector<>(1.0, 0.708341392 - ofs, 0.1);
        case ARM_C:
            return ChVector<>(1.0, 0.708341392 - ofs, 0.3);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> Kraz_tractor_Steering::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector<>(0, 1, 0);
        default:
            return ChVector<>(0, 1, 0);
    }
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
