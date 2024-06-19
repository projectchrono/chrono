// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Class for modeling the steering of Jeep Cherokee 1997
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "Cherokee_Steering.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Cherokee_Steering::m_pitmanArmMass = 1.605;
const double Cherokee_Steering::m_pitmanArmRadius = 0.02;
const double Cherokee_Steering::m_maxAngle = 28 * CH_DEG_TO_RAD;
const ChVector3d Cherokee_Steering::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector3d Cherokee_Steering::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

Cherokee_Steering::Cherokee_Steering(const std::string& name) : ChRotaryArm(name) {}

const ChVector3d Cherokee_Steering::getLocation(PointId which) {
    switch (which) {
        case ARM_L:
            return ChVector3d(-0.20, 0.5, 0.0381);
        case ARM_C:
            return ChVector3d(-0.45, 0.5, 0.0381);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d Cherokee_Steering::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector3d(0, 0, 1);
        default:
            return ChVector3d(0, 0, 1);
    }
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
