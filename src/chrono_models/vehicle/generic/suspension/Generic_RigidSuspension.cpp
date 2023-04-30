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
// Generic concrete rigid suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChRigidSuspension) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include <vector>
#include <algorithm>

#include "chrono_models/vehicle/generic/suspension/Generic_RigidSuspension.h"
#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double Generic_RigidSuspension::m_spindleMass = 1.103;

const double Generic_RigidSuspension::m_spindleRadius = 0.15;
const double Generic_RigidSuspension::m_spindleWidth = 0.06;

const ChVector<> Generic_RigidSuspension::m_spindleInertia(0.000478, 0.000496, 0.000478);

const double Generic_RigidSuspension::m_axleInertia = 0.4;

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector<> Generic_RigidSuspension::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0, 1.100, 0);  // location of spindle center of mass
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
