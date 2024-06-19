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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Rainer Gericke
// =============================================================================
//
// Rear BMW E90 suspension subsystems (double A-arm), replacement for multilink
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/bmw/BMW_E90_DoubleWishbone.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

// -----------------------------------------------------------------------------

const double BMW_E90_DoubleWishbone::m_UCAMass = 5.813;
const double BMW_E90_DoubleWishbone::m_LCAMass = 23.965;
const double BMW_E90_DoubleWishbone::m_uprightMass = 19.450;
const double BMW_E90_DoubleWishbone::m_spindleMass = 14.705;
const double BMW_E90_DoubleWishbone::m_tierodMass = 6.0;

const double BMW_E90_DoubleWishbone::m_spindleRadius = 0.10;
const double BMW_E90_DoubleWishbone::m_spindleWidth = 0.06;
const double BMW_E90_DoubleWishbone::m_LCARadius = 0.015;
const double BMW_E90_DoubleWishbone::m_UCARadius = 0.015;
const double BMW_E90_DoubleWishbone::m_uprightRadius = 0.025;
const double BMW_E90_DoubleWishbone::m_tierodRadius = 0.01;

// TODO: Fix these values
const ChVector3d BMW_E90_DoubleWishbone::m_spindleInertia(0.04117, 0.07352, 0.04117);
const ChVector3d BMW_E90_DoubleWishbone::m_UCAInertiaMoments(0.03, 0.03, 0.06276);
const ChVector3d BMW_E90_DoubleWishbone::m_UCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector3d BMW_E90_DoubleWishbone::m_LCAInertiaMoments(0.4, 0.4, 0.8938);
const ChVector3d BMW_E90_DoubleWishbone::m_LCAInertiaProducts(0.0, 0.0, 0.0);
const ChVector3d BMW_E90_DoubleWishbone::m_uprightInertiaMoments(0.1656, 0.1934, 0.04367);
const ChVector3d BMW_E90_DoubleWishbone::m_uprightInertiaProducts(0.0, 0.0, 0.0);
const ChVector3d BMW_E90_DoubleWishbone::m_tierodInertia(0.05, 0.05, 0.5);

const double BMW_E90_DoubleWishbone::m_axleInertia = 0.4;

const double BMW_E90_DoubleWishbone::m_springRestLength = 0.275364585;
const double BMW_E90_DoubleWishbone::m_springConstant = 37130.0;

const double BMW_E90_DoubleWishbone::m_springPreload = 7600;
const double BMW_E90_DoubleWishbone::m_damperConstant = 5810.4;

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

BMW_E90_DoubleWishbone::BMW_E90_DoubleWishbone(const std::string& name, bool use_tierod_bodies)
    : ChDoubleWishbone(name), m_use_tierod_bodies(use_tierod_bodies) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springConstant, m_springPreload);
    auto sptr = std::static_pointer_cast<LinearSpringForce>(m_springForceCB);
    sptr->enable_stops(m_springRestLength - 0.05, m_springRestLength + 0.05);
    sptr->set_stops(2.0 * m_springConstant, 2.0 * m_springConstant);
    m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_damperConstant);
}

BMW_E90_DoubleWishbone::~BMW_E90_DoubleWishbone() {}

// -----------------------------------------------------------------------------

const ChVector3d BMW_E90_DoubleWishbone::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector3d(0, 0.7493, 0);
        case UPRIGHT:
            return ChVector3d(0, 0.6493, 0);
        case UCA_F:
            return ChVector3d(0.14986, 0.4572, 0.0635);
        case UCA_B:
            return ChVector3d(-0.0508, 0.40132, 0.11684);
        case UCA_U:
            return ChVector3d(0.01397, 0.65024, 0.08636);
        case UCA_CM:
            return ChVector3d(0.037676667, 0.50292, 0.0889);
        case LCA_F:
            return ChVector3d(0.22352, 0.41148, -0.07874);
        case LCA_B:
            return ChVector3d(-0.1778, 0.25908, -0.12446);
        case LCA_U:
            return ChVector3d(-0.01778, 0.64389, -0.127);
        case LCA_CM:
            return ChVector3d(0.009313333, 0.43815, -0.110066667);
        case SHOCK_C:
            return ChVector3d(-0.09906, 0.508, 0.36576);
        case SHOCK_A:
            return ChVector3d(-0.08382, 0.56388, -0.127);
        case SPRING_C:
            return ChVector3d(-0.12446, 0.44704, 0.09906);
        case SPRING_A:
            return ChVector3d(-0.11176, 0.46736, -0.17526);
        case TIEROD_C:
            return ChVector3d(-0.2235, 0.25781, -0.04064);
        case TIEROD_U:
            return ChVector3d(-0.1524, 0.65786, -0.04572);
        default:
            return ChVector3d(0, 0, 0);
    }
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
