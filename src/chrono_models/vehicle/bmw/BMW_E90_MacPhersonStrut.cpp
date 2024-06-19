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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// BMW E90 (330i 2006) concrete MacPherson Strut suspension subsystem.
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include <vector>
#include <algorithm>

#include "chrono_models/vehicle/bmw/BMW_E90_MacPhersonStrut.h"
#include "chrono/core/ChCubicSpline.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------
const double BMW_E90_MacPhersonStrut::m_spindleMass = 9.962477554;
const double BMW_E90_MacPhersonStrut::m_strutMass = 13.5;
const double BMW_E90_MacPhersonStrut::m_uprightMass = 13.51;
const double BMW_E90_MacPhersonStrut::m_LCAMass = 2.72;
const double BMW_E90_MacPhersonStrut::m_tierodMass = 1.13;
const double BMW_E90_MacPhersonStrut::m_axleInertia = 0.4;

const double BMW_E90_MacPhersonStrut::m_spindleRadius = 0.1;
const double BMW_E90_MacPhersonStrut::m_strutRadius = 0.028;
const double BMW_E90_MacPhersonStrut::m_spindleWidth = 0.05;
const double BMW_E90_MacPhersonStrut::m_LCARadius = 0.025;
const double BMW_E90_MacPhersonStrut::m_uprightRadius = 0.03;

const double BMW_E90_MacPhersonStrut::m_trackWidth = 2.0 * 0.750062;
const ChVector3d BMW_E90_MacPhersonStrut::m_spindleInertia(0.000478, 0.000496, 0.000478);
const ChVector3d BMW_E90_MacPhersonStrut::m_strutInertia(0.000478, 0.000496, 0.000478);
const ChVector3d BMW_E90_MacPhersonStrut::m_LCAInertiaMoments(0.0151, 0.0207, 0.0355);
const ChVector3d BMW_E90_MacPhersonStrut::m_uprightInertiaMoments(0.0138, 0.0146, 0.00283);
const ChVector3d BMW_E90_MacPhersonStrut::m_tierodInertiaMoments(0.01, 0.001, 0.01);

const double BMW_E90_MacPhersonStrut::m_springCoefficient = 29770.0;
const double BMW_E90_MacPhersonStrut::m_dampingCoefficient = 4352.486957;
// Spring design length = 0.482914056 m
const double BMW_E90_MacPhersonStrut::m_springRestLength = 0.482914056;
// static wheel load = 560 kg
const double BMW_E90_MacPhersonStrut::m_springPreload = 5600;

BMW_E90_MacPhersonStrut::BMW_E90_MacPhersonStrut(const std::string& name) : ChMacPhersonStrut(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient, m_springPreload);
    auto ptr = std::static_pointer_cast<LinearSpringForce>(m_springForceCB);
    ptr->enable_stops(m_springRestLength - 0.05, m_springRestLength + 0.05);
    ptr->set_stops(2.0 * m_springCoefficient, 2.0 * m_springCoefficient);
    // m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient);
    m_shockForceCB =
        chrono_types::make_shared<DegressiveDamperForce>(m_dampingCoefficient, 1.3, m_dampingCoefficient, 1.3);
}

BMW_E90_MacPhersonStrut::~BMW_E90_MacPhersonStrut() {}

const ChVector3d BMW_E90_MacPhersonStrut::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector3d(0, 0.750062, 0);  // location of spindle center of mass
        case UPRIGHT:
            return ChVector3d(0, 0.650062, 0);  // location of upright center of mass
        case LCA_F:
            return ChVector3d(0.27051, 0.34544, -0.05969);  // LCA front connection point to chassis
        case LCA_B:
            return ChVector3d(-0.04318, 0.37338, -0.12573);  // LCA rear (back) connection point to chassis
        case LCA_U:
            return ChVector3d(0.02794, 0.66294, -0.10414);  // LCA connection point to upright
        case LCA_CM:
            return ChVector3d(0.0708025, 0.511175, -0.098425);  // location of LCA center of mass
        case SHOCK_C:
            return ChVector3d(-0.08382, 0.54102, 0.46863);  // shock connection to chassis
        case SHOCK_U:
            return ChVector3d(-0.00508, 0.61976, -0.00127);  // shock connection to upright
        case SPRING_C:
            return ChVector3d(-0.08382, 0.54102, 0.46863);  // spring connection to chassis
        case SPRING_U:
            return ChVector3d(-0.00508, 0.61976, -0.00127);  // spring connection to upright
        case TIEROD_C:
            return ChVector3d(-0.2, 0.2, -0.05);  // tierod connection point to chassis/steering
        case TIEROD_U:
            return ChVector3d(-0.2,	0.574803759,	-0.00127);  // tierod connection point to upright
        default:
            return ChVector3d(0, 0, 0);
    }
}
}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
