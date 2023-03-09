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
// Authors: Daniel Melanz
// =============================================================================
//
// Generic concrete MacPherson strut subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChMacPhersonStrut) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_MacPhersonStrut.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double Generic_MacPhersonStrut::m_spindleMass = 1.103;
const double Generic_MacPhersonStrut::m_uprightMass = 20.000;
const double Generic_MacPhersonStrut::m_strutMass = 5.000;
const double Generic_MacPhersonStrut::m_LCAMass = 5.091;

const double Generic_MacPhersonStrut::m_spindleRadius = 0.15;
const double Generic_MacPhersonStrut::m_spindleWidth = 0.06;
const double Generic_MacPhersonStrut::m_uprightRadius = 0.025;
const double Generic_MacPhersonStrut::m_strutRadius = 0.02;
const double Generic_MacPhersonStrut::m_LCARadius = 0.02;

const ChVector<> Generic_MacPhersonStrut::m_spindleInertia(0.000478, 0.000479, 0.000496);
const ChVector<> Generic_MacPhersonStrut::m_uprightInertia(0.0138, 0.0146, 0.00283);
const ChVector<> Generic_MacPhersonStrut::m_strutInertia(0.01, 0.01, 0.005);
const ChVector<> Generic_MacPhersonStrut::m_LCAInertia(0.0269, 0.06058, 0.03377);

const double Generic_MacPhersonStrut::m_axleInertia = 0.4;

const double Generic_MacPhersonStrut::m_springCoefficient = 369149.000;
const double Generic_MacPhersonStrut::m_dampingCoefficient = 22459.000;
const double Generic_MacPhersonStrut::m_springRestLength = 0.306;

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Generic_MacPhersonStrut::Generic_MacPhersonStrut(const std::string& name) : ChMacPhersonStrut(name) {
    m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient);
    m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
Generic_MacPhersonStrut::~Generic_MacPhersonStrut() {}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector<> Generic_MacPhersonStrut::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(-0.04000403, 1.09999965, -0.02602507);  // location of spindle center of mass
        case UPRIGHT:
            return ChVector<>(-0.04000015, 0.90999965, -0.0260268);  // location of upright center of mass
        case LCA_F:
            return ChVector<>(0.220, 0.520, -0.156);  // LCA front connection point to chassis
        case LCA_B:
            return ChVector<>(-0.300, 0.507, -0.137);  // LCA rear (back) connection point to chassis
        case LCA_U:
            return ChVector<>(-0.02165371, 0.94057703, -0.17402826);  // LCA connection point to upright
        case LCA_CM:
            return ChVector<>(-0.031, 0.646, -0.156);  // location of LCA center of mass
        case SHOCK_C:
            return ChVector<>(-0.115, 0.785, 0.579);  // shock connection to chassis
        case SHOCK_U:
            return ChVector<>(-0.07402507, 0.8532915, 0.2484536);  // shock connection point to LCA
        case SPRING_C:
            return ChVector<>(-0.115, 0.785, 0.579);  // spring connection point to chassis
        case SPRING_U:
            return ChVector<>(-0.07402507, 0.8532915, 0.2484536);  // spring connection point to LCA
        case TIEROD_C:
            return ChVector<>(-0.300, 0.520, -0.059);  // tierod connection point to chassis
        case TIEROD_U:
            return ChVector<>(-0.2373756, 0.89495045, -0.01605418);  // tierod connection point to upright
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
