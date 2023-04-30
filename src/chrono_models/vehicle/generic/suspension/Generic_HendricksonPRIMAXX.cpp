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
// Generic concrete Hendrickson PRIMAXX suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChHendricksonPRIMAXX) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/generic/suspension/Generic_HendricksonPRIMAXX.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double Generic_HendricksonPRIMAXX::m_axlehousingMass = 44.958;
const double Generic_HendricksonPRIMAXX::m_knuckleMass = 1.356;
const double Generic_HendricksonPRIMAXX::m_spindleMass = 1.103;
const double Generic_HendricksonPRIMAXX::m_torquerodMass = 1.446;
const double Generic_HendricksonPRIMAXX::m_lowerbeamMass = 2.892;
const double Generic_HendricksonPRIMAXX::m_transversebeamMass = 1.0;

const double Generic_HendricksonPRIMAXX::m_axlehousingRadius = 0.03;
const double Generic_HendricksonPRIMAXX::m_knuckleRadius = 0.02;
const double Generic_HendricksonPRIMAXX::m_spindleRadius = 0.08;
const double Generic_HendricksonPRIMAXX::m_spindleWidth = 0.04;
const double Generic_HendricksonPRIMAXX::m_torquerodRadius = 0.02;
const double Generic_HendricksonPRIMAXX::m_lowerbeamRadius = 0.03;
const double Generic_HendricksonPRIMAXX::m_transversebeamRadius = 0.02;

const ChVector<> Generic_HendricksonPRIMAXX::m_axlehousingCOM(0, 0, 0);
const ChVector<> Generic_HendricksonPRIMAXX::m_transversebeamCOM(-0.376, 0, -0.197);

const ChVector<> Generic_HendricksonPRIMAXX::m_axlehousingInertia(0.744, 0.045, 0.744);
const ChVector<> Generic_HendricksonPRIMAXX::m_knuckleInertia(0.00255, 0.00134, 0.00196);
const ChVector<> Generic_HendricksonPRIMAXX::m_spindleInertia(0.0000558, 0.0000279, 0.0000558);
const ChVector<> Generic_HendricksonPRIMAXX::m_torquerodInertia(0.011, 0.011, 0.000142);
const ChVector<> Generic_HendricksonPRIMAXX::m_lowerbeamInertia(0.0514, 0.0514, 0.00037);
const ChVector<> Generic_HendricksonPRIMAXX::m_transversebeamInertia(0.5, 0.2, 0.5);

const double Generic_HendricksonPRIMAXX::m_axleInertia = 0.4;

const double Generic_HendricksonPRIMAXX::m_shockAH_springCoefficient = 0;
const double Generic_HendricksonPRIMAXX::m_shockAH_dampingCoefficient = 22459.0;
const double Generic_HendricksonPRIMAXX::m_shockAH_restLength = 0.3948;

const double Generic_HendricksonPRIMAXX::m_shockLB_springCoefficient = 267062.0;
const double Generic_HendricksonPRIMAXX::m_shockLB_dampingCoefficient = 0;
const double Generic_HendricksonPRIMAXX::m_shockLB_restLength = 0.3948;

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Generic_HendricksonPRIMAXX::Generic_HendricksonPRIMAXX(const std::string& name) : ChHendricksonPRIMAXX(name) {
    m_shockAHForceCB =
        chrono_types::make_shared<LinearSpringDamperForce>(m_shockAH_springCoefficient, m_shockAH_dampingCoefficient);
    m_shockLBForceCB =
        chrono_types::make_shared<LinearSpringDamperForce>(m_shockLB_springCoefficient, m_shockLB_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
Generic_HendricksonPRIMAXX::~Generic_HendricksonPRIMAXX() {}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector<> Generic_HendricksonPRIMAXX::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector<>(0, 0.910, 0);  // spindle location
        case KNUCKLE_L:
            return ChVector<>(0.006, 0.849, -0.061);  // lower knuckle point
        case KNUCKLE_U:
            return ChVector<>(-0.018, 0.819, 0.091);  // upper knuckle point
        case TIEROD_C:
            return ChVector<>(-0.091, 0.400, -0.079);  // tierod, chassis
        case TIEROD_K:
            return ChVector<>(-0.091, 0.825, -0.079);  // tierod, knuckle
        case TORQUEROD_C:
            return ChVector<>(0.629, 0.65, 0.1);  // torquerod, chassis
        case TORQUEROD_AH:
            return ChVector<>(-0.0, 0.65, 0.1);  // torquerod, axle housing (AH)
        case LOWERBEAM_C:
            return ChVector<>(0.629, 0.65, -0.0);  // lowerbeam, chassis
        case LOWERBEAM_AH:
            return ChVector<>(-0.0, 0.65, -0.197);  // lowerbeam, axle housing (AH)
        case LOWERBEAM_TB:
            return ChVector<>(-0.376, 0.65, -0.197);  // lowerbeam, transverse beam
        case SHOCKAH_C:
            return ChVector<>(-0.1, 0.65, 0.15);  // shock at axle housing (AH), chassis
        case SHOCKAH_AH:
            return ChVector<>(-0.1, 0.65, -0.1);  // shock at axle housing (AH), axle housing
        case SHOCKLB_C:
            return ChVector<>(-0.376, 0.65, 0.15);  //  shock at lower beam (LB), chassis
        case SHOCKLB_LB:
            return ChVector<>(-0.376, 0.65, -0.197);  //  shock at lower beam (LB), lower beam
        case KNUCKLE_CM:
            return ChVector<>(-0.006, 0.834, 0.015);  //  knuckle, center of mass
        case TORQUEROD_CM:
            return ChVector<>(0.0, 0.65, 0.0);  //  torquerod, center of mass
        case LOWERBEAM_CM:
            return ChVector<>(0.0, 0.65, 0.0);  // lowerbeam, center of mass
        default:
            return ChVector<>(0, 0, 0);
    }
}

// -----------------------------------------------------------------------------
// Implementation of the getDirection() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Generic_HendricksonPRIMAXX::getDirection(DirectionId which) {
    switch (which) {
        case UNIV_AXIS_TORQUEROD_ROD:
            return ChVector<>(0, 1, 0);
        case UNIV_AXIS_TORQUEROD_CHASSIS:
            return ChVector<>(0, 0, 1);
        case UNIV_AXIS_LOWERBEAM_BEAM:
            return ChVector<>(0, -1, 0);
        case UNIV_AXIS_LOWERBEAM_CHASSIS:
            return ChVector<>(0, 0, 1);
        default:
            return ChVector<>(0, 0, 1);
    }
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
