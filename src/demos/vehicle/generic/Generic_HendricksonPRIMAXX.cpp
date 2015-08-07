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
// Authors: Radu Serban
// =============================================================================
//
// Generic concrete Hendrickson PRIMAXX suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChHendricksonPRIMAXX) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include "generic/Generic_HendricksonPRIMAXX.h"

using namespace chrono;


// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double     Generic_HendricksonPRIMAXX::m_axlehousingMass = 44.958;
const double     Generic_HendricksonPRIMAXX::m_knuckleMass = 1.356;
const double     Generic_HendricksonPRIMAXX::m_spindleMass = 1.103;
const double     Generic_HendricksonPRIMAXX::m_torquerodMass = 1.446;
const double     Generic_HendricksonPRIMAXX::m_lowerbeamMass = 2.892;
const double     Generic_HendricksonPRIMAXX::m_transversebeamMass = 1.0;

const double     Generic_HendricksonPRIMAXX::m_axlehousingRadius = 0.03;
const double     Generic_HendricksonPRIMAXX::m_knuckleRadius = 0.01;
const double     Generic_HendricksonPRIMAXX::m_spindleRadius = 0.15;
const double     Generic_HendricksonPRIMAXX::m_spindleWidth = 0.06;
const double     Generic_HendricksonPRIMAXX::m_torquerodRadius = 0.02;
const double     Generic_HendricksonPRIMAXX::m_lowerbeamRadius = 0.02;
const double     Generic_HendricksonPRIMAXX::m_transversebeamRadius = 0.02;

const ChVector<> Generic_HendricksonPRIMAXX::m_axlehousingCOM(0, 0, 0);

const ChVector<> Generic_HendricksonPRIMAXX::m_axlehousingInertia(7.744, 0.045, 7.744);
const ChVector<> Generic_HendricksonPRIMAXX::m_knuckleInertia(0.00255, 0.00134, 0.00196);
const ChVector<> Generic_HendricksonPRIMAXX::m_spindleInertia(0.0000558, 0.0000279, 0.0000558);
const ChVector<> Generic_HendricksonPRIMAXX::m_torquerodInertia(0.011, 0.011, 0.000142);
const ChVector<> Generic_HendricksonPRIMAXX::m_lowerbeamInertia(0.0514, 0.0514, 0.00037);
const ChVector<> Generic_HendricksonPRIMAXX::m_transversebeamInertia(1, 1, 1);

const double     Generic_HendricksonPRIMAXX::m_axleInertia = 0.4;

//
// HH Values from the springs are copied from SolidAxle
// Have to be modified
//
const double     Generic_HendricksonPRIMAXX::m_springAHCoefficient = 267062.0;
const double     Generic_HendricksonPRIMAXX::m_dampingAHCoefficient = 22459.0;
const double     Generic_HendricksonPRIMAXX::m_springAHRestLength = 0.3948;

const double     Generic_HendricksonPRIMAXX::m_springLBCoefficient = 267062.0;
const double     Generic_HendricksonPRIMAXX::m_dampingLBCoefficient = 22459.0;
const double     Generic_HendricksonPRIMAXX::m_springLBRestLength = 0.3948;


// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Generic_HendricksonPRIMAXX::Generic_HendricksonPRIMAXX(const std::string& name)
: ChHendricksonPRIMAXX(name)
{
  m_springAHForceCB = new LinearSpringForce(m_springAHCoefficient);
  m_shockAHForceCB = new LinearDamperForce(m_dampingAHCoefficient);

  m_springLBForceCB = new LinearSpringForce(m_springLBCoefficient);
  m_shockLBForceCB = new LinearDamperForce(m_dampingLBCoefficient);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
Generic_HendricksonPRIMAXX::~Generic_HendricksonPRIMAXX()
{
  delete m_springAHForceCB;
  delete m_shockAHForceCB;

  delete m_springLBForceCB;
  delete m_shockLBForceCB;

}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector<> Generic_HendricksonPRIMAXX::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:           return ChVector<>(0, 0.910, 0);            ///< spindle location
  case KNUCKLE_L:         return ChVector<>(0.006, 0.849, -0.061);   ///< lower knuckle point
  case KNUCKLE_U:         return ChVector<>(-0.018, 0.819, 0.091);   ///< upper knuckle point
  case TIEROD_C:          return ChVector<>(-0.091, 0.400, -0.079);  ///< tierod, chassis
  case TIEROD_K:          return ChVector<>(-0.091, 0.825, -0.079);  ///< tierod, knuckle
  case TORQUEROD_C:       return ChVector<>(0.629, 0.65, 0.1);  ///< torquerod, chassis
  case TORQUEROD_AH:      return ChVector<>(-0.0, 0.65, 0.1);  ///< torquerod, axle housing (AH)
  case LOWERBEAM_C:       return ChVector<>(0.629, 0.65, -0.0);  ///< lowerbeam, chassis
  case LOWERBEAM_AH:      return ChVector<>(-0.0, 0.65, -0.197);  ///< lowerbeam, axle housing (AH)
  case LOWERBEAM_TB:      return ChVector<>(-0.376, 0.65, -0.197);  ///< lowerbeam, transverse beam 
  case SHOCKAH_C:         return ChVector<>(-0.1, 0.65, 0.15);  ///< shock at axle housing (AH), chasis
  case SHOCKAH_AH:        return ChVector<>(-0.1, 0.65, -0.1);  ///< shock at axle housing (AH), axle housing
  case SPRINGAH_C:        return ChVector<>(-0.1, 0.65, 0.15);  ///< spring at axle housing (AH), chasis
  case SPRINGAH_AH:       return ChVector<>(-0.1, 0.65, -0.1);  ///< spring at axle housing (AH), axle housing
  case SHOCKLB_C:         return ChVector<>(-0.376, 0.65, 0.15);  ///< shock at lower beam (LB), chasis
  case SHOCKLB_LB:        return ChVector<>(-0.376, 0.65, -0.197);  ///< shock at lower beam (LB), lower beam
  case SPRINGLB_C:        return ChVector<>(-0.376, 0.65, 0.15);  ///< spring at lower beam (LB), chasis
  case SPRINGLB_LB:       return ChVector<>(-0.376, 0.65, -0.197);  ///< spring at lower beam (LB), lower beam
  case KNUCKLE_CM:        return ChVector<>(-0.006, 0.834, 0.015);  ///< knuckle, center of mass
  case TORQUEROD_CM:      return ChVector<>(-0.0, 0.65, -0.0);  ///< torquerod, center of mass
  case LOWERBEAM_CM:      return ChVector<>(-0.0, 0.65, -0.0);  ///< lowerbeam, center of mass
  case SPRINGAH_CM:       return ChVector<>(-0.0, 0.65, -0.0);  ///< spring at axle housing (AH), center of mass
  case SPRINGLB_CM:       return ChVector<>(-0.0, 0.65, -0.0);  ///< spring at lower beam (LB), center of mass
  case TRANSVERSEBEAM_CM: return ChVector<>(-0.0, 0.0, -0.0);  ///< transverse beam, center of mass
  case AXLEHOUSING_CM:    return ChVector<>(-0.0, 0.0, -0.0);     ///< axle housing (AH), center of mass
  default:                return ChVector<>(0, 0, 0);
  }
}

// -----------------------------------------------------------------------------
// Implementation of the getDirection() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Generic_HendricksonPRIMAXX::getDirection(DirectionId which)
{
  switch (which) {
  case UNIV_TORQUEROD_AXIS_ROD:      return ChVector<>(0, 1, 0);
  case UNIV_TORQUEROD_AXIS_CHASSIS:  return ChVector<>(0, 0, 1);
  case UNIV_LOWERBEAM_AXIS_BEAM:     return ChVector<>(0, -1, 0);
  case UNIV_LOWERBEAM_AXIS_CHASSIS:  return ChVector<>(0, 0, 1);
  default:                           return ChVector<>(0, 0, 1);
  }
}
