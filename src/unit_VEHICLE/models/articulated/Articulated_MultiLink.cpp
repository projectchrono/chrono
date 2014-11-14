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
// Authors: Daniel Melanz, Radu Serban, Alessandro Tasora
// =============================================================================
//
// Front and Rear multi-link suspension subsystems.
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origins at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include "models/articulated/Articulated_MultiLink.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double      Articulated_MultiLinkFront::m_spindleMass = 1.103;
const double      Articulated_MultiLinkFront::m_upperArmMass = 4.744;
const double      Articulated_MultiLinkFront::m_lateralMass = 1.910;
const double      Articulated_MultiLinkFront::m_trailingLinkMass = 15.204;
const double      Articulated_MultiLinkFront::m_uprightMass = 3.201;

const double      Articulated_MultiLinkFront::m_spindleRadius = 0.15;
const double      Articulated_MultiLinkFront::m_spindleWidth = 0.03;
const double      Articulated_MultiLinkFront::m_upperArmRadius = 0.02;
const double      Articulated_MultiLinkFront::m_lateralRadius = 0.02;
const double      Articulated_MultiLinkFront::m_trailingLinkRadius = 0.03;
const double      Articulated_MultiLinkFront::m_uprightRadius = 0.02;

const ChVector<>  Articulated_MultiLinkFront::m_spindleInertia(0.000478, 0.000478, 0.000496);
const ChVector<>  Articulated_MultiLinkFront::m_upperArmInertia(0.0237, 0.0294, 0.00612);
const ChVector<>  Articulated_MultiLinkFront::m_lateralInertia(0.0543, 0.0541, 0.000279);
const ChVector<>  Articulated_MultiLinkFront::m_trailingLinkInertia(0.0762, 0.527, 0.567);
const ChVector<>  Articulated_MultiLinkFront::m_uprightInertia(0.0250, 0.00653, 0.0284);

const double      Articulated_MultiLinkFront::m_axleInertia = 0.166;

const double      Articulated_MultiLinkFront::m_springCoefficient = 167062.000;
const double      Articulated_MultiLinkFront::m_dampingCoefficient = 60068.000;
const double      Articulated_MultiLinkFront::m_springRestLength = 0.339;

// -----------------------------------------------------------------------------

const double      Articulated_MultiLinkRear::m_spindleMass = 1.103;
const double      Articulated_MultiLinkRear::m_upperArmMass = 4.744;
const double      Articulated_MultiLinkRear::m_lateralMass = 1.910;
const double      Articulated_MultiLinkRear::m_trailingLinkMass = 15.204;
const double      Articulated_MultiLinkRear::m_uprightMass = 3.201;

const double      Articulated_MultiLinkRear::m_spindleRadius = 0.15;
const double      Articulated_MultiLinkRear::m_spindleWidth = 0.03;
const double      Articulated_MultiLinkRear::m_upperArmRadius = 0.02;
const double      Articulated_MultiLinkRear::m_lateralRadius = 0.02;
const double      Articulated_MultiLinkRear::m_trailingLinkRadius = 0.03;
const double      Articulated_MultiLinkRear::m_uprightRadius = 0.02;

const ChVector<>  Articulated_MultiLinkRear::m_spindleInertia(0.000478, 0.000478, 0.000496);
const ChVector<>  Articulated_MultiLinkRear::m_upperArmInertia(0.0237, 0.0294, 0.00612);
const ChVector<>  Articulated_MultiLinkRear::m_lateralInertia(0.0543, 0.0541, 0.000279);
const ChVector<>  Articulated_MultiLinkRear::m_trailingLinkInertia(0.0762, 0.527, 0.567);
const ChVector<>  Articulated_MultiLinkRear::m_uprightInertia(0.0250, 0.00653, 0.0284);

const double      Articulated_MultiLinkRear::m_axleInertia = 0.166;

const double      Articulated_MultiLinkRear::m_springCoefficient = 167062.000;
const double      Articulated_MultiLinkRear::m_dampingCoefficient = 60068.000;
const double      Articulated_MultiLinkRear::m_springRestLength = 0.339;


// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Articulated_MultiLinkFront::Articulated_MultiLinkFront(const std::string& name)
: ChMultiLink(name)
{
}

Articulated_MultiLinkRear::Articulated_MultiLinkRear(const std::string& name)
: ChMultiLink(name)
{
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Articulated_MultiLinkFront::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:   return ChVector<>(0.000, 0.950, 0.000);
  case UPRIGHT:   return ChVector<>(0.000, 0.910, 0.000);
  case UA_F:      return ChVector<>(0.060, 0.547, 0.082);
  case UA_B:      return ChVector<>(-0.157, 0.508, 0.062);
  case UA_U:      return ChVector<>(0.056, 0.864, 0.151);
  case UA_CM:     return ChVector<>(-0.014, 0.640, 0.098);
  case LAT_C:     return ChVector<>(0.036, 0.338, -0.133);
  case LAT_U:     return ChVector<>(0.029, 0.842, -0.093);
  case LAT_CM:    return ChVector<>(0.033, 0.590, -0.113);
  case TL_C:      return ChVector<>(0.723, 0.599, -0.072);
  case TL_U:      return ChVector<>(-0.000, 0.864, -0.156);
  case TL_CM:     return ChVector<>(0.279, 0.693, -0.132);
  case SHOCK_C:   return ChVector<>(0.171, 0.628, 0.315);
  case SHOCK_L:   return ChVector<>(0.181, 0.669, -0.162);
  case SPRING_C:  return ChVector<>(0.181, 0.641, 0.110);
  case SPRING_L:  return ChVector<>(0.181, 0.669, -0.164);
  case TIEROD_C:  return ChVector<>(-0.257, 0.320, -0.116);
  case TIEROD_U:  return ChVector<>(-0.144, 0.862, -0.056);
  default:        return ChVector<>(0, 0, 0);
  }
}

const ChVector<> Articulated_MultiLinkRear::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:   return ChVector<>(0.000, 0.950, 0.000);
  case UPRIGHT:   return ChVector<>(0.000, 0.910, 0.000);
  case UA_F:      return ChVector<>(0.060, 0.547, 0.082);
  case UA_B:      return ChVector<>(-0.157, 0.508, 0.062);
  case UA_U:      return ChVector<>(0.056, 0.864, 0.151);
  case UA_CM:     return ChVector<>(-0.014, 0.640, 0.098);
  case LAT_C:     return ChVector<>(0.036, 0.338, -0.133);
  case LAT_U:     return ChVector<>(0.029, 0.842, -0.093);
  case LAT_CM:    return ChVector<>(0.033, 0.590, -0.113);
  case TL_C:      return ChVector<>(0.723, 0.599, -0.072);
  case TL_U:      return ChVector<>(-0.000, 0.864, -0.156);
  case TL_CM:     return ChVector<>(0.279, 0.693, -0.132);
  case SHOCK_C:   return ChVector<>(0.171, 0.628, 0.315);
  case SHOCK_L:   return ChVector<>(0.181, 0.669, -0.162);
  case SPRING_C:  return ChVector<>(0.181, 0.641, 0.110);
  case SPRING_L:  return ChVector<>(0.181, 0.669, -0.164);
  case TIEROD_C:  return ChVector<>(-0.257, 0.320, -0.116);
  case TIEROD_U:  return ChVector<>(-0.144, 0.862, -0.056);
  default:        return ChVector<>(0, 0, 0);
  }
}

// -----------------------------------------------------------------------------
// Implementations of the getDirection() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Articulated_MultiLinkFront::getDirection(DirectionId which)
{
  switch (which) {
  case UNIV_AXIS_LINK_TL:     return ChVector<>(0, 0, 1);
  case UNIV_AXIS_CHASSIS_TL:  return ChVector<>(0.272, 0.962, 0);
  case UNIV_AXIS_LINK_LAT:    return ChVector<>(-0.978950, 0.204099, 0);
  case UNIV_AXIS_CHASSIS_LAT: return ChVector<>(-0.021990, -0.105472, 0.994179);
  default:                    return ChVector<>(0, 0, 1);
  }
}

const ChVector<> Articulated_MultiLinkRear::getDirection(DirectionId which)
{
  switch (which) {
  case UNIV_AXIS_LINK_TL:     return ChVector<>(0, 0, 1);
  case UNIV_AXIS_CHASSIS_TL:  return ChVector<>(0.272, 0.962, 0);
  case UNIV_AXIS_LINK_LAT:    return ChVector<>(-0.978950, 0.204099, 0);
  case UNIV_AXIS_CHASSIS_LAT: return ChVector<>(-0.021990, -0.105472, 0.994179);
  default:                    return ChVector<>(0, 0, 1);
  }
}

