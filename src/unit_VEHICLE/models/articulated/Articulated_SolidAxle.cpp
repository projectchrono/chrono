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
// Front and Rear solid axle suspension subsystems.
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origins at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include "models/articulated/Articulated_SolidAxle.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double     Articulated_SolidAxleFront::m_ULMass = 1.446;
const double     Articulated_SolidAxleFront::m_LLMass = 2.892;
const double     Articulated_SolidAxleFront::m_knuckleMass = 1.356;
const double     Articulated_SolidAxleFront::m_spindleMass = 0.248;
const double     Articulated_SolidAxleFront::m_axleTubeMass = 44.958;

const double     Articulated_SolidAxleFront::m_spindleRadius = 0.06;
const double     Articulated_SolidAxleFront::m_spindleWidth = 0.04;
const double     Articulated_SolidAxleFront::m_ULRadius = 0.02;
const double     Articulated_SolidAxleFront::m_LLRadius = 0.02;
const double     Articulated_SolidAxleFront::m_axleTubeRadius = 0.03;
const double     Articulated_SolidAxleFront::m_knuckleRadius = 0.01;

const ChVector<> Articulated_SolidAxleFront::m_axleTubeCOM(0, 0, 0);

const ChVector<> Articulated_SolidAxleFront::m_axleTubeInertia(7.744, 0.045, 7.744);
const ChVector<> Articulated_SolidAxleFront::m_spindleInertia(0.0000558, 0.0000279, 0.0000558);
const ChVector<> Articulated_SolidAxleFront::m_ULInertia(0.011, 0.011, 0.000142);
const ChVector<> Articulated_SolidAxleFront::m_LLInertia(0.0514, 0.0514, 0.00037);
const ChVector<> Articulated_SolidAxleFront::m_knuckleInertia(0.00255, 0.00134, 0.00196);

const double     Articulated_SolidAxleFront::m_axleInertia = 0.4;

const double     Articulated_SolidAxleFront::m_springCoefficient  = 267062.0;
const double     Articulated_SolidAxleFront::m_dampingCoefficient = 22459.0;
const double     Articulated_SolidAxleFront::m_springRestLength = 0.3948;

// -----------------------------------------------------------------------------

const double     Articulated_SolidAxleRear::m_ULMass = 1.446;
const double     Articulated_SolidAxleRear::m_LLMass = 2.892;
const double     Articulated_SolidAxleRear::m_knuckleMass = 1.356;
const double     Articulated_SolidAxleRear::m_spindleMass = 0.248;
const double     Articulated_SolidAxleRear::m_axleTubeMass = 44.958;

const double     Articulated_SolidAxleRear::m_spindleRadius = 0.06;
const double     Articulated_SolidAxleRear::m_spindleWidth = 0.04;
const double     Articulated_SolidAxleRear::m_ULRadius = 0.02;
const double     Articulated_SolidAxleRear::m_LLRadius = 0.02;
const double     Articulated_SolidAxleRear::m_axleTubeRadius = 0.03;
const double     Articulated_SolidAxleRear::m_knuckleRadius = 0.01;

const ChVector<> Articulated_SolidAxleRear::m_axleTubeCOM(0, 0, 0);

const ChVector<> Articulated_SolidAxleRear::m_axleTubeInertia(7.744, 0.045, 7.744);
const ChVector<> Articulated_SolidAxleRear::m_spindleInertia(0.0000558, 0.0000279, 0.0000558);
const ChVector<> Articulated_SolidAxleRear::m_ULInertia(0.011, 0.011, 0.000142);
const ChVector<> Articulated_SolidAxleRear::m_LLInertia(0.0514, 0.0514, 0.00037);
const ChVector<> Articulated_SolidAxleRear::m_knuckleInertia(0.00255, 0.00134, 0.00196);

const double     Articulated_SolidAxleRear::m_axleInertia = 0.4;

const double     Articulated_SolidAxleRear::m_springCoefficient = 267062.0;
const double     Articulated_SolidAxleRear::m_dampingCoefficient = 22459.0;
const double     Articulated_SolidAxleRear::m_springRestLength = 0.3948;


// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Articulated_SolidAxleFront::Articulated_SolidAxleFront(const std::string& name)
: ChSolidAxle(name)
{
}

Articulated_SolidAxleRear::Articulated_SolidAxleRear(const std::string& name)
: ChSolidAxle(name)
{
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Articulated_SolidAxleFront::getLocation(PointId which)
{
  switch (which) {  
  case SHOCK_A:    return ChVector<>(-0.079, 0.697, -0.030);
  case SHOCK_C:    return ChVector<>(-0.097, 0.679, 0.364);
  case KNUCKLE_L:  return ChVector<>(0.006, 0.849, -0.061);
  case KNUCKLE_U:  return ChVector<>(-0.018, 0.819, 0.091);
  case LL_A:       return ChVector<>(0.012, 0.728, -0.091);
  case LL_C:       return ChVector<>(0.546, 0.425, -0.055);
  case UL_A:       return ChVector<>(-0.067, 0.576, 0.182);
  case UL_C:       return ChVector<>(0.431, 0.606, 0.182);
  case SPRING_A:   return ChVector<>(-0.079, 0.697, -0.030);
  case SPRING_C:   return ChVector<>(-0.097, 0.679, 0.364);
  case TIEROD_C:   return ChVector<>(-0.091, 0.400, -0.079);
  case TIEROD_K:   return ChVector<>(-0.091, 0.825, -0.079);
  case SPINDLE:    return ChVector<>(0, 0.910, 0);
  case KNUCKLE_CM: return ChVector<>(-0.006, 0.834, 0.015);
  case LL_CM:      return ChVector<>(0.279, 0.577, -0.073);
  case UL_CM:      return ChVector<>(0.182, 0.591, 0.182);
  default:         return ChVector<>(0, 0, 0);
  }
}

const ChVector<> Articulated_SolidAxleRear::getLocation(PointId which)
{
  switch (which) {
  case SHOCK_A:    return ChVector<>(-0.079, 0.697, -0.030);
  case SHOCK_C:    return ChVector<>(-0.097, 0.679, 0.364);
  case KNUCKLE_L:  return ChVector<>(0.006, 0.849, -0.061);
  case KNUCKLE_U:  return ChVector<>(-0.018, 0.819, 0.091);
  case LL_A:       return ChVector<>(0.012, 0.728, -0.091);
  case LL_C:       return ChVector<>(0.546, 0.425, -0.055);
  case UL_A:       return ChVector<>(-0.067, 0.576, 0.182);
  case UL_C:       return ChVector<>(0.431, 0.606, 0.182);
  case SPRING_A:   return ChVector<>(-0.079, 0.697, -0.030);
  case SPRING_C:   return ChVector<>(-0.097, 0.679, 0.364);
  case TIEROD_C:   return ChVector<>(-0.091, 0.400, -0.079);
  case TIEROD_K:   return ChVector<>(-0.091, 0.825, -0.079);
  case SPINDLE:    return ChVector<>(0, 0.910, 0);
  case KNUCKLE_CM: return ChVector<>(-0.006, 0.834, 0.015);
  case LL_CM:      return ChVector<>(0.279, 0.577, -0.073);
  case UL_CM:      return ChVector<>(0.182, 0.591, 0.182);
  default:         return ChVector<>(0, 0, 0);
  }
}

// -----------------------------------------------------------------------------
// Implementations of the getDirection() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Articulated_SolidAxleFront::getDirection(DirectionId which)
{
  switch (which) {
  case UNIV_AXIS_LINK_L:      return ChVector<>(0, 1, 0);
  case UNIV_AXIS_CHASSIS_L:   return ChVector<>(0, 0, 1);
  case UNIV_AXIS_LINK_U:      return ChVector<>(0, -1, 0);
  case UNIV_AXIS_CHASSIS_U:   return ChVector<>(0, 0, 1);
  default:                    return ChVector<>(0, 0, 1);
  }
}

const ChVector<> Articulated_SolidAxleRear::getDirection(DirectionId which)
{
  switch (which) {
  case UNIV_AXIS_LINK_L:      return ChVector<>(0, 1, 0);
  case UNIV_AXIS_CHASSIS_L:   return ChVector<>(0, 0, 1);
  case UNIV_AXIS_LINK_U:      return ChVector<>(0, -1, 0);
  case UNIV_AXIS_CHASSIS_U:   return ChVector<>(0, 0, 1);
  default:                    return ChVector<>(0, 0, 1);
  }
}

