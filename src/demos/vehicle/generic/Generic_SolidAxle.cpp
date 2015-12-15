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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Generic solid axle suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChSolidAxle) and origin at the midpoint between the wheel
// centers.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include "generic/Generic_SolidAxle.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double     Generic_SolidAxle::m_ULMass = 1.446;
const double     Generic_SolidAxle::m_LLMass = 2.892;
const double     Generic_SolidAxle::m_knuckleMass = 1.356;
const double     Generic_SolidAxle::m_spindleMass = 0.248;
const double     Generic_SolidAxle::m_axleTubeMass = 44.958;

const double     Generic_SolidAxle::m_spindleRadius = 0.06;
const double     Generic_SolidAxle::m_spindleWidth = 0.04;
const double     Generic_SolidAxle::m_ULRadius = 0.02;
const double     Generic_SolidAxle::m_LLRadius = 0.02;
const double     Generic_SolidAxle::m_axleTubeRadius = 0.03;
const double     Generic_SolidAxle::m_knuckleRadius = 0.01;

const ChVector<> Generic_SolidAxle::m_axleTubeCOM(0, 0, 0);

const ChVector<> Generic_SolidAxle::m_axleTubeInertia(7.744, 0.045, 7.744);
const ChVector<> Generic_SolidAxle::m_spindleInertia(0.0000558, 0.0000279, 0.0000558);
const ChVector<> Generic_SolidAxle::m_ULInertia(0.011, 0.011, 0.000142);
const ChVector<> Generic_SolidAxle::m_LLInertia(0.0514, 0.0514, 0.00037);
const ChVector<> Generic_SolidAxle::m_knuckleInertia(0.00255, 0.00134, 0.00196);

const double     Generic_SolidAxle::m_axleInertia = 0.4;

const double     Generic_SolidAxle::m_springCoefficient  = 267062.0;
const double     Generic_SolidAxle::m_dampingCoefficient = 22459.0;
const double     Generic_SolidAxle::m_springRestLength = 0.3948;

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------
Generic_SolidAxle::Generic_SolidAxle(const std::string& name)
: ChSolidAxle(name)
{
  m_springForceCB = new LinearSpringForce(m_springCoefficient);
  m_shockForceCB = new LinearDamperForce(m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructor
// -----------------------------------------------------------------------------
Generic_SolidAxle::~Generic_SolidAxle()
{
  delete m_springForceCB;
  delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Implementation of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Generic_SolidAxle::getLocation(PointId which)
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
