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
// Front and Rear HMMWV suspension subsystems (solid axle).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames having X pointing towards the rear, Y to the right, and Z up (as
// imposed by the base class ChSolidAxle) and origins at the midpoint between
// the wheel centers.
//
// =============================================================================

#include "HMMWV_SolidAxle.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static double scaleCon = 35.815 / 750;

static const double in2m = 0.0254;

const double     HMMWV_SolidAxleFront::m_ULMass = 12.0/2.2; 
const double     HMMWV_SolidAxleFront::m_LLMass = 36.0/2.2;
const double     HMMWV_SolidAxleFront::m_knuckleMass = 60.0/2.2;
const double     HMMWV_SolidAxleFront::m_spindleMass = 35.0/2.2;
const double     HMMWV_SolidAxleFront::m_axleTubeMass = 35.0/2.2;

const double     HMMWV_SolidAxleFront::m_spindleRadius = 0.15;
const double     HMMWV_SolidAxleFront::m_spindleWidth = 0.06;
const double     HMMWV_SolidAxleFront::m_ULRadius = 0.02;
const double     HMMWV_SolidAxleFront::m_LLRadius = 0.02;
const double     HMMWV_SolidAxleFront::m_axleTubeRadius = 0.03;
const double     HMMWV_SolidAxleFront::m_knuckleRadius = 0.01;

const ChVector<> HMMWV_SolidAxleFront::m_axleTubeCOM(0, 0, 0);

const ChVector<> HMMWV_SolidAxleFront::m_axleTubeInertia(2, 4, 2);
const ChVector<> HMMWV_SolidAxleFront::m_spindleInertia(2, 4, 2);
const ChVector<> HMMWV_SolidAxleFront::m_ULInertia(2, 2, 2);
const ChVector<> HMMWV_SolidAxleFront::m_LLInertia(3, 3, 3);
const ChVector<> HMMWV_SolidAxleFront::m_knuckleInertia(5, 5, 5);

const double     HMMWV_SolidAxleFront::m_axleInertia = 0.4;

const double     HMMWV_SolidAxleFront::m_springCoefficient  = 267062.0;
const double     HMMWV_SolidAxleFront::m_dampingCoefficient = 22459.0;
const double     HMMWV_SolidAxleFront::m_springRestLength   = in2m * 13.36;

// -----------------------------------------------------------------------------

const double     HMMWV_SolidAxleRear::m_ULMass = 12.0/2.2; 
const double     HMMWV_SolidAxleRear::m_LLMass = 36.0/2.2;
const double     HMMWV_SolidAxleRear::m_knuckleMass = 60.0/2.2;
const double     HMMWV_SolidAxleRear::m_spindleMass = 35.0/2.2;
const double     HMMWV_SolidAxleRear::m_axleTubeMass = 35.0/2.2;

const double     HMMWV_SolidAxleRear::m_spindleRadius = 0.15;
const double     HMMWV_SolidAxleRear::m_spindleWidth = 0.06;
const double     HMMWV_SolidAxleRear::m_ULRadius = 0.02;
const double     HMMWV_SolidAxleRear::m_LLRadius = 0.02;
const double     HMMWV_SolidAxleRear::m_axleTubeRadius = 0.03;
const double     HMMWV_SolidAxleRear::m_knuckleRadius = 0.01;

const ChVector<> HMMWV_SolidAxleRear::m_axleTubeCOM(0, 0, 0);

const ChVector<> HMMWV_SolidAxleRear::m_axleTubeInertia(2, 4, 2);
const ChVector<> HMMWV_SolidAxleRear::m_spindleInertia(2, 4, 2);
const ChVector<> HMMWV_SolidAxleRear::m_ULInertia(2, 2, 2);
const ChVector<> HMMWV_SolidAxleRear::m_LLInertia(3, 3, 3);
const ChVector<> HMMWV_SolidAxleRear::m_knuckleInertia(5, 5, 5);

const double     HMMWV_SolidAxleRear::m_axleInertia = 0.4;

const double     HMMWV_SolidAxleRear::m_springCoefficient  = 267062.0;
const double     HMMWV_SolidAxleRear::m_dampingCoefficient = 22459.0;
const double     HMMWV_SolidAxleRear::m_springRestLength   = in2m * 13.36;


// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
HMMWV_SolidAxleFront::HMMWV_SolidAxleFront(const std::string& name,
                                           bool               driven)
: ChSolidAxle(name, true, driven)
{
}

HMMWV_SolidAxleRear::HMMWV_SolidAxleRear(const std::string& name,
                                         bool               driven)
: ChSolidAxle(name, false, driven)
{
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> HMMWV_SolidAxleFront::getLocation(PointId which)
{
  switch (which) {  
  case AXLE_OUTER: return in2m * scaleCon * ChVector<>(0, 700, 0);
  case SHOCK_A:    return in2m * scaleCon * ChVector<>(65, 575, -25);
  case SHOCK_C:    return in2m * scaleCon * ChVector<>(80, 560, 300);
  case KNUCKLE_L:  return in2m * scaleCon * ChVector<>(-5, 700, -50);
  case KNUCKLE_U:  return in2m * scaleCon * ChVector<>(15, 675, 75);
  case LL_A:       return in2m * scaleCon * ChVector<>(-10, 600, -75);
  case LL_A_X:     return in2m * scaleCon * ChVector<>(-10, 600, 45);
  case LL_A_Z:     return in2m * scaleCon * ChVector<>(-10, 500, -55);
  case LL_C:       return in2m * scaleCon * ChVector<>(-450, 350, -45);
  case LL_C_X:     return in2m * scaleCon * ChVector<>(-450, 350, 55);
  case LL_C_Z:     return in2m * scaleCon * ChVector<>(-450, 500, -455);
  case UL_A:       return in2m * scaleCon * ChVector<>(55, 475, 150);
  case UL_A_X:     return in2m * scaleCon * ChVector<>(55, 475, 250);
  case UL_A_Z:     return in2m * scaleCon * ChVector<>(55, 400, 150);
  case UL_C:       return in2m * scaleCon * ChVector<>(-355, 500, 150);
  case UL_C_X:     return in2m * scaleCon * ChVector<>(-355, 500, 250);
  case UL_C_Z:     return in2m * scaleCon * ChVector<>(-355, 400, 150);
  case SPRING_A:   return in2m * scaleCon * ChVector<>(65, 575, -25);
  case SPRING_C:   return in2m * scaleCon * ChVector<>(80, 560, 300);
  case TIEROD_C:   return in2m * scaleCon * ChVector<>(75, 325, -65);
  case TIEROD_K:   return in2m * scaleCon * ChVector<>(75, 680, -65);
  case SPINDLE:    return in2m * scaleCon * ChVector<>(0, 750, 0);
  case KNUCKLE_CM: return in2m * scaleCon * ChVector<>(-5, 700, -50);
  case LL_CM:      return in2m * scaleCon * ChVector<>(-10, 600, -75);
  case UL_CM:      return in2m * scaleCon * ChVector<>(55, 475, 150);
  default:         return ChVector<>(0, 0, 0);
  }
}

const ChVector<> HMMWV_SolidAxleRear::getLocation(PointId which)
{
  switch (which) {
  case AXLE_OUTER: return in2m * scaleCon * ChVector<>(0, 700, 0);
  case SHOCK_A:    return in2m * scaleCon * ChVector<>(65, 575, -25);
  case SHOCK_C:    return in2m * scaleCon * ChVector<>(80, 560, 300);
  case KNUCKLE_L:  return in2m * scaleCon * ChVector<>(-5, 700, -50);
  case KNUCKLE_U:  return in2m * scaleCon * ChVector<>(15, 675, 75);
  case LL_A:       return in2m * scaleCon * ChVector<>(-10, 600, -75);
  case LL_A_X:     return in2m * scaleCon * ChVector<>(-10, 600, 45);
  case LL_A_Z:     return in2m * scaleCon * ChVector<>(-10, 500, -55);
  case LL_C:       return in2m * scaleCon * ChVector<>(-450, 350, -45);
  case LL_C_X:     return in2m * scaleCon * ChVector<>(-450, 350, 55);
  case LL_C_Z:     return in2m * scaleCon * ChVector<>(-450, 500, -455);
  case UL_A:       return in2m * scaleCon * ChVector<>(55, 475, 150);
  case UL_A_X:     return in2m * scaleCon * ChVector<>(55, 475, 250);
  case UL_A_Z:     return in2m * scaleCon * ChVector<>(55, 400, 150);
  case UL_C:       return in2m * scaleCon * ChVector<>(-355, 500, 150);
  case UL_C_X:     return in2m * scaleCon * ChVector<>(-355, 500, 250);
  case UL_C_Z:     return in2m * scaleCon * ChVector<>(-355, 400, 150);
  case SPRING_A:   return in2m * scaleCon * ChVector<>(65, 575, -25);
  case SPRING_C:   return in2m * scaleCon * ChVector<>(80, 560, 300);
  case TIEROD_C:   return in2m * scaleCon * ChVector<>(75, 325, -65);
  case TIEROD_K:   return in2m * scaleCon * ChVector<>(75, 680, -65);
  case SPINDLE:    return in2m * scaleCon * ChVector<>(0, 750, 0);
  case KNUCKLE_CM: return in2m * scaleCon * ChVector<>(-5, 700, -50);
  case LL_CM:      return in2m * scaleCon * ChVector<>(-10, 600, -75);
  case UL_CM:      return in2m * scaleCon * ChVector<>(55, 475, 150);
  default:         return ChVector<>(0, 0, 0);
  }
}


} // end namespace hmmwv
