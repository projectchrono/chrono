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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames having X pointing towards the rear, Y to the right, and Z up (as
// imposed by the base class ChDoubleWishbone) and origins at the 
// midpoint between the lower control arm's connection points to the chassis.
//
// =============================================================================

#include "HMMWV_DoubleWishbone.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double     HMMWV_DoubleWishboneFront::m_spindleMass = 1; // There is no data for this in the HMMWV description
const double     HMMWV_DoubleWishboneFront::m_UCAMass = 5.446;
const double     HMMWV_DoubleWishboneFront::m_LCAMass = 16.32;
const double     HMMWV_DoubleWishboneFront::m_uprightMass = 1; // There is no data for this in the HMMWV description

const double     HMMWV_DoubleWishboneFront::m_spindleRadius = 0.15;
const double     HMMWV_DoubleWishboneFront::m_spindleWidth = 0.06;
const double     HMMWV_DoubleWishboneFront::m_LCARadius = 0.02;
const double     HMMWV_DoubleWishboneFront::m_UCARadius = 0.02;
const double     HMMWV_DoubleWishboneFront::m_uprightRadius = 0.02;

const ChVector<> HMMWV_DoubleWishboneFront::m_spindleInertia(0.113, 0.113, 0.113);
const ChVector<> HMMWV_DoubleWishboneFront::m_UCAInertia(0.113, 0.113, 0.113);
const ChVector<> HMMWV_DoubleWishboneFront::m_LCAInertia(0.113, 0.113, 0.113);
const ChVector<> HMMWV_DoubleWishboneFront::m_uprightInertia(1, 1, 1); // There is no data for this in the HMMWV description

const double     HMMWV_DoubleWishboneFront::m_axleInertia = 0.4;

const double     HMMWV_DoubleWishboneFront::m_springCoefficient  = 166546.0;
const double     HMMWV_DoubleWishboneFront::m_dampingCoefficient = 17513; // This value is actually associated with the tire damping
const double     HMMWV_DoubleWishboneFront::m_springRestLength   = in2m * 13.360;

// -----------------------------------------------------------------------------

const double     HMMWV_DoubleWishboneRear::m_spindleMass = 1; // There is no data for this in the HMMWV description
const double     HMMWV_DoubleWishboneRear::m_UCAMass = 5.446;
const double     HMMWV_DoubleWishboneRear::m_LCAMass = 16.32;
const double     HMMWV_DoubleWishboneRear::m_uprightMass = 1; // There is no data for this in the HMMWV description

const double     HMMWV_DoubleWishboneRear::m_spindleRadius = 0.15;
const double     HMMWV_DoubleWishboneRear::m_spindleWidth = 0.06;
const double     HMMWV_DoubleWishboneRear::m_LCARadius = 0.02;
const double     HMMWV_DoubleWishboneRear::m_UCARadius = 0.02;
const double     HMMWV_DoubleWishboneRear::m_uprightRadius = 0.02;

const ChVector<> HMMWV_DoubleWishboneRear::m_spindleInertia(0.113, 0.113, 0.113);
const ChVector<> HMMWV_DoubleWishboneRear::m_UCAInertia(0.113, 0.113, 0.113);
const ChVector<> HMMWV_DoubleWishboneRear::m_LCAInertia(0.113, 0.113, 0.113);
const ChVector<> HMMWV_DoubleWishboneRear::m_uprightInertia(1, 1, 1); // There is no data for this in the HMMWV description

const double     HMMWV_DoubleWishboneRear::m_axleInertia = 0.4;

const double     HMMWV_DoubleWishboneRear::m_springCoefficient = 369167.0;
const double     HMMWV_DoubleWishboneRear::m_dampingCoefficient = 17513; // This value is actually associated with the tire damping
const double     HMMWV_DoubleWishboneRear::m_springRestLength = in2m * 15.03;


// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
HMMWV_DoubleWishboneFront::HMMWV_DoubleWishboneFront(const std::string& name,
                                                       ChSuspension::Side side,
                                                       bool               driven)
: ChDoubleWishbone(name, side, driven)
{
}

HMMWV_DoubleWishboneRear::HMMWV_DoubleWishboneRear(const std::string& name,
                                                     ChSuspension::Side side,
                                                     bool               driven)
: ChDoubleWishbone(name, side, driven)
{
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> HMMWV_DoubleWishboneFront::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(1.59, 23.725, -1.035);
  case UPRIGHT:  return in2m * ChVector<>(1.59, 17.4775, -1.0350);
  case UCA_F:    return in2m * ChVector<>(1.89, 5.468, 9.630);
  case UCA_B:    return in2m * ChVector<>(6.223, 6.093, 8.665);
  case UCA_U:    return in2m * ChVector<>(2.088, 16.08, 8.484);
  case UCA_CM:   return in2m * ChVector<>(4.155, 11.086, 8.575);
  case LCA_F:    return in2m * ChVector<>(-1, 0, 0);
  case LCA_B:    return in2m * ChVector<>(0, 0, 0);
  case LCA_U:    return in2m * ChVector<>(1.40, 18.875, -4.65);
  case LCA_CM:   return in2m * ChVector<>(0, 9.438, -2.325);
  case SHOCK_C:  return in2m * ChVector<>(-4.095, 7.980, 7.775);  // This vector is actually associated with the SPRING bodies in the HMMWV description
  case SHOCK_U:  return in2m * ChVector<>(-3.827, 9.295, -1.835);  // This vector is actually associated with the SPRING bodies in the HMMWV description
  case TIEROD_C: return in2m * ChVector<>(9.855, 5.565, 2.135);
  case TIEROD_U: return in2m * ChVector<>(6.922, 20.237, -0.643);
  default:       return ChVector<>(0, 0, 0);
  }
}

const ChVector<> HMMWV_DoubleWishboneRear::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(-1.40, 23.725, -1.035);
  case UPRIGHT:  return in2m * ChVector<>(-1.40, 17.4775, -1.035);
  case UCA_F:    return in2m * ChVector<>(-9.390, 6.105, 8.885);
  case UCA_B:    return in2m * ChVector<>(-8.390, 6.105, 8.885);
  case UCA_U:    return in2m * ChVector<>(-1.40, 16.08, 8.5);
  case UCA_CM:   return in2m * ChVector<>(-4.895, 11.093, 8.692);
  case LCA_F:    return in2m * ChVector<>(-1, 0, 0);
  case LCA_B:    return in2m * ChVector<>(0, 0, 0);
  case LCA_U:    return in2m * ChVector<>(-1.40, 18.875, -4.650);
  case LCA_CM:   return in2m * ChVector<>(0, 9.437, -2.325);
  case SHOCK_C:  return in2m * ChVector<>(4.095, 7.657, 10.098);  // This vector is actually associated with the SPRING bodies in the HMMWV description
  case SHOCK_U:  return in2m * ChVector<>(3.827, 9.295, -1.835);  // This vector is actually associated with the SPRING bodies in the HMMWV description
  case TIEROD_C: return in2m * ChVector<>(-8.790, 4.290, 2.310);
  case TIEROD_U: return in2m * ChVector<>(-6.704, 20.237, -0.365);
  default:       return ChVector<>(0, 0, 0);
  }
}


} // end namespace hmmwv
