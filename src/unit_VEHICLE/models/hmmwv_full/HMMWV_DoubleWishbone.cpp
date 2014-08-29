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

const double     HMMWV_DoubleWishboneFront::m_UCAMass = 12.0/2.2; 
const double     HMMWV_DoubleWishboneFront::m_LCAMass = 36.0/2.2;
// entire wheel assembly = 195 lbs, includes upright, spindle and tire.
//	HMMWV tires run ~ 100 lbs, so the spindle and upright should be ~ 95 lbs combined
const double     HMMWV_DoubleWishboneFront::m_uprightMass = 60.0/2.2;
const double     HMMWV_DoubleWishboneFront::m_spindleMass = 35.0/2.2;

const double     HMMWV_DoubleWishboneFront::m_spindleRadius = 0.15;
const double     HMMWV_DoubleWishboneFront::m_spindleWidth = 0.06;
const double     HMMWV_DoubleWishboneFront::m_LCARadius = 0.03;	// LCA is much thicker than UCA
const double     HMMWV_DoubleWishboneFront::m_UCARadius = 0.02;
const double     HMMWV_DoubleWishboneFront::m_uprightRadius = 0.03;

const ChVector<> HMMWV_DoubleWishboneFront::m_spindleInertia(2, 4, 2); // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneFront::m_UCAInertia(2, 2, 2);  // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneFront::m_LCAInertia(3, 3, 3);  // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneFront::m_uprightInertia(5, 5, 5); // TODO: This is not the correct value

const double     HMMWV_DoubleWishboneFront::m_axleInertia = 0.4;

const double     HMMWV_DoubleWishboneFront::m_springCoefficient  = 167062.0;
const double     HMMWV_DoubleWishboneFront::m_dampingCoefficient = 22459.0;
// 13.36" + 4.65" (design free length + offset between upper shock, spring attachment points)
const double     HMMWV_DoubleWishboneFront::m_springRestLength   = 0.4575;
// const double     HMMWV_DoubleWishboneFront::m_springRestLength   = 0.4562;

// -----------------------------------------------------------------------------

const double     HMMWV_DoubleWishboneRear::m_UCAMass = 12.0/2.2;
const double     HMMWV_DoubleWishboneRear::m_LCAMass = 36.0/2.2;
// entire wheel assembly = 195 lbs, includes upright, spindle and tire.
//	HMMWV tires run ~ 100 lbs, so the spindle and upright should be ~ 95 lbs combined
const double     HMMWV_DoubleWishboneRear::m_uprightMass = 60.0/2.2;
const double     HMMWV_DoubleWishboneRear::m_spindleMass = 35.0/2.2;

const double     HMMWV_DoubleWishboneRear::m_spindleRadius = 0.15;
const double     HMMWV_DoubleWishboneRear::m_spindleWidth = 0.06;
const double     HMMWV_DoubleWishboneRear::m_LCARadius = 0.03; 	// LCA is much thicker than UCA
const double     HMMWV_DoubleWishboneRear::m_UCARadius = 0.02;
const double     HMMWV_DoubleWishboneRear::m_uprightRadius = 0.03;

const ChVector<> HMMWV_DoubleWishboneRear::m_spindleInertia(2, 4, 2);
const ChVector<> HMMWV_DoubleWishboneRear::m_UCAInertia(2, 2, 2);  // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneRear::m_LCAInertia(3, 3, 3);  // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneRear::m_uprightInertia(5, 5, 5);

const double     HMMWV_DoubleWishboneRear::m_axleInertia = 0.4;

const double     HMMWV_DoubleWishboneRear::m_springCoefficient = 369149.0;
const double     HMMWV_DoubleWishboneRear::m_dampingCoefficient = 35024.0;
// 15.03" + 2.35", using same logic as for the front suspension
const double     HMMWV_DoubleWishboneRear::m_springRestLength = 0.4414;
// const double     HMMWV_DoubleWishboneRear::m_springRestLength = 0.4562;


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
  case SPINDLE:  return in2m * ChVector<>(1.59, 23.72, -1.0350);
  case UPRIGHT:  return in2m * ChVector<>(1.59, 19.72, -1.0350);
  case UCA_F:    return in2m * ChVector<>(1.89, 5.46, 9.63);
  case UCA_B:    return in2m * ChVector<>(10.56, 5.46, 7.69);
  case UCA_U:    return in2m * ChVector<>(2.09, 16.07, 8.48);
  case UCA_CM:   return in2m * ChVector<>(4.85, 9.00, 8.60); // TODO: This is not the correct value
  case LCA_F:    return in2m * ChVector<>(-8.79, 0, 0);
  case LCA_B:    return in2m * ChVector<>(8.79, 0, 0);
  case LCA_U:    return in2m * ChVector<>(1.40, 18.87, -4.65);
  case LCA_CM:   return in2m * ChVector<>(0.47, 6.29, -1.55); // TODO: This is not the correct value
  case SHOCK_C:  return in2m * ChVector<>(-4.095, 7.508, 12.722);
  case SHOCK_U:  return in2m * ChVector<>(-3.827, 9.295, -1.835);
  case TIEROD_C: return in2m * ChVector<>(13.39, -2.29, -1.0350);
  case TIEROD_U: return in2m * ChVector<>(6.92, 20.22, -1.0350);
  default:       return ChVector<>(0, 0, 0);
  }
}

const ChVector<> HMMWV_DoubleWishboneRear::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(-1.40, 23.72, -1.035);
  case UPRIGHT:  return in2m * ChVector<>(-1.40, 19.72, -1.035);
  case UCA_F:    return in2m * ChVector<>(-13.78, 6.10, 8.88);
  case UCA_B:    return in2m * ChVector<>(-3.07, 6.10, 8.88);
  case UCA_U:    return in2m * ChVector<>(-1.40, 16.07, 9.28);
  case UCA_CM:   return in2m * ChVector<>(-6.08, 9.42, 9.01); // TODO: This is not the correct value
  case LCA_F:    return in2m * ChVector<>(-8.79, 0, 0);
  case LCA_B:    return in2m * ChVector<>(8.79, 0, 0);
  case LCA_U:    return in2m * ChVector<>(-1.40, 18.87, -4.65);
  case LCA_CM:   return in2m * ChVector<>(-0.47, 6.29, -1.55); // TODO: This is not the correct value
  case SHOCK_C:  return in2m * ChVector<>(4.095, 7.508, 12.722);
  case SHOCK_U:  return in2m * ChVector<>(3.827, 9.325, -1.511);
  case TIEROD_C: return in2m * ChVector<>(-12.70, 4.28, -0.37);
  case TIEROD_U: return in2m * ChVector<>(-6.70, 20.23, -0.37);
  default:       return ChVector<>(0, 0, 0);
  }
}


} // end namespace hmmwv
