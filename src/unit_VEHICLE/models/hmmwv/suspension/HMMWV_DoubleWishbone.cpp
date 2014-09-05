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
// midpoint between the lower control arms' connection points to the chassis.
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
// HMMWV tires run ~ 100 lbs, so the spindle and upright should be ~ 95 lbs combined
const double     HMMWV_DoubleWishboneFront::m_uprightMass = 60.0/2.2;
const double     HMMWV_DoubleWishboneFront::m_spindleMass = 35.0/2.2;

const double     HMMWV_DoubleWishboneFront::m_spindleRadius = 0.15;
const double     HMMWV_DoubleWishboneFront::m_spindleWidth = 0.06;
const double     HMMWV_DoubleWishboneFront::m_LCARadius = 0.03;        // LCA is much thicker than UCA
const double     HMMWV_DoubleWishboneFront::m_UCARadius = 0.02;
const double     HMMWV_DoubleWishboneFront::m_uprightRadius = 0.03;

const ChVector<> HMMWV_DoubleWishboneFront::m_spindleInertia(2, 4, 2); // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneFront::m_UCAInertia(2, 2, 2);     // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneFront::m_LCAInertia(3, 3, 3);     // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneFront::m_uprightInertia(5, 5, 5); // TODO: This is not the correct value

const double     HMMWV_DoubleWishboneFront::m_axleInertia = 0.4;

const double     HMMWV_DoubleWishboneFront::m_springCoefficient  = 167062.0;
const double     HMMWV_DoubleWishboneFront::m_dampingCoefficient = 22459.0;
const double     HMMWV_DoubleWishboneFront::m_springRestLength   = in2m * 13.36;

// -----------------------------------------------------------------------------

const double     HMMWV_DoubleWishboneRear::m_UCAMass = 12.0/2.2;
const double     HMMWV_DoubleWishboneRear::m_LCAMass = 36.0/2.2;
// entire wheel assembly = 195 lbs, includes upright, spindle and tire.
// HMMWV tires run ~ 100 lbs, so the spindle and upright should be ~ 95 lbs combined
const double     HMMWV_DoubleWishboneRear::m_uprightMass = 60.0/2.2;
const double     HMMWV_DoubleWishboneRear::m_spindleMass = 35.0/2.2;

const double     HMMWV_DoubleWishboneRear::m_spindleRadius = 0.15;
const double     HMMWV_DoubleWishboneRear::m_spindleWidth = 0.06;
const double     HMMWV_DoubleWishboneRear::m_LCARadius = 0.03;         // LCA is much thicker than UCA
const double     HMMWV_DoubleWishboneRear::m_UCARadius = 0.02;
const double     HMMWV_DoubleWishboneRear::m_uprightRadius = 0.03;

const ChVector<> HMMWV_DoubleWishboneRear::m_spindleInertia(2, 4, 2);  // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneRear::m_UCAInertia(2, 2, 2);      // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneRear::m_LCAInertia(3, 3, 3);      // TODO: This is not the correct value
const ChVector<> HMMWV_DoubleWishboneRear::m_uprightInertia(5, 5, 5);  // TODO: This is not the correct value

const double     HMMWV_DoubleWishboneRear::m_axleInertia = 0.4;

const double     HMMWV_DoubleWishboneRear::m_springCoefficient = 369149.0;
const double     HMMWV_DoubleWishboneRear::m_dampingCoefficient = 35024.0;
const double     HMMWV_DoubleWishboneRear::m_springRestLength = in2m * 15.03;


// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
HMMWV_DoubleWishboneFront::HMMWV_DoubleWishboneFront(const std::string& name,
                                                     bool               driven)
: ChDoubleWishbone(name, true, driven)
{
}

HMMWV_DoubleWishboneRear::HMMWV_DoubleWishboneRear(const std::string& name,
                                                   bool               driven)
: ChDoubleWishbone(name, false, driven)
{
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> HMMWV_DoubleWishboneFront::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(1.59, 35.815, -1.035);
  case UPRIGHT:  return in2m * ChVector<>(1.59, 29.5675, -1.0350);
  case UCA_F:    return in2m * ChVector<>(1.8864, 17.5575, 9.6308);
  case UCA_B:    return in2m * ChVector<>(10.5596, 18.8085, 7.6992);
  case UCA_U:    return in2m * ChVector<>(2.088, 28.17, 8.484);
  case UCA_CM:   return in2m * ChVector<>(4.155, 23.176, 8.575);
  case LCA_F:    return in2m * ChVector<>(-8.7900, 12.09, 0);
  case LCA_B:    return in2m * ChVector<>(8.7900, 12.09, 0);
  case LCA_U:    return in2m * ChVector<>(1.40, 30.965, -4.65);
  case LCA_CM:   return in2m * ChVector<>(0, 21.528, -2.325);
  case SHOCK_C:  return in2m * ChVector<>(-4.095, 19.598, 12.722);
  case SHOCK_A:  return in2m * ChVector<>(-3.827, 21.385, -1.835);
  case SPRING_C: return in2m * ChVector<>(-4.095, 20.07, 7.775);
  case SPRING_A: return in2m * ChVector<>(-3.827, 21.385, -1.835);
  case TIEROD_C: return in2m * ChVector<>(9.855, 17.655, 2.135);
  case TIEROD_U: return in2m * ChVector<>(6.922, 32.327, -0.643);
  default:       return ChVector<>(0, 0, 0);
  }
}

const ChVector<> HMMWV_DoubleWishboneRear::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(-1.40, 35.815, -1.035);
  case UPRIGHT:  return in2m * ChVector<>(-1.40, 29.5675, -1.035);
  case UCA_F:    return in2m * ChVector<>(-13.7445, 18.1991, 8.9604);
  case UCA_B:    return in2m * ChVector<>(-3.0355, 18.1909, 8.8096);
  case UCA_U:    return in2m * ChVector<>(-1.40, 28.17, 8.5);
  case UCA_CM:   return in2m * ChVector<>(-4.895, 23.183, 8.692);
  case LCA_F:    return in2m * ChVector<>(-8.7900, 12.09, 0);
  case LCA_B:    return in2m * ChVector<>(8.7900, 12.09, 0);
  case LCA_U:    return in2m * ChVector<>(-1.40, 30.965, -4.650);
  case LCA_CM:   return in2m * ChVector<>(0, 21.527, -2.325);
  case SHOCK_C:  return in2m * ChVector<>(4.095, 19.598, 12.722);
  case SHOCK_A:  return in2m * ChVector<>(3.827, 21.415, -1.511);
  case SPRING_C: return in2m * ChVector<>(4.095, 19.747, 10.098);
  case SPRING_A: return in2m * ChVector<>(3.827, 21.385, -1.835);
  case TIEROD_C: return in2m * ChVector<>(-8.790, 16.38, 2.310);
  case TIEROD_U: return in2m * ChVector<>(-6.704, 32.327, -0.365);
  default:       return ChVector<>(0, 0, 0);
  }
}


} // end namespace hmmwv
