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
// Generic concrete double wishbone suspension subsystem.
//
// This concrete suspension subsystem is defined with respect to a right-handed
// frame with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origin in the chassis midplane.
//
// All point locations are provided for the left half of the supspension.
//
// =============================================================================

#include "models/generic/Generic_DoubleWishbone.h"

using namespace chrono;


// -----------------------------------------------------------------------------
// Static variables -- all in SI units
// -----------------------------------------------------------------------------

const double     Generic_DoubleWishbone::m_spindleMass = 1.103;
const double     Generic_DoubleWishbone::m_uprightMass = 1.397;
const double     Generic_DoubleWishbone::m_UCAMass = 1.032;
const double     Generic_DoubleWishbone::m_LCAMass = 1.611;

const double     Generic_DoubleWishbone::m_spindleRadius = 0.15;
const double     Generic_DoubleWishbone::m_spindleWidth = 0.06;
const double     Generic_DoubleWishbone::m_uprightRadius = 0.025;
const double     Generic_DoubleWishbone::m_UCARadius = 0.02;
const double     Generic_DoubleWishbone::m_LCARadius = 0.03;

const ChVector<> Generic_DoubleWishbone::m_spindleInertia(0.000478, 0.000496, 0.000478);
const ChVector<> Generic_DoubleWishbone::m_uprightInertia(0.0138, 0.0146, 0.00283);
const ChVector<> Generic_DoubleWishbone::m_UCAInertia(0.00591, 0.00190, 0.00769);
const ChVector<> Generic_DoubleWishbone::m_LCAInertia(0.0151, 0.0207, 0.0355);

const double     Generic_DoubleWishbone::m_axleInertia = 0.4;

const double     Generic_DoubleWishbone::m_springCoefficient = 369149.000;
const double     Generic_DoubleWishbone::m_dampingCoefficient = 22459.000;
const double     Generic_DoubleWishbone::m_springRestLength = 0.306;


// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual method.
// This function returns the position of the specified suspension hardpoint,
// with respect to the suspension subsystem's reference frame (a right-hand
// frame with X pointing towards the front, Y to the left, and Z up and with
// its origin and origin in the chassis midplane. The hardpoints returned by this
// function are for the left half of the suspension only.
// -----------------------------------------------------------------------------

const ChVector<> Generic_DoubleWishbone::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return ChVector<>(-0.040, 1.100, -0.026);  // location of spindle center of mass
  case UPRIGHT:  return ChVector<>(-0.040, 0.880, -0.026);  // location of upright center of mass
  case UCA_F:    return ChVector<>(0.160, 0.539, 0.243);   // UCA front connection point to chassis
  case UCA_B:    return ChVector<>(-0.339, 0.587, 0.249);   // UCA rear (back) connection point to chassis
  case UCA_U:    return ChVector<>(-0.088, 0.808, 0.243);   // UCA connection point to upright
  case UCA_CM:   return ChVector<>(-0.196, 0.645, 0.245);   // location of UCA center of mass
  case LCA_F:    return ChVector<>(0.199, 0.479, -0.206);   // LCA front connection point to chassis
  case LCA_B:    return ChVector<>(-0.279, 0.539, -0.200);  // LCA rear (back) connection point to chassis
  case LCA_U:    return ChVector<>(-0.040, 0.898, -0.265);  // LCA connection point to upright
  case LCA_CM:   return ChVector<>(-0.040, 0.639, -0.224);  // location of LCA center of mass
  case SHOCK_C:  return ChVector<>(-0.088, 0.599, 0.393);   // shock connection to chassis
  case SHOCK_A:  return ChVector<>(-0.040, 0.718, -0.206);  // shock connection point to LCA
  case SPRING_C: return ChVector<>(-0.064, 0.659, 0.094);   // spring connection point to chassis
  case SPRING_A: return ChVector<>(-0.040, 0.718, -0.206);  // spring connection point to LCA
  case TIEROD_C: return ChVector<>(-0.279, 0.479, -0.026);  // tierod connection point to chassis
  case TIEROD_U: return ChVector<>(-0.220, 0.898, -0.026);  // tierod connection point to upright
  default:       return ChVector<>(0, 0, 0);
  }
}


