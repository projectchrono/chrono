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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (reduced double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames having X pointing towards the rear, Y to the right, and Z up (as
// imposed by the base class ChDoubleWishboneReduced) and origins at the 
// midpoint between the lower control arm's connection points to the chassis.
//
// =============================================================================

#include "HMMWV9_DoubleWishbone.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double     HMMWV9_DoubleWishboneFront::m_spindleMass = 1;
const double     HMMWV9_DoubleWishboneFront::m_uprightMass = 1;

const double     HMMWV9_DoubleWishboneFront::m_uprightRadius = 0.02;

const ChVector<> HMMWV9_DoubleWishboneFront::m_spindleInertia(1, 1, 1);
const ChVector<> HMMWV9_DoubleWishboneFront::m_uprightInertia(5, 5, 5);

const double     HMMWV9_DoubleWishboneFront::m_axleInertia = 0.4;

const double     HMMWV9_DoubleWishboneFront::m_springCoefficient  = 167062.0;
const double     HMMWV9_DoubleWishboneFront::m_dampingCoefficient = 22459.0;
const double     HMMWV9_DoubleWishboneFront::m_springRestLength   = 0.4062;

// -----------------------------------------------------------------------------

const double     HMMWV9_DoubleWishboneRear::m_spindleMass = 1;
const double     HMMWV9_DoubleWishboneRear::m_uprightMass = 1;

const double     HMMWV9_DoubleWishboneRear::m_uprightRadius = 0.02;

const ChVector<> HMMWV9_DoubleWishboneRear::m_spindleInertia(1, 1, 1);
const ChVector<> HMMWV9_DoubleWishboneRear::m_uprightInertia(5, 5, 5);

const double     HMMWV9_DoubleWishboneRear::m_axleInertia = 0.4;

const double     HMMWV9_DoubleWishboneRear::m_springCoefficient = 369149.0;
const double     HMMWV9_DoubleWishboneRear::m_dampingCoefficient = 35024.0;
const double     HMMWV9_DoubleWishboneRear::m_springRestLength = 0.4162;


// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
HMMWV9_DoubleWishboneFront::HMMWV9_DoubleWishboneFront(const std::string& name,
                                                       ChSuspension::Side side,
                                                       bool               driven)
: ChDoubleWishboneReduced(name, side, driven)
{
}

HMMWV9_DoubleWishboneRear::HMMWV9_DoubleWishboneRear(const std::string& name,
                                                     ChSuspension::Side side,
                                                     bool               driven)
: ChDoubleWishboneReduced(name, side, driven)
{
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> HMMWV9_DoubleWishboneFront::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(1.59, 23.72, -1.0350);
  case UPRIGHT:  return in2m * ChVector<>(1.59, 19.72, -1.0350);
  case UCA_F:    return in2m * ChVector<>(1.89, 5.46, 9.63);
  case UCA_B:    return in2m * ChVector<>(10.56, 5.46, 7.69);
  case UCA_U:    return in2m * ChVector<>(2.09, 16.07, 8.48);
  case LCA_F:    return in2m * ChVector<>(-8.79, 0, 0);
  case LCA_B:    return in2m * ChVector<>(8.79, 0, 0);
  case LCA_U:    return in2m * ChVector<>(1.40, 18.87, -4.65);
  case SHOCK_C:  return in2m * ChVector<>(-4.10, 15.77, 12.72);
  case SHOCK_U:  return in2m * ChVector<>(-3.83, 18.87, -1.52);
  case TIEROD_C: return in2m * ChVector<>(13.39, -2.29, -1.0350);
  case TIEROD_U: return in2m * ChVector<>(6.92, 20.22, -1.0350);
  default:       return ChVector<>(0, 0, 0);
  }
}

const ChVector<> HMMWV9_DoubleWishboneRear::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(-1.40, 23.72, -1.035);
  case UPRIGHT:  return in2m * ChVector<>(-1.40, 19.72, -1.035);
  case UCA_F:    return in2m * ChVector<>(-3.07, 6.10, 8.88);
  case UCA_B:    return in2m * ChVector<>(-13.78, 6.10, 8.88);
  case UCA_U:    return in2m * ChVector<>(-1.40, 16.07, 9.28);
  case LCA_F:    return in2m * ChVector<>(8.79, 0, 0);
  case LCA_B:    return in2m * ChVector<>(-8.79, 0, 0);
  case LCA_U:    return in2m * ChVector<>(-1.40, 18.87, -4.65);
  case SHOCK_C:  return in2m * ChVector<>(4.09, 16.10, 12.72);
  case SHOCK_U:  return in2m * ChVector<>(4.09, 18.87, -1.51);
  case TIEROD_C: return in2m * ChVector<>(-12.70, 4.28, -0.37);
  case TIEROD_U: return in2m * ChVector<>(-6.70, 20.23, -0.37);
  default:       return ChVector<>(0, 0, 0);
  }
}


} // end namespace hmmwv
