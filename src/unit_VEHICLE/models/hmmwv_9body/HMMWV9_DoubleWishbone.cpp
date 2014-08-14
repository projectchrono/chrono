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
// Front and Rear HMMWV suspension subsystems (reduced double A-arm)
//
// =============================================================================

#include "assets/ChBoxShape.h"
#include "assets/ChColorAsset.h"

#include "HMMWV9_DoubleWishbone.h"

using namespace chrono;

namespace hmmwv9 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double     HMMWV9_DoubleWishboneFront::m_spindleMass = 1;
const double     HMMWV9_DoubleWishboneFront::m_uprightMass = 1;

const ChVector<> HMMWV9_DoubleWishboneFront::m_spindleInertia(1, 1, 1);
const ChVector<> HMMWV9_DoubleWishboneFront::m_uprightInertia(1, 1, 1);

const double     HMMWV9_DoubleWishboneFront::m_axleInertia = 0.4;

const ChVector<> HMMWV9_DoubleWishboneFront::m_uprightDims(0.1, 0.05, 0.1);

const double     HMMWV9_DoubleWishboneFront::m_springCoefficient  = 167062.0;
const double     HMMWV9_DoubleWishboneFront::m_dampingCoefficient = 22459.0;
const double     HMMWV9_DoubleWishboneFront::m_springRestLength   = 0.4562;

// -----------------------------------------------------------------------------

const double     HMMWV9_DoubleWishboneRear::m_spindleMass = 1;
const double     HMMWV9_DoubleWishboneRear::m_uprightMass = 1;

const ChVector<> HMMWV9_DoubleWishboneRear::m_spindleInertia(1, 1, 1);
const ChVector<> HMMWV9_DoubleWishboneRear::m_uprightInertia(1, 1, 1);

const double     HMMWV9_DoubleWishboneRear::m_axleInertia = 0.4;

const ChVector<> HMMWV9_DoubleWishboneRear::m_uprightDims(0.1, 0.05, 0.1);

const double     HMMWV9_DoubleWishboneRear::m_springCoefficient = 369149.0;
const double     HMMWV9_DoubleWishboneRear::m_dampingCoefficient = 35024.0;
const double     HMMWV9_DoubleWishboneRear::m_springRestLength = 0.4562;


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
  case SPINDLE:  return in2m * ChVector<>(0, 0, 0);
  case UPRIGHT:  return in2m * ChVector<>(0, -4.0000, 0);
  case UCA_F:    return in2m * ChVector<>(0.3000, -18.2600, 10.6650);
  case UCA_B:    return in2m * ChVector<>(8.9700, -18.2600, 8.7250);
  case UCA_U:    return in2m * ChVector<>(0.5000, -7.6500, 9.5150);
  case LCA_F:    return in2m * ChVector<>(-10.3800, -23.7200, 1.0350);
  case LCA_B:    return in2m * ChVector<>(7.2000, -23.7200, 1.0350);
  case LCA_U:    return in2m * ChVector<>(-0.1900, -4.8500, -3.6150);
  case SHOCK_C:  return in2m * ChVector<>(-5.6900, -7.9500, 13.7550);
  case SHOCK_U:  return in2m * ChVector<>(-5.4200, -4.8500, -0.4850);
  case TIEROD_C: return in2m * ChVector<>(11.8000, -26.0100, 0);
  case TIEROD_U: return in2m * ChVector<>(5.3300, -3.5000, 0);
  default:       return ChVector<>(0, 0, 0);
  }
}

const ChVector<> HMMWV9_DoubleWishboneRear::getLocation(PointId which)
{
  switch (which) {
  case SPINDLE:  return in2m * ChVector<>(0, 0, 0);
  case UPRIGHT:  return in2m * ChVector<>(0, -4.0000, 0);
  case UCA_F:    return in2m * ChVector<>(-1.6700, -17.620, 9.9150);
  case UCA_B:    return in2m * ChVector<>(-12.3800, -17.6200, 9.9150);
  case UCA_U:    return in2m * ChVector<>(0, -7.6500, 10.3150);
  case LCA_F:    return in2m * ChVector<>(10.1900, -23.7200, 1.0350);
  case LCA_B:    return in2m * ChVector<>(-7.3900, -23.7200, 1.0350);
  case LCA_U:    return in2m * ChVector<>(0, -4.8500, -3.6150);
  case SHOCK_C:  return in2m * ChVector<>(5.4900, -7.6200, 13.7550);
  case SHOCK_U:  return in2m * ChVector<>(5.4900, -4.8500, -0.4750);
  case TIEROD_C: return in2m * ChVector<>(-11.3000, -19.4400, 0.6650);
  case TIEROD_U: return in2m * ChVector<>(-5.3000, -3.4900, 0.6650);
  default:       return ChVector<>(0, 0, 0);
  }
}

// -----------------------------------------------------------------------------
// Implementations of the OnInitializeUpright() virtual methods.
// Add simple visualization for the upright body.
// -----------------------------------------------------------------------------

void HMMWV9_DoubleWishboneFront::OnInitializeUpright()
{
  ChSharedPtr<ChBoxShape> box(new ChBoxShape);
  box->GetBoxGeometry().SetLenghts(m_uprightDims);
  m_upright->AddAsset(box);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (m_side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.2f, 0.2f)); break;
  case LEFT:  col->SetColor(ChColor(0.2f, 0.6f, 0.2f)); break;
  }
  m_upright->AddAsset(col);
}

void HMMWV9_DoubleWishboneRear::OnInitializeUpright()
{
  ChSharedPtr<ChBoxShape> box(new ChBoxShape);
  box->GetBoxGeometry().SetLenghts(m_uprightDims);
  m_upright->AddAsset(box);

  ChSharedPtr<ChColorAsset> col(new ChColorAsset);
  switch (m_side) {
  case RIGHT: col->SetColor(ChColor(0.6f, 0.4f, 0.4f)); break;
  case LEFT:  col->SetColor(ChColor(0.4f, 0.6f, 0.6f)); break;
  }
  m_upright->AddAsset(col);
}


} // end namespace hmmwv9
