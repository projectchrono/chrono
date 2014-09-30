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
// HMMWV Pitman arm steering model.
//
// =============================================================================

#include "models/hmmwv/steering/HMMWV_PitmanArm.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double      HMMWV_PitmanArm::m_steeringLinkMass = 9.072;
const double      HMMWV_PitmanArm::m_pitmanArmMass = 2.259;

const double      HMMWV_PitmanArm::m_steeringLinkRadius = 0.03;
const double      HMMWV_PitmanArm::m_pitmanArmRadius = 0.02;

const double      HMMWV_PitmanArm::m_maxAngle = 50.0 * (CH_C_PI / 180);

const ChVector<>  HMMWV_PitmanArm::m_steeringLinkInertia(1, 1, 1);
const ChVector<>  HMMWV_PitmanArm::m_pitmanArmInertia(1, 1, 1);


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_PitmanArm::HMMWV_PitmanArm(const std::string& name)
: ChPitmanArm(name)
{
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------
const ChVector<> HMMWV_PitmanArm::getLocation(PointId which)
{
  switch (which) {
  case STEERINGLINK:   return ChVector<>(0.129, 0, 0);
  case PITMANARM:      return ChVector<>(0.064, 0.249, 0);
  case REV:            return ChVector<>(0, 0.249, 0);
  case UNIV:           return ChVector<>(0.129, 0.249, 0);
  case REVSPH_R:       return ChVector<>(0, -0.325, 0);
  case REVSPH_S:       return ChVector<>(0.129, -0.325, 0);
  case TIEROD_PA:      return ChVector<>(0.195, 0.448, 0.035);
  case TIEROD_IA:      return ChVector<>(0.195, -0.448, 0.035);
  default:             return ChVector<>(0, 0, 0);
  }
}

const ChVector<> HMMWV_PitmanArm::getDirection(DirectionId which)
{
  switch (which) {
  case REV_AXIS:       return ChVector<>(0, 0, 1);
  case UNIV_AXIS_ARM:  return ChVector<>(0, 0, 1);
  case UNIV_AXIS_LINK: return ChVector<>(1, 0, 0);
  case REVSPH_AXIS:    return ChVector<>(0, 0, 1);
  default:             return ChVector<>(0, 0, 1);
  }
}


}  // end namespace chrono
