// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// HMMWV rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/hmmwv/steering/HMMWV_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_RackPinion::m_steeringLinkMass = 9.072;
const ChVector<> HMMWV_RackPinion::m_steeringLinkInertia(1, 1, 1);
const double HMMWV_RackPinion::m_steeringLinkCOM = 0;
const double HMMWV_RackPinion::m_steeringLinkLength = 0.896;
const double HMMWV_RackPinion::m_steeringLinkRadius = 0.03;

const double HMMWV_RackPinion::m_pinionRadius = 0.1;

const double HMMWV_RackPinion::m_maxAngle = 30.0 * (CH_C_PI / 180);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_RackPinion::HMMWV_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
