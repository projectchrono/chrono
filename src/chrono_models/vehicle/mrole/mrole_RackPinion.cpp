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
// mrole rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/mrole_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double max_steer_deg = 120;
const double mrole_RackPinion1::m_steeringLinkMass = 9.072;
const ChVector<> mrole_RackPinion1::m_steeringLinkInertia(1, 1, 1);
const double mrole_RackPinion1::m_steeringLinkCOM = 0;
const double mrole_RackPinion1::m_steeringLinkLength = 1.0;
const double mrole_RackPinion1::m_steeringLinkRadius = 0.03;

const double mrole_RackPinion1::m_pinionRadius = 0.1;

const double mrole_RackPinion1::m_maxAngle = max_steer_deg * (CH_C_PI / 180);

const double mrole_RackPinion2::m_steeringLinkMass = 9.072;
const ChVector<> mrole_RackPinion2::m_steeringLinkInertia(1, 1, 1);
const double mrole_RackPinion2::m_steeringLinkCOM = 0;
const double mrole_RackPinion2::m_steeringLinkLength = 1.0;
const double mrole_RackPinion2::m_steeringLinkRadius = 0.03;

const double mrole_RackPinion2::m_pinionRadius = 0.1;

const double mrole_RackPinion2::m_maxAngle = max_steer_deg * (CH_C_PI / 180) / 1.50436005;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
mrole_RackPinion1::mrole_RackPinion1(const std::string& name) : ChRackPinion(name) {}

mrole_RackPinion2::mrole_RackPinion2(const std::string& name) : ChRackPinion(name) {}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
