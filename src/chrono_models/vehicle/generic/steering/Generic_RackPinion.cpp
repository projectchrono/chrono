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
// Generic rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/generic/steering/Generic_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace generic {

const double Generic_RackPinion::m_steeringLinkMass = 9.072;
const ChVector<> Generic_RackPinion::m_steeringLinkInertia(1, 1, 1);
const double Generic_RackPinion::m_steeringLinkCOM = 0;
const double Generic_RackPinion::m_steeringLinkLength = 0.896;
const double Generic_RackPinion::m_steeringLinkRadius = 0.03;

const double Generic_RackPinion::m_pinionRadius = 0.1;

const double Generic_RackPinion::m_maxAngle = 30.0 * (CH_C_PI / 180);

Generic_RackPinion::Generic_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
