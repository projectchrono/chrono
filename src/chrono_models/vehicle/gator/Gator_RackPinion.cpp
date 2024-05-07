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
// Gator rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Gator_RackPinion::m_steeringLinkMass = 2;
const ChVector3d Gator_RackPinion::m_steeringLinkInertia(0.138, 0.05, 0.138);
const double Gator_RackPinion::m_steeringLinkCOM = 0;
const double Gator_RackPinion::m_steeringLinkLength = 0.5;
const double Gator_RackPinion::m_steeringLinkRadius = 0.03;

const double Gator_RackPinion::m_pinionRadius = 0.035;

const double Gator_RackPinion::m_maxAngle = 1;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Gator_RackPinion::Gator_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
