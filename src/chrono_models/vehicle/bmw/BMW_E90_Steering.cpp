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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// BMW E90 rack-pinion steering model.
// Vehicle Parameters taken from SAE Paper 2007-01-0818
// Steering trapez behind origin (original in front of origin)
//
// =============================================================================

#include "chrono_models/vehicle/bmw/BMW_E90_Steering.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double BMW_E90_Steering::m_steeringLinkMass = 9.072;
const ChVector3d BMW_E90_Steering::m_steeringLinkInertia(1, 1, 1);
const double BMW_E90_Steering::m_steeringLinkCOM = 0;
const double BMW_E90_Steering::m_steeringLinkLength = 0.4;
const double BMW_E90_Steering::m_steeringLinkRadius = 0.015;

const double BMW_E90_Steering::m_pinionRadius = 0.1;

const double BMW_E90_Steering::m_maxAngle = 64 * CH_DEG_TO_RAD;  // ~ 11.0 m turn radius RWD

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_Steering::BMW_E90_Steering(const std::string& name) : ChRackPinion(name) {}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
