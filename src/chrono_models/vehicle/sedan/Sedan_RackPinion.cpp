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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Sedan rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/sedan/Sedan_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace sedan {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Sedan_RackPinion::m_steeringLinkMass = 1.889;
const ChVector3d Sedan_RackPinion::m_steeringLinkInertia(.138, 0.00009, .138);
const double Sedan_RackPinion::m_steeringLinkCOM = 0;
// const double Sedan_RackPinion::m_steeringLinkLength = 0.896;
const double Sedan_RackPinion::m_steeringLinkLength = 0.5;
const double Sedan_RackPinion::m_steeringLinkRadius = 0.03;

const double Sedan_RackPinion::m_pinionRadius = 0.03;

const double Sedan_RackPinion::m_maxAngle = 0.08 / 0.03;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Sedan_RackPinion::Sedan_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono
