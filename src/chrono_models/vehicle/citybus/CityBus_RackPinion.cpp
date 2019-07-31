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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl
// =============================================================================
//
// CityBus rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double CityBus_RackPinion::m_steeringLinkMass = 3.0;
const ChVector<> CityBus_RackPinion::m_steeringLinkInertia(.138, 0.09, .138);
const double CityBus_RackPinion::m_steeringLinkCOM = 0;
//const double CityBus_RackPinion::m_steeringLinkLength = 0.896;
const double CityBus_RackPinion::m_steeringLinkLength = 0.5;
const double CityBus_RackPinion::m_steeringLinkRadius = 0.03;

const double CityBus_RackPinion::m_pinionRadius = -0.03;

//const double CityBus_RackPinion::m_maxAngle = 0.08 / 0.03;
const double CityBus_RackPinion::m_maxAngle = 5.5;



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_RackPinion::CityBus_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
