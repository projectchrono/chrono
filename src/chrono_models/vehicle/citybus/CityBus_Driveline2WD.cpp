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
// Authors: Alessandro Tasora, Radu Serban, Asher Elmquist, Evan Hoerl
// =============================================================================
//
// CityBus 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus_Driveline2WD.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double CityBus_Driveline2WD::m_driveshaft_inertia = 1.5;
const double CityBus_Driveline2WD::m_differentialbox_inertia = 1.6;

const double CityBus_Driveline2WD::m_conicalgear_ratio = -0.2;
const double CityBus_Driveline2WD::m_differential_ratio = -1;

const double CityBus_Driveline2WD::m_axle_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the CityBus_Driveline2WD.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
CityBus_Driveline2WD::CityBus_Driveline2WD(const std::string& name) : ChShaftsDriveline2WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
