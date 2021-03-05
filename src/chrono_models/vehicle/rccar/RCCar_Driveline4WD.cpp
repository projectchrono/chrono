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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// RCCar 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/rccar/RCCar_Driveline4WD.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace rccar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double RCCar_Driveline4WD::m_central_differentialbox_inertia = 0.06;
const double RCCar_Driveline4WD::m_front_differentialbox_inertia = 0.06;
const double RCCar_Driveline4WD::m_rear_differentialbox_inertia = 0.06;
const double RCCar_Driveline4WD::m_driveshaft_inertia = 0.05;
const double RCCar_Driveline4WD::m_frontshaft_inertia = 0.05;
const double RCCar_Driveline4WD::m_rearshaft_inertia = 0.05;

const double RCCar_Driveline4WD::m_front_conicalgear_ratio = 1;
const double RCCar_Driveline4WD::m_rear_conicalgear_ratio = 1;

const double RCCar_Driveline4WD::m_axle_differential_locking_limit = 100;
const double RCCar_Driveline4WD::m_central_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the RCCar_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
RCCar_Driveline4WD::RCCar_Driveline4WD(const std::string& name) : ChShaftsDriveline4WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // end namespace rccar
}  // namespace vehicle
}  // namespace chrono
