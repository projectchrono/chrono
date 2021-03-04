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
// MAN 5t 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_5t_Driveline4WD.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double MAN_5t_Driveline4WD::m_central_differentialbox_inertia = 0.6;
const double MAN_5t_Driveline4WD::m_front_differentialbox_inertia = 0.6;
const double MAN_5t_Driveline4WD::m_rear_differentialbox_inertia = 0.6;
const double MAN_5t_Driveline4WD::m_driveshaft_inertia = 0.5;
const double MAN_5t_Driveline4WD::m_frontshaft_inertia = 0.5;
const double MAN_5t_Driveline4WD::m_rearshaft_inertia = 0.5;

const double MAN_5t_Driveline4WD::m_front_conicalgear_ratio = 1 / 3.947 / 1.706 / 1.02;
const double MAN_5t_Driveline4WD::m_rear_conicalgear_ratio = 1 / 3.947 / 1.706 / 1.02;

const double MAN_5t_Driveline4WD::m_axle_differential_locking_limit = 100;
const double MAN_5t_Driveline4WD::m_central_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the MAN_5t_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
MAN_5t_Driveline4WD::MAN_5t_Driveline4WD(const std::string& name) : ChShaftsDriveline4WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
