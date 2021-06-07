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
// FEDA 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_Driveline4WD.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double FEDA_Driveline4WD::m_central_differentialbox_inertia = 0.6;
const double FEDA_Driveline4WD::m_front_differentialbox_inertia = 0.6;
const double FEDA_Driveline4WD::m_rear_differentialbox_inertia = 0.6;
const double FEDA_Driveline4WD::m_driveshaft_inertia = 0.5;
const double FEDA_Driveline4WD::m_frontshaft_inertia = 0.5;
const double FEDA_Driveline4WD::m_rearshaft_inertia = 0.5;

const double FEDA_Driveline4WD::m_front_conicalgear_ratio = 1 / 4.88;
const double FEDA_Driveline4WD::m_rear_conicalgear_ratio = 1 / 4.88;

const double FEDA_Driveline4WD::m_axle_differential_locking_limit = 100;
const double FEDA_Driveline4WD::m_central_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the FEDA_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
FEDA_Driveline4WD::FEDA_Driveline4WD(const std::string& name) : ChShaftsDriveline4WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
