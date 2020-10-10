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
// Kraz 64431 28t 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_tractor_Driveline.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Kraz_tractor_Driveline::m_central_differentialbox_inertia = 0.6;
const double Kraz_tractor_Driveline::m_front_differentialbox_inertia = 0.6;
const double Kraz_tractor_Driveline::m_rear_differentialbox_inertia = 0.6;
const double Kraz_tractor_Driveline::m_driveshaft_inertia = 0.5;
const double Kraz_tractor_Driveline::m_frontshaft_inertia = 0.5;
const double Kraz_tractor_Driveline::m_rearshaft_inertia = 0.5;

const double Kraz_tractor_Driveline::m_central_differential_ratio = -1;
const double Kraz_tractor_Driveline::m_front_differential_ratio = -1;
const double Kraz_tractor_Driveline::m_rear_differential_ratio = -1;
const double Kraz_tractor_Driveline::m_front_conicalgear_ratio = -1.0 / 6.154;
const double Kraz_tractor_Driveline::m_rear_conicalgear_ratio = -1.0 / 6.154;

const double Kraz_tractor_Driveline::m_axle_differential_locking_limit = 100;
const double Kraz_tractor_Driveline::m_central_differential_locking_limit = 10000;

// -----------------------------------------------------------------------------
// Constructor of the SemiTractor_driveline.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
Kraz_tractor_Driveline::Kraz_tractor_Driveline(const std::string& name) : ChShaftsDriveline4WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
