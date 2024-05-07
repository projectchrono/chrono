// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Jeep Cherokee 4WD driveline model based on ChShaft objects.
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "Cherokee_Driveline4WD.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Cherokee_Driveline4WD::m_central_differentialbox_inertia = 0.6;
const double Cherokee_Driveline4WD::m_front_differentialbox_inertia = 0.6;
const double Cherokee_Driveline4WD::m_rear_differentialbox_inertia = 0.6;
const double Cherokee_Driveline4WD::m_driveshaft_inertia = 0.5;
const double Cherokee_Driveline4WD::m_frontshaft_inertia = 0.5;
const double Cherokee_Driveline4WD::m_rearshaft_inertia = 0.5;

const double Cherokee_Driveline4WD::m_front_conicalgear_ratio = 0.2;
const double Cherokee_Driveline4WD::m_rear_conicalgear_ratio = 0.2;

const double Cherokee_Driveline4WD::m_axle_differential_locking_limit = 100;
const double Cherokee_Driveline4WD::m_central_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the Cherokee_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
Cherokee_Driveline4WD::Cherokee_Driveline4WD(const std::string& name) : ChShaftsDriveline4WD(name) {
    SetMotorBlockDirection(ChVector3d(1, 0, 0));
    SetAxleDirection(ChVector3d(0, 1, 0));
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
