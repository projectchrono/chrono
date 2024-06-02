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
// Gator 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/gator/Gator_Driveline2WD.h"

namespace chrono {
namespace vehicle {
namespace gator {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Gator_Driveline2WD::m_driveshaft_inertia = 0.5;
const double Gator_Driveline2WD::m_differentialbox_inertia = 0.6;

const double Gator_Driveline2WD::m_conicalgear_ratio = 1.0;

const double Gator_Driveline2WD::m_axle_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
Gator_Driveline2WD::Gator_Driveline2WD(const std::string& name) : ChShaftsDriveline2WD(name) {
    SetMotorBlockDirection(ChVector3d(1, 0, 0));
    SetAxleDirection(ChVector3d(0, 1, 0));
}

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono
