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
// Generic 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/generic/driveline/Generic_Driveline2WD.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Generic_Driveline2WD::m_driveshaft_inertia = 0.5;
const double Generic_Driveline2WD::m_differentialbox_inertia = 0.6;

const double Generic_Driveline2WD::m_conicalgear_ratio = 0.2433;

const double Generic_Driveline2WD::m_axle_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the Generic_Driveline2WD.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
Generic_Driveline2WD::Generic_Driveline2WD(const std::string& name) : ChShaftsDriveline2WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
