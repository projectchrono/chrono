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
// mrole 8WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/driveline/mrole_Driveline8WD.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double mrole_Driveline8WD::m_driveshaft_inertia = 0.5;
const double mrole_Driveline8WD::m_group_inshaft_inertia = 0.5;
const double mrole_Driveline8WD::m_axle_inshaft_inertia = 0.5;
const double mrole_Driveline8WD::m_axle_diffbox_inertia = 0.6;

const double mrole_Driveline8WD::m_axle_conicalgear_ratio = 0.1;

const double mrole_Driveline8WD::m_central_differential_locking_limit = 5000;
const double mrole_Driveline8WD::m_group_differential_locking_limit = 5000;
const double mrole_Driveline8WD::m_axle_differential_locking_limit = 3000;

// -----------------------------------------------------------------------------
// Constructor of the mrole_Driveline8WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
mrole_Driveline8WD::mrole_Driveline8WD(const std::string& name) : ChShaftsDriveline8WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
