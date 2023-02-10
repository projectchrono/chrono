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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// M113 driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/m113/M113_DrivelineBDS.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_DrivelineBDS::m_driveshaft_inertia = 0.5;
const double M113_DrivelineBDS::m_differentialbox_inertia = 0.6;

const double M113_DrivelineBDS::m_conicalgear_ratio = 0.5;

const double M113_DrivelineBDS::m_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the M113_DrivelineBDS.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
M113_DrivelineBDS::M113_DrivelineBDS() : ChTrackDrivelineBDS("M113_DrivelineBDS") {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
