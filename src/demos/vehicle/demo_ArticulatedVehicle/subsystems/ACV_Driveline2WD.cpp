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

#include "subsystems/ACV_Driveline2WD.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double ACV_Driveline2WD::m_driveshaft_inertia = 0.5;
const double ACV_Driveline2WD::m_differentialbox_inertia = 0.6;

const double ACV_Driveline2WD::m_conicalgear_ratio = -0.2433;
const double ACV_Driveline2WD::m_differential_ratio = -1;

const double ACV_Driveline2WD::m_axle_differential_locking_limit = 100;

ACV_Driveline2WD::ACV_Driveline2WD(const std::string& name) : ChShaftsDriveline2WD(name) {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}
