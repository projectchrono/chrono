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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// HMMWV 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "models/vehicle/hmmwv/HMMWV_Driveline2WD.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_Driveline2WD::m_driveshaft_inertia = 0.5;
const double HMMWV_Driveline2WD::m_differentialbox_inertia = 0.6;

const double HMMWV_Driveline2WD::m_conicalgear_ratio = -0.2;
const double HMMWV_Driveline2WD::m_differential_ratio = -1;

// -----------------------------------------------------------------------------
// Constructor of the HMMWV_Driveline2WD.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
HMMWV_Driveline2WD::HMMWV_Driveline2WD() : ChShaftsDriveline2WD() {
    SetMotorBlockDirection(ChVector<>(1, 0, 0));
    SetAxleDirection(ChVector<>(0, 1, 0));
}

}  // end namespace hmmwv
