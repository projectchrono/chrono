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
// Authors: Radu Serban
// =============================================================================
//
// HMMWV 4WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "hmmwv/driveline/HMMWV_Driveline4WD.h"

using namespace chrono;

namespace hmmwv {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double  HMMWV_Driveline4WD::m_central_differentialbox_inertia = 0.6;
const double  HMMWV_Driveline4WD::m_front_differentialbox_inertia = 0.6;
const double  HMMWV_Driveline4WD::m_rear_differentialbox_inertia = 0.6;
const double  HMMWV_Driveline4WD::m_driveshaft_inertia = 0.5;
const double  HMMWV_Driveline4WD::m_frontshaft_inertia = 0.5;
const double  HMMWV_Driveline4WD::m_rearshaft_inertia = 0.5;

const double  HMMWV_Driveline4WD::m_central_differential_ratio = -1;
const double  HMMWV_Driveline4WD::m_front_differential_ratio = -1;
const double  HMMWV_Driveline4WD::m_rear_differential_ratio = -1;
const double  HMMWV_Driveline4WD::m_front_conicalgear_ratio = -0.2;
const double  HMMWV_Driveline4WD::m_rear_conicalgear_ratio = -0.2;


// -----------------------------------------------------------------------------
// Constructor of the HMMWV_Driveline4WD.
// The direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
HMMWV_Driveline4WD::HMMWV_Driveline4WD()
: ChShaftsDriveline4WD()
{
  SetMotorBlockDirection(ChVector<>(1, 0, 0));
  SetAxleDirection(ChVector<>(0, 1, 0));
}


} // end namespace hmmwv
