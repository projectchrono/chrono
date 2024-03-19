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
// Authors: Alessandro Tasora, Radu Serban, Asher Elmquist, Rainer Gericke
// data from http://www.treffseiten.de/bmw/info/daten_320i_325i_330i_320d_limousine_05_09.pdf
// Conical gear ratio from automatic gearbox version
// =============================================================================
//
// BMW E90 (330i 2006) 2WD driveline model based on ChShaft objects.
//
// =============================================================================

#include "chrono_models/vehicle/bmw/BMW_E90_Driveline.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double BMW_E90_Driveline::m_driveshaft_inertia = 0.5;
const double BMW_E90_Driveline::m_differentialbox_inertia = 0.6;

const double BMW_E90_Driveline::m_conicalgear_ratio = 1.0 / 3.64;  // Vmax = 93.3 m/s (330 km/h)
// In Germany Vmax is limited to 250 km/h
// const double BMW_E90_Driveline::m_conicalgear_ratio = 0.203;  Vmax = 69.03 m/s (250 km/h)

const double BMW_E90_Driveline::m_axle_differential_locking_limit = 100;

// -----------------------------------------------------------------------------
// Constructor of the BMW_E90_Driveline.
// the direction of the motor block is along the X axis, while the directions of
// the axles is along the Y axis (relative to the chassis coordinate frame),
// -----------------------------------------------------------------------------
BMW_E90_Driveline::BMW_E90_Driveline(const std::string& name) : ChShaftsDriveline2WD(name) {
    SetMotorBlockDirection(ChVector3d(1, 0, 0));
    SetAxleDirection(ChVector3d(0, 1, 0));
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
