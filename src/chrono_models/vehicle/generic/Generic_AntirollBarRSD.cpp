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
// Generic antirollbar RSD model.
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_AntirollBarRSD.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Generic_AntirollBarRSD::m_arm_mass = 1.0;
const ChVector<> Generic_AntirollBarRSD::m_arm_inertia(1.0, 1.0, 1.0);
const double Generic_AntirollBarRSD::m_arm_length = 0.7;
const double Generic_AntirollBarRSD::m_arm_width = 0.25;
const double Generic_AntirollBarRSD::m_droplink_height = -0.2;
const double Generic_AntirollBarRSD::m_arm_radius = 0.02;
const double Generic_AntirollBarRSD::m_spring_coef = 100000.0;
const double Generic_AntirollBarRSD::m_damping_coef = 20000.0;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_AntirollBarRSD::Generic_AntirollBarRSD(const std::string& name) : ChAntirollBarRSD(name) {}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
