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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// BME E90 antirollbar RSD model.
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// =============================================================================

#include "chrono_models/vehicle/bmw/BMW_E90_AntiRollBarFront.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double BMW_E90_AntiRollBarFront::m_arm_mass = 5.0;
const ChVector3d BMW_E90_AntiRollBarFront::m_arm_inertia(0.494431838, 0.010999093, 0.494431838);
const double BMW_E90_AntiRollBarFront::m_arm_length = 1.0 / 2.0;
const double BMW_E90_AntiRollBarFront::m_arm_width = 0.33;
const double BMW_E90_AntiRollBarFront::m_droplink_height = -0.17043;
const double BMW_E90_AntiRollBarFront::m_arm_radius = 0.01;

const double BMW_E90_AntiRollBarFront::m_spring_coef = 13000.0*0.69; // front/rear 69%/31%;
const double BMW_E90_AntiRollBarFront::m_damping_coef = 0.05 * BMW_E90_AntiRollBarFront::m_spring_coef;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_AntiRollBarFront::BMW_E90_AntiRollBarFront(const std::string& name) : ChAntirollBarRSD(name) {}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
