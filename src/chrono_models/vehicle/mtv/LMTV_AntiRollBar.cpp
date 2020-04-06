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
// LMTV antirollbar RSD model. Unknown parameters!
//
// =============================================================================

#include "chrono_models/vehicle/mtv/LMTV_AntiRollBar.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double LMTV_AntirollBarRSD::m_arm_mass = 15.02;
const ChVector<> LMTV_AntirollBarRSD::m_arm_inertia(0.494431838, 0.010999093, 0.494431838);
const double LMTV_AntirollBarRSD::m_arm_length = 0.9 / 2.0;
const double LMTV_AntirollBarRSD::m_arm_width = -0.33;
const double LMTV_AntirollBarRSD::m_droplink_height = -0.17043;
const double LMTV_AntirollBarRSD::m_arm_radius = 0.02;
// const double LMTV_AntirollBarRSD::m_spring_coef = 130784.754;
// const double LMTV_AntirollBarRSD::m_damping_coef = 5000.0;
const double LMTV_AntirollBarRSD::m_spring_coef = 15 * 1307.84754;
const double LMTV_AntirollBarRSD::m_damping_coef = 15 * 50.000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LMTV_AntirollBarRSD::LMTV_AntirollBarRSD(const std::string& name) : ChAntirollBarRSD(name) {}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
