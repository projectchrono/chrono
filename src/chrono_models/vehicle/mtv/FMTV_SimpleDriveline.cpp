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
// FMTV simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/mtv/FMTV_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

const double FMTV_SimpleDriveline::m_front_torque_frac = 0.5;
const double FMTV_SimpleDriveline::m_front_diff_bias = 2.0;
const double FMTV_SimpleDriveline::m_rear_diff_bias = 2.0;

FMTV_SimpleDriveline::FMTV_SimpleDriveline(const std::string& name) : ChSimpleDriveline(name) {}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
