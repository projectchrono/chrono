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
// MAN 5t simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/man/MAN_5t_SimpleDrivelineXWD.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double MAN_5t_SimpleDrivelineXWD::m_diff_bias = 2.0;

// -----------------------------------------------------------------------------
// Constructor of MAN_5t_SimpleDriveline.
// -----------------------------------------------------------------------------
MAN_5t_SimpleDrivelineXWD::MAN_5t_SimpleDrivelineXWD(const std::string& name) : ChSimpleDrivelineXWD(name) {}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
