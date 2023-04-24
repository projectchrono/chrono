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
// mrole simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/mrole/driveline/mrole_SimpleDrivelineXWD.h"

namespace chrono {
namespace vehicle {
namespace mrole {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double mrole_SimpleDrivelineXWD::m_diff_bias = 2.0;

// -----------------------------------------------------------------------------
// Constructor of mrole_SimpleDriveline.
// -----------------------------------------------------------------------------
mrole_SimpleDrivelineXWD::mrole_SimpleDrivelineXWD(const std::string& name) : ChSimpleDrivelineXWD(name) {}

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono
