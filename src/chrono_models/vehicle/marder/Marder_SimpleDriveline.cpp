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
// Authors: Rainer Gericke
// =============================================================================
//
// A simplified Marder driveline.
//
// =============================================================================

#include "chrono_models/vehicle/marder/Marder_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_SimpleDriveline::m_diff_maxBias = 1;  //// 3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Marder_SimpleDriveline::Marder_SimpleDriveline() : ChSimpleTrackDriveline("Marder_SimpleDriveline") {}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
