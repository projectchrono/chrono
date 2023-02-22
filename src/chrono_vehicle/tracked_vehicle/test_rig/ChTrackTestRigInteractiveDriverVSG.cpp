// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// VSG-based GUI driver for the a track test rig.
// This class implements the functionality required by its base class using
// keyboard inputs.
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriverVSG.h"

namespace chrono {
namespace vehicle {

//// TODO

ChTrackTestRigInteractiveDriverVSG::ChTrackTestRigInteractiveDriverVSG(vsg3d::ChVisualSystemVSG& vsys)
    : m_current_post(0), m_msg("Active post: 0"), m_displDelta(1.0 / 250), m_throttleDelta(1.0 / 50) {
}


}  // end namespace vehicle
}  // end namespace chrono
