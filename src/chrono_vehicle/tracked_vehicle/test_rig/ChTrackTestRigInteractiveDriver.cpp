// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Interactive driver for a track test rig.
// Independent of keyboard event handler.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriver.h"

namespace chrono {
namespace vehicle {

ChTrackTestRigInteractiveDriver::ChTrackTestRigInteractiveDriver()
    : m_current_post(0), m_msg("Active post: 0"), m_displ_delta(1.0 / 250), m_throttle_delta(1.0 / 50) {}


double ChTrackTestRigInteractiveDriver::GetPost() {
    return m_displ[m_current_post];
}

void ChTrackTestRigInteractiveDriver::NextPost() {
    m_current_post = (m_current_post + 1) % m_nposts;
}

void ChTrackTestRigInteractiveDriver::PreviousPost() {
    m_current_post = (m_current_post == 0) ? m_nposts - 1 : m_current_post - 1;
}

void ChTrackTestRigInteractiveDriver::IncreasePost() {
    SetDisplacement(m_current_post, m_displ[m_current_post] + m_displ_delta);
}

void ChTrackTestRigInteractiveDriver::DecreasePost() {
    SetDisplacement(m_current_post, m_displ[m_current_post] - m_displ_delta);
}

void ChTrackTestRigInteractiveDriver::IncreaseThrottle() {
    SetThrottle(m_throttle + m_throttle_delta);
}

void ChTrackTestRigInteractiveDriver::DecreaseThrottle() {
    SetThrottle(m_throttle - m_throttle_delta);
}

}  // end namespace vehicle
}  // end namespace chrono
