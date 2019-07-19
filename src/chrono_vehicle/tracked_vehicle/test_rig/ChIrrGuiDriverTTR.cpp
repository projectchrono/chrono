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
// Irrlicht-based GUI driver for the a track test rig. This class extends
// the ChIrrGuiDriver for a vehicle with controls for the shaker post.
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/tracked_vehicle/test_rig/ChIrrGuiDriverTTR.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiDriverTTR::ChIrrGuiDriverTTR(irrlicht::ChIrrApp& app)
    : m_app(app), m_current_post(0), m_msg("Active post: 0"), m_displDelta(1.0 / 250), m_throttleDelta(1.0 / 50) {
    app.SetUserEventReceiver(this);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChIrrGuiDriverTTR::OnEvent(const SEvent& event) {
    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_ADD:
            case KEY_PLUS:
                m_current_post = (m_current_post + 1) % m_displ.size();
                m_msg = "Active post: " + std::to_string(m_current_post);
                return true;
            case KEY_SUBTRACT:
            case KEY_MINUS:
                m_current_post = (m_current_post == 0) ? (int)m_displ.size() - 1 : m_current_post - 1;
                m_msg = "Active post: " + std::to_string(m_current_post);
                return true;
            case KEY_KEY_T:
                SetDisplacement(m_current_post, m_displ[m_current_post] + m_displDelta);
                return true;
            case KEY_KEY_G:
                SetDisplacement(m_current_post, m_displ[m_current_post] - m_displDelta);
                return true;
            case KEY_KEY_W:
                SetThrottle(m_throttle + m_throttleDelta);
                return true;
            case KEY_KEY_S:
                SetThrottle(m_throttle - m_throttleDelta);
                return true;
            default:
                break;
        }
    }

    return false;
}

}  // end namespace vehicle
}  // end namespace chrono
