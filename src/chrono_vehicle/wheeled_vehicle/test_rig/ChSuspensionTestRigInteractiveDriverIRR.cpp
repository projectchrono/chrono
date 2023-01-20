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
// Irrlicht-based GUI driver for the a suspension test rig.
// This class implements the functionality required by its base ChSuspensionTestRigDriver
// class using keyboard inputs.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current inputs.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriverIRR.h"

using namespace irr;

namespace chrono {
namespace vehicle {

ChSuspensionTestRigInteractiveDriverIRR::ChSuspensionTestRigInteractiveDriverIRR(irrlicht::ChVisualSystemIrrlicht& vsys)
    : m_current_post(0), m_msg("Active post: 0"), m_vsys(vsys), m_displDelta(1.0 / 50), m_steeringDelta(1.0 / 250) {
    vsys.AddUserEventReceiver(this);
}

bool ChSuspensionTestRigInteractiveDriverIRR::OnEvent(const SEvent& event) {
    if (m_vsys.GetSystem(0).GetChTime() < m_delay)
        return false;

    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_ADD:
            case KEY_PLUS:
                m_current_post = (m_current_post + 1) % m_naxles;
                m_msg = "Active post: " + std::to_string(m_current_post);
                return true;
            case KEY_SUBTRACT:
            case KEY_MINUS:
                m_current_post = (m_current_post == 0) ? m_naxles - 1 : m_current_post - 1;
                m_msg = "Active post: " + std::to_string(m_current_post);
                return true;
            case KEY_KEY_T:  // left post up
                SetDisplacementLeft(m_current_post, m_displLeft[m_current_post] + m_displDelta);
                return true;
            case KEY_KEY_G:  // left post down
                SetDisplacementLeft(m_current_post, m_displLeft[m_current_post] - m_displDelta);
                return true;
            case KEY_KEY_Y:  // right post up
                SetDisplacementRight(m_current_post, m_displRight[m_current_post] + m_displDelta);
                return true;
            case KEY_KEY_H:  // right post down
                SetDisplacementRight(m_current_post, m_displRight[m_current_post] - m_displDelta);
                return true;
            case KEY_KEY_A:
                SetSteering(m_steering + m_steeringDelta);
                return true;
            case KEY_KEY_D:
                SetSteering(m_steering - m_steeringDelta);
                return true;
            default:
                break;
        }
    }

    return false;
}

}  // end namespace vehicle
}  // end namespace chrono
