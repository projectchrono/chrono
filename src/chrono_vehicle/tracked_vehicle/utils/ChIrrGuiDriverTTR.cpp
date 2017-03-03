// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono/core/ChMathematics.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChIrrGuiDriverTTR.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiDriverTTR::ChIrrGuiDriverTTR(ChVehicleIrrApp& app, double displacement_limit)
    : ChIrrGuiDriver(app),
      m_displacement(0),
      m_displacementDelta(displacement_limit / 50),
      m_minDisplacement(-displacement_limit),
      m_maxDisplacement(displacement_limit) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChIrrGuiDriverTTR::OnEvent(const SEvent& event) {
    // Allow the base class to first interpret events.
    if (ChIrrGuiDriver::OnEvent(event))
        return true;

    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_T:  // left post up
                SetDisplacement(m_displacement + m_displacementDelta);
                return true;
            case KEY_KEY_G:  // left post down
                SetDisplacement(m_displacement - m_displacementDelta);
                return true;
            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriverTTR::SetDisplacement(double vertical_disp) {
    m_displacement = ChClamp(vertical_disp, m_minDisplacement, m_maxDisplacement);
}

}  // end namespace vehicle
}  // end namespace chrono
