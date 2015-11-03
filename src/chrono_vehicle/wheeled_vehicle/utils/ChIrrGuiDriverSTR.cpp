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
// Irrlicht-based GUI driver for the a suspension test rig. This class extends
// the ChIrrGuiDriver for a vehicle with controls for the shaker posts.
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChMathematics.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChIrrGuiDriverSTR.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiDriverSTR::ChIrrGuiDriverSTR(ChVehicleIrrApp& app, double displacement_limit)
    : ChIrrGuiDriver(app),
      m_displacementLeft(0),
      m_displacementRight(0),
      m_displacementDelta(displacement_limit / 50),
      m_minDisplacement(-displacement_limit),
      m_maxDisplacement(displacement_limit) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChIrrGuiDriverSTR::OnEvent(const SEvent& event) {
    // Allow the base class to first interpret events.
    if (ChIrrGuiDriver::OnEvent(event))
        return true;

    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_T:  // left post up
                SetDisplacementLeft(m_displacementLeft + m_displacementDelta);
                return true;
            case KEY_KEY_G:  // left post down
                SetDisplacementLeft(m_displacementLeft - m_displacementDelta);
                return true;
            case KEY_KEY_Y:  // right post up
                SetDisplacementRight(m_displacementRight + m_displacementDelta);
                return true;
            case KEY_KEY_H:  // right post down
                SetDisplacementRight(m_displacementRight - m_displacementDelta);
                return true;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriverSTR::SetDisplacementLeft(double vertical_disp) {
    m_displacementLeft = ChClamp(vertical_disp, m_minDisplacement, m_maxDisplacement);
}

void ChIrrGuiDriverSTR::SetDisplacementRight(double vertical_disp) {
    m_displacementRight = ChClamp(vertical_disp, m_minDisplacement, m_maxDisplacement);
}

}  // end namespace vehicle
}  // end namespace chrono
