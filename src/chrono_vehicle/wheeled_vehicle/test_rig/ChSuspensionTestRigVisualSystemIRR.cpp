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
// Irrlicht-based visualization for a suspension test rig.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigVisualSystemIRR.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigInteractiveDriver.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChSTRKeyboardHandlerIRR::ChSTRKeyboardHandlerIRR(ChSuspensionTestRigVisualSystemIRR* app) : m_app(app) {}

bool ChSTRKeyboardHandlerIRR::OnEvent(const SEvent& event) {
    if (!m_app->m_rig)
        return false;
    auto driver = std::dynamic_pointer_cast<ChSuspensionTestRigInteractiveDriver>(m_app->m_rig->GetDriver());
    if (!driver)
        return false;

    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_ADD:
            case KEY_PLUS:
                driver->NextAxle();
                return true;
            case KEY_SUBTRACT:
            case KEY_MINUS:
                driver->PreviousAxle();
                return true;
            case KEY_KEY_T:  // left post up
                driver->DecrementLeft();
                return true;
            case KEY_KEY_G:  // left post down
                driver->IncrementLeft();
                return true;
            case KEY_KEY_Y:  // right post up
                driver->DecrementRight();
                return true;
            case KEY_KEY_H:  // right post down
                driver->IncrementRight();
                return true;
            case KEY_KEY_A:
                driver->IncrementSteering();
                return true;
            case KEY_KEY_D:
                driver->DecrementSteering();
                return true;
            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------

ChSuspensionTestRigVisualSystemIRR::ChSuspensionTestRigVisualSystemIRR()
    : ChVisualSystemIrrlicht(), m_irr_initialized(false), m_rig(nullptr) {}

void ChSuspensionTestRigVisualSystemIRR::AttachSTR(ChSuspensionTestRig* rig) {
    m_rig = rig;
    if (m_systems.empty())
        AttachSystem(rig->GetVehicle().GetSystem());
}

void ChSuspensionTestRigVisualSystemIRR::Initialize() {
    if (!m_initialized) {
        SetCameraVertical(CameraVerticalDir::Z);

        ChVisualSystemIrrlicht::Initialize();
        
        AddCamera(ChVector3d(-1.5, 0, 0.5));
        AddLightDirectional();
        AddSkyBox();
        AddLogo();

        m_keyboard_handler = chrono_types::make_shared<ChSTRKeyboardHandlerIRR>(this);
        AddUserEventReceiver(m_keyboard_handler.get());
    }

    if (m_rig) {
        ChVector3d target =
            0.5 * (m_rig->GetSpindlePos(0, LEFT) + m_rig->GetSpindlePos(0, RIGHT)) + ChVector3d(0, 0, 0.5);
        ChVector3d position = target - ChVector3d(5, 0, 0);
        SetCameraPosition(position);
        SetCameraTarget(target);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
