// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Irrlicht-based visualization wrapper for track test rig.
// This class extends ChVehicleVisualSystemIrrlicht.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigVisualSystemIRR.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigInteractiveDriver.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChTTRKeyboardHandlerIRR::ChTTRKeyboardHandlerIRR(ChTrackTestRigVisualSystemIRR* app) : m_app(app) {}

bool ChTTRKeyboardHandlerIRR::OnEvent(const SEvent& event) {
    if (!m_app->m_rig)
        return false;
    auto driver = std::dynamic_pointer_cast<ChTrackTestRigInteractiveDriver>(m_app->m_rig->GetDriver());
    if (!driver)
        return false;

    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_ADD:
            case KEY_PLUS:
                driver->NextPost();
                return true;
            case KEY_SUBTRACT:
            case KEY_MINUS:
                driver->PreviousPost();
                return true;
            case KEY_KEY_T:
                driver->IncreasePost();
                return true;
            case KEY_KEY_G:
                driver->DecreasePost();
                return true;
            case KEY_KEY_W:
                driver->IncreaseThrottle();
                return true;
            case KEY_KEY_S:
                driver->DecreaseThrottle();
                return true;
            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------

ChTrackTestRigVisualSystemIRR::ChTrackTestRigVisualSystemIRR()
    : ChVisualSystemIrrlicht(),
      m_rig(nullptr),
      m_render_frame_idler(false),
      m_render_frame_shoes(false),
      m_render_frame_sprocket(false),
      m_axis_shoes(1),
      m_axis_sprocket(1),
      m_axis_idler(1) {}

void ChTrackTestRigVisualSystemIRR::AttachTTR(ChTrackTestRig* rig) {
    m_rig = rig;
    AttachSystem(m_rig->GetSystem());
}

void ChTrackTestRigVisualSystemIRR::Initialize() {
    if (!m_initialized) {
        SetCameraVertical(CameraVerticalDir::Z);

        ChVisualSystemIrrlicht::Initialize();

        AddCamera(ChVector3d(0, -3, 0.5));
        AddLightDirectional();
        AddSkyBox();
        AddLogo();

        m_keyboard_handler = chrono_types::make_shared<ChTTRKeyboardHandlerIRR>(this);
        AddUserEventReceiver(m_keyboard_handler.get());
    }

    if (m_rig) {
        auto sprocket_pos = m_rig->m_track->GetSprocketLocation();
        auto idler_pos = m_rig->m_track->GetIdlerLocation();
        ChVector3d target = 0.5 * (sprocket_pos + idler_pos) + ChVector3d(0, 0, 0.5);
        ChVector3d position = target - ChVector3d(0, 3, 0);
        SetCameraPosition(position);
        SetCameraTarget(target);
    }
}

void ChTrackTestRigVisualSystemIRR::Render() {
    ChVisualSystemIrrlicht::Render();

    renderOtherGraphics();
}

void ChTrackTestRigVisualSystemIRR::RenderTrackShoeFrames(bool state, double axis_length) {
    m_render_frame_shoes = state;
    m_axis_shoes = axis_length;
}

void ChTrackTestRigVisualSystemIRR::RenderSprocketFrame(bool state, double axis_length) {
    m_render_frame_sprocket = state;
    m_axis_sprocket = axis_length;
}

void ChTrackTestRigVisualSystemIRR::RenderIdlerFrame(bool state, double axis_length) {
    m_render_frame_idler = state;
    m_axis_idler = axis_length;
}

// Render contact normals for monitored subsystems
void ChTrackTestRigVisualSystemIRR::renderOtherGraphics() {
    bool normals = m_rig->m_contact_manager->m_render_normals;
    bool forces = m_rig->m_contact_manager->m_render_forces;
    double scale_normals = 0.4;
    double scale_forces = m_rig->m_contact_manager->m_scale_forces;

    // Track shoe frames
    if (m_render_frame_shoes) {
        for (size_t i = 0; i < m_rig->GetTrackAssembly()->GetNumTrackShoes(); i++) {
            RenderFrame(*m_rig->GetTrackAssembly()->GetTrackShoe(i)->GetShoeBody(), m_axis_shoes);
        }
    }

    // Sprocket frame
    if (m_render_frame_sprocket) {
        RenderFrame(*m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody(), m_axis_sprocket);
    }

    // Idler frame
    if (m_render_frame_idler) {
        RenderFrame(*m_rig->GetTrackAssembly()->GetIdler()->GetWheelBody(), m_axis_idler);
    }

    // Contact normals on left sprocket.
    // Note that we only render information for contacts on the outside gear profile
    for (const auto& c : m_rig->m_contact_manager->m_sprocket_L_contacts) {
        ChVector3d v1 = c.m_point;
        if (normals) {
            ChVector3d v2 = v1 + c.m_csys.GetAxisX() * scale_normals;
            if (v1.y() > m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
        if (forces) {
            ChVector3d v2 = v1 + c.m_force * scale_forces;
            if (v1.y() > m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
    }

    // Contact normals on rear sprocket.
    // Note that we only render information for contacts on the outside gear profile
    for (const auto& c : m_rig->m_contact_manager->m_sprocket_R_contacts) {
        ChVector3d v1 = c.m_point;
        if (normals) {
            ChVector3d v2 = v1 + c.m_csys.GetAxisX() * scale_normals;
            if (v1.y() < m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
        if (forces) {
            ChVector3d v2 = v1 + c.m_force * scale_forces;
            if (v1.y() > m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
    }

    // Contact normals on monitored track shoes.
    renderContacts(m_rig->m_contact_manager->m_shoe_L_contacts, ChColor(0.31f, 0.31f, 0.00f), normals, forces,
                   scale_normals, scale_forces);
    renderContacts(m_rig->m_contact_manager->m_shoe_R_contacts, ChColor(0.31f, 0.31f, 0.00f), normals, forces,
                   scale_normals, scale_forces);

    // Contact normals on idler wheels.
    renderContacts(m_rig->m_contact_manager->m_idler_L_contacts, ChColor(0.00f, 0.00f, 0.31f), normals, forces,
                   scale_normals, scale_forces);
    renderContacts(m_rig->m_contact_manager->m_idler_R_contacts, ChColor(0.00f, 0.00f, 0.31f), normals, forces,
                   scale_normals, scale_forces);
}

// Render normal for all contacts in the specified list, using the given color.
void ChTrackTestRigVisualSystemIRR::renderContacts(const std::list<ChTrackContactManager::ContactInfo>& lst,
                                                        const ChColor& col,
                                                        bool normals,
                                                        bool forces,
                                                        double scale_normals,
                                                        double scale_forces) {
    for (const auto& c : lst) {
        ChVector3d v1 = c.m_point;
        if (normals) {
            ChVector3d v2 = v1 + c.m_csys.GetAxisX() * scale_normals;
            irrlicht::tools::drawSegment(this, v1, v2, col, false);
        }
        if (forces) {
            ChVector3d v2 = v1 + c.m_force * scale_forces;
            irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.71f, 0.00f, 0.00f), false);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
