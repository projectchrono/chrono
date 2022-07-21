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

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackTestRigVisualSystemIrrlicht.h"

using namespace irr;

namespace chrono {
namespace vehicle {

ChTrackTestRigVisualSystemIrrlicht::ChTrackTestRigVisualSystemIrrlicht()
    : ChVehicleVisualSystemIrrlicht(),
      m_rig(nullptr),
      m_render_frame_idler(false),
      m_render_frame_shoes(false),
      m_render_frame_sprocket(false),
      m_axis_shoes(1),
      m_axis_sprocket(1),
      m_axis_idler(1) {}

void ChTrackTestRigVisualSystemIrrlicht::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemIrrlicht::AttachVehicle(vehicle);
    m_rig = dynamic_cast<ChTrackTestRig*>(m_vehicle);
    assert(m_rig);
}

void ChTrackTestRigVisualSystemIrrlicht::RenderTrackShoeFrames(bool state, double axis_length) {
    m_render_frame_shoes = state;
    m_axis_shoes = axis_length;
}

void ChTrackTestRigVisualSystemIrrlicht::RenderSprocketFrame(bool state, double axis_length) {
    m_render_frame_sprocket = state;
    m_axis_sprocket = axis_length;
}

void ChTrackTestRigVisualSystemIrrlicht::RenderIdlerFrame(bool state, double axis_length) {
    m_render_frame_idler = state;
    m_axis_idler = axis_length;
}

// Render contact normals for monitored subsystems
void ChTrackTestRigVisualSystemIrrlicht::renderOtherGraphics() {
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
        ChVector<> v1 = c.m_point;
        if (normals) {
            ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis() * scale_normals;
            if (v1.y() > m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
        if (forces) {
            ChVector<> v2 = v1 + c.m_force * scale_forces;
            if (v1.y() > m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
    }

    // Contact normals on rear sprocket.
    // Note that we only render information for contacts on the outside gear profile
    for (const auto& c : m_rig->m_contact_manager->m_sprocket_R_contacts) {
        ChVector<> v1 = c.m_point;
        if (normals) {
            ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis() * scale_normals;
            if (v1.y() < m_rig->GetTrackAssembly()->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
        if (forces) {
            ChVector<> v2 = v1 + c.m_force * scale_forces;
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
void ChTrackTestRigVisualSystemIrrlicht::renderContacts(const std::list<ChTrackContactManager::ContactInfo>& lst,
                                                        const ChColor& col,
                                                        bool normals,
                                                        bool forces,
                                                        double scale_normals,
                                                        double scale_forces) {
    for (const auto& c : lst) {
        ChVector<> v1 = c.m_point;
        if (normals) {
            ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis() * scale_normals;
            irrlicht::tools::drawSegment(this, v1, v2, col, false);
        }
        if (forces) {
            ChVector<> v2 = v1 + c.m_force * scale_forces;
            irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.71f, 0.00f, 0.00f), false);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
