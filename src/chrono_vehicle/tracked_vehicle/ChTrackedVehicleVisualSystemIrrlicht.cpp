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
// Irrlicht-based visualization for tracked vehicles.
// This class extends ChVehicleVisualSystemIrrlicht.
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackedVehicleVisualSystemIrrlicht::ChTrackedVehicleVisualSystemIrrlicht()
    : ChVehicleVisualSystemIrrlicht(),
      m_tvehicle(nullptr),
      m_render_frame_idlers{false, false},
      m_render_frame_shoes{false, false},
      m_render_frame_sprockets{false, false} {}

void ChTrackedVehicleVisualSystemIrrlicht::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemIrrlicht::AttachVehicle(vehicle);
    m_tvehicle = dynamic_cast<ChTrackedVehicle*>(m_vehicle);
    assert(m_tvehicle);
}

void ChTrackedVehicleVisualSystemIrrlicht::RenderTrackShoeFrames(VehicleSide side, bool state, double axis_length) {
    m_render_frame_shoes[side] = state;
    m_axis_shoes[side] = axis_length;
}

void ChTrackedVehicleVisualSystemIrrlicht::RenderSprocketFrame(VehicleSide side, bool state, double axis_length) {
    m_render_frame_sprockets[side] = state;
    m_axis_sprockets[side] = axis_length;
}

void ChTrackedVehicleVisualSystemIrrlicht::RenderIdlerFrame(VehicleSide side, bool state, double axis_length) {
    m_render_frame_idlers[side] = state;
    m_axis_idlers[side] = axis_length;
}

// -----------------------------------------------------------------------------
// Render stats for the vehicle driveline subsystem.
// -----------------------------------------------------------------------------
void ChTrackedVehicleVisualSystemIrrlicht::renderOtherStats(int left, int top) {
    char msg[100];

    auto driveline = m_tvehicle->GetDriveline();
    double toRPM = 30 / CH_C_PI;

    double shaft_speed = driveline->GetOutputDriveshaftSpeed() * toRPM;
    snprintf(msg, sizeof(msg), "Driveshaft (RPM): %+.2f", shaft_speed);
    renderLinGauge(std::string(msg), shaft_speed / 2000, true, left, top, 170, 15);

    double speedL = driveline->GetSprocketSpeed(LEFT) * toRPM;
    snprintf(msg, sizeof(msg), "L (RPM): %+.2f", speedL);
    renderLinGauge(std::string(msg), speedL / 2000, true, left, top + 30, 170, 15);

    double torqueL = driveline->GetSprocketTorque(LEFT);
    snprintf(msg, sizeof(msg), "T.L (Nm): %+.2f", torqueL);
    renderLinGauge(std::string(msg), torqueL / 5000, true, left, top + 50, 170, 15);

    double speedR = driveline->GetSprocketSpeed(RIGHT) * toRPM;
    snprintf(msg, sizeof(msg), "R (RPM): %+.2f", speedR);
    renderLinGauge(std::string(msg), speedR / 2000, true, left + 190, top + 30, 170, 15);

    double torqueR = driveline->GetSprocketTorque(RIGHT);
    snprintf(msg, sizeof(msg), "T.R (Nm): %+.2f", torqueR);
    renderLinGauge(std::string(msg), torqueR / 5000, true, left + 190, top + 50, 170, 15);
}

// -----------------------------------------------------------------------------
// Render contact normals for monitored subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicleVisualSystemIrrlicht::renderOtherGraphics() {
    bool normals = m_tvehicle->m_contact_manager->m_render_normals;
    bool forces = m_tvehicle->m_contact_manager->m_render_forces;
    double scale_normals = 0.4;
    double scale_forces = m_tvehicle->m_contact_manager->m_scale_forces;

    // Track shoe frames
    if (m_render_frame_shoes[LEFT]) {
        for (size_t i = 0; i < m_tvehicle->GetTrackAssembly(LEFT)->GetNumTrackShoes(); i++) {
            RenderFrame(*m_tvehicle->GetTrackAssembly(LEFT)->GetTrackShoe(i)->GetShoeBody(), m_axis_shoes[LEFT]);
        }
    }
    if (m_render_frame_shoes[RIGHT]) {
        for (size_t i = 0; i < m_tvehicle->GetTrackAssembly(RIGHT)->GetNumTrackShoes(); i++) {
            RenderFrame(*m_tvehicle->GetTrackAssembly(RIGHT)->GetTrackShoe(i)->GetShoeBody(), m_axis_shoes[RIGHT]);
        }
    }

    // Sprocket frames
    if (m_render_frame_sprockets[LEFT]) {
        RenderFrame(*m_tvehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody(), m_axis_sprockets[LEFT]);
    }
    if (m_render_frame_sprockets[RIGHT]) {
        RenderFrame(*m_tvehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody(), m_axis_sprockets[RIGHT]);
    }

    // Idler frames
    if (m_render_frame_idlers[LEFT]) {
        RenderFrame(*m_tvehicle->GetTrackAssembly(LEFT)->GetIdler()->GetWheelBody(), m_axis_idlers[LEFT]);
    }
    if (m_render_frame_idlers[RIGHT]) {
        RenderFrame(*m_tvehicle->GetTrackAssembly(RIGHT)->GetIdler()->GetWheelBody(), m_axis_idlers[RIGHT]);
    }

    // Contact normals and/or forces on left sprocket.
    // Note that we only render information for contacts on the outside gear profile
    for (const auto& c : m_tvehicle->m_contact_manager->m_sprocket_L_contacts) {
        ChVector<> v1 = c.m_point;
        if (normals) {
            ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis() * scale_normals;
            if (v1.y() > m_tvehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
        if (forces) {
            ChVector<> v2 = v1 + c.m_force * scale_forces;
            if (v1.y() > m_tvehicle->GetTrackAssembly(LEFT)->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
    }

    // Contact normals on right sprocket.
    // Note that we only render information for contacts on the outside gear profile
    for (const auto& c : m_tvehicle->m_contact_manager->m_sprocket_R_contacts) {
        ChVector<> v1 = c.m_point;
        if (normals) {
            ChVector<> v2 = v1 + c.m_csys.Get_A_Xaxis() * scale_normals;
            if (v1.y() < m_tvehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
        if (forces) {
            ChVector<> v2 = v1 + c.m_force * scale_forces;
            if (v1.y() > m_tvehicle->GetTrackAssembly(RIGHT)->GetSprocket()->GetGearBody()->GetPos().y())
                irrlicht::tools::drawSegment(this, v1, v2, ChColor(0.31f, 0.00f, 0.00f), false);
        }
    }

    // Contact normals on monitored track shoes.
    renderContacts(m_tvehicle->m_contact_manager->m_shoe_L_contacts, ChColor(0.31f, 0.00f, 0.00f), normals, forces,
                   scale_normals, scale_forces);
    renderContacts(m_tvehicle->m_contact_manager->m_shoe_R_contacts, ChColor(0.31f, 0.00f, 0.00f), normals, forces,
                   scale_normals, scale_forces);

    // Contact normals on idler wheels.
    renderContacts(m_tvehicle->m_contact_manager->m_idler_L_contacts, ChColor(0.31f, 0.00f, 0.00f), normals, forces,
                   scale_normals, scale_forces);
    renderContacts(m_tvehicle->m_contact_manager->m_idler_R_contacts, ChColor(0.31f, 0.00f, 0.00f), normals, forces,
                   scale_normals, scale_forces);

    // Contact normals on chassis.
    renderContacts(m_tvehicle->m_contact_manager->m_chassis_contacts, ChColor(0.31f, 0.00f, 0.00f), normals, forces,
                   scale_normals, scale_forces);
}

// Render normal for all contacts in the specified list, using the given color.
void ChTrackedVehicleVisualSystemIrrlicht::renderContacts(const std::list<ChTrackContactManager::ContactInfo>& lst,
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
