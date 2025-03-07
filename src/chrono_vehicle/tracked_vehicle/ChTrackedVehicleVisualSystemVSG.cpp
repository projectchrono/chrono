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
// VSG-based visualization for tracked vehicles.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

ChTrackedVehicleVisualSystemVSG::ChTrackedVehicleVisualSystemVSG()
    : ChVehicleVisualSystemVSG(),
      m_tvehicle(nullptr),
      m_chassis_visible(true),
      m_sprocket_visible(true),
      m_idler_visible(true),
      m_suspension_visible(true),
      m_shoe_visible(true),
      m_wheel_visible(true) {}

void ChTrackedVehicleVisualSystemVSG::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemVSG::AttachVehicle(vehicle);
    m_tvehicle = dynamic_cast<ChTrackedVehicle*>(m_vehicle);
    assert(m_tvehicle);
}

void ChTrackedVehicleVisualSystemVSG::AppendGUIStats() {
    auto sprk_torque_L = m_tvehicle->GetDriveline()->GetSprocketTorque(LEFT);
    auto sprk_torque_R = m_tvehicle->GetDriveline()->GetSprocketTorque(RIGHT);
    auto sprk_speed_L = m_tvehicle->GetDriveline()->GetSprocketSpeed(LEFT) * 30.0 / CH_PI;
    auto sprk_speed_R = m_tvehicle->GetDriveline()->GetSprocketSpeed(RIGHT) * 30.0 / CH_PI;

    ImGui::Spacing();

    if (ImGui::BeginTable("TrackDriveTable", 3, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Sprocket Torque");
        ImGui::TableNextColumn();
        ImGui::Text("L: %+5.1f Nm", sprk_torque_L);
        ImGui::TableNextColumn();
        ImGui::Text(" R: %+5.1f Nm", sprk_torque_R);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Sprocket Speed");
        ImGui::TableNextColumn();
        ImGui::Text("L: %+5.1f RPM", sprk_speed_L);
        ImGui::TableNextColumn();
        ImGui::Text(" R: %+5.1f RPM", sprk_speed_R);
        ImGui::TableNextRow();
        ImGui::EndTable();
    }

    if (ImGui::BeginTable("Subsystems", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        uint16_t vtag = m_vehicle->GetVehicleTag();

        ImGui::TableNextColumn();
        static bool chassis_visible = true;
        if (ImGui::Checkbox("Chassis", &chassis_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::CHASSIS);
            m_chassis_visible = !m_chassis_visible;
            SetBodyObjVisibility(m_chassis_visible, tag);
            SetLinkObjVisibility(m_chassis_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool sprocket_visible = true;
        if (ImGui::Checkbox("Sprockets", &sprocket_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::SPROCKET);
            m_sprocket_visible = !m_sprocket_visible;
            SetBodyObjVisibility(m_sprocket_visible, tag);
            SetLinkObjVisibility(m_sprocket_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool idler_visible = true;
        if (ImGui::Checkbox("Idlers", &idler_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::IDLER);
            m_idler_visible = !m_idler_visible;
            SetBodyObjVisibility(m_idler_visible, tag);
            SetLinkObjVisibility(m_idler_visible, tag);
            SetSpringVisibility(m_idler_visible, tag);
            SetSegmentVisibility(m_idler_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool suspension_visible = true;
        if (ImGui::Checkbox("Suspension", &suspension_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::TRACK_SUSPENSION);
            m_suspension_visible = !m_suspension_visible;
            SetBodyObjVisibility(m_suspension_visible, tag);
            SetLinkObjVisibility(m_suspension_visible, tag);
            SetSpringVisibility(m_suspension_visible, tag);
            SetSegmentVisibility(m_suspension_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool shoe_visible = true;
        if (ImGui::Checkbox("Track shoes", &shoe_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::SHOE);
            m_shoe_visible = !m_shoe_visible;
            SetBodyObjVisibility(m_shoe_visible, tag);
            SetLinkObjVisibility(m_shoe_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool wheel_visible = true;
        if (ImGui::Checkbox("Track wheels", &wheel_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::TRACK_WHEEL);
            m_wheel_visible = !m_wheel_visible;
            SetBodyObjVisibility(m_wheel_visible, tag);
            SetLinkObjVisibility(m_wheel_visible, tag);
        }

        ImGui::EndTable();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
