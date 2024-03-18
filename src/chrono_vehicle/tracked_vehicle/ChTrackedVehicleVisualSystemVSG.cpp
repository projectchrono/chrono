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

ChTrackedVehicleVisualSystemVSG::ChTrackedVehicleVisualSystemVSG() : ChVehicleVisualSystemVSG(), m_tvehicle(nullptr) {}

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
}

}  // end namespace vehicle
}  // end namespace chrono
