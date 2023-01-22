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
    auto sprk_speed_L = m_tvehicle->GetDriveline()->GetSprocketSpeed(LEFT) * 30.0 / CH_C_PI;
    auto sprk_speed_R = m_tvehicle->GetDriveline()->GetSprocketSpeed(RIGHT) * 30.0 / CH_C_PI;

    char label[64];
    int nstr = sizeof(label) - 1;

    ImGui::Spacing();

    if (ImGui::BeginTable("TrackDriveTable", 3, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextColumn();
        snprintf(label, nstr, "Sprocket Torque");
        ImGui::Text(label);
        ImGui::TableNextColumn();
        snprintf(label, nstr, "L: %+5.1f Nm", sprk_torque_L);
        ImGui::Text(label);
        ImGui::TableNextColumn();
        snprintf(label, nstr, " R: %+5.1f Nm", sprk_torque_R);
        ImGui::Text(label);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        snprintf(label, nstr, "Sprocket Speed");
        ImGui::Text(label);
        ImGui::TableNextColumn();
        snprintf(label, nstr, "L: %+5.1f RPM", sprk_speed_L);
        ImGui::Text(label);
        ImGui::TableNextColumn();
        snprintf(label, nstr, " R: %+5.1f RPM", sprk_speed_R);
        ImGui::Text(label);
        ImGui::TableNextRow();
        ImGui::EndTable();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
