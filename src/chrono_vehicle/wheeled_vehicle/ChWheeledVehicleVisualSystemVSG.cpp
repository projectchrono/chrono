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
// VSG-based visualization for wheeled vehicles.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline8WD.h"

namespace chrono {
namespace vehicle {

ChWheeledVehicleVisualSystemVSG::ChWheeledVehicleVisualSystemVSG() : ChVehicleVisualSystemVSG(), m_wvehicle(nullptr) {}

void ChWheeledVehicleVisualSystemVSG::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemVSG::AttachVehicle(vehicle);
    m_wvehicle = dynamic_cast<ChWheeledVehicle*>(m_vehicle);
    assert(m_wvehicle);
}

void ChWheeledVehicleVisualSystemVSG::AppendGUIStats() {
    const auto& driveline = m_wvehicle->GetDriveline();
    const auto& num_driven_axles = driveline->GetNumDrivenAxles();
    const auto& driven_axles = driveline->GetDrivenAxleIndexes();

    if (num_driven_axles < 1)
        return;

    char label[64];
    int nstr = sizeof(label) - 1;

    ImGui::Spacing();

    if (ImGui::BeginTable("TireTable", 3, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        for (int i = 0; i < num_driven_axles; i++) {
            int axle = driven_axles[i];
            ImGui::TableNextColumn();
            snprintf(label, nstr, "Axle %1d torques", driven_axles[i]);
            ImGui::Text(label);

            ImGui::TableNextColumn();
            snprintf(label, nstr, " L: %5.1f Nm", driveline->GetSpindleTorque(axle, VehicleSide::LEFT));
            ImGui::Text(label);

            ImGui::TableNextColumn();
            snprintf(label, nstr, " R: %5.1f Nm", driveline->GetSpindleTorque(axle, VehicleSide::RIGHT));
            ImGui::Text(label);

            ImGui::TableNextRow();
        }
        ImGui::EndTable();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
