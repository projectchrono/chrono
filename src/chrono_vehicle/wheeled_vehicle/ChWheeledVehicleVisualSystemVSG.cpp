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

ChWheeledVehicleVisualSystemVSG::ChWheeledVehicleVisualSystemVSG()
    : ChVehicleVisualSystemVSG(),
      m_wvehicle(nullptr),
      m_chassis_visible(true),
      m_suspension_visible(true),
      m_steering_visible(true),
      m_wheel_visible(true),
      m_tire_visible(true) {
    // Set a rendering target of 60 fps to avoid expensive recordAndSubmit() overhead (cpu to gpu data transfers too often)
    // Can override with SetTargetRenderFPS(0) for unlimited or other values
    SetTargetRenderFPS(60);
}

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

    ImGui::Spacing();

    if (ImGui::BeginTable("TireTable", 3, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f))) {
        for (unsigned int i = 0; i < num_driven_axles; i++) {
            unsigned int axle = driven_axles[i];
            ImGui::TableNextColumn();
            ImGui::Text("Axle %1d torques", driven_axles[i]);

            ImGui::TableNextColumn();
            ImGui::Text(" L: %5.1f Nm", driveline->GetSpindleTorque(axle, VehicleSide::LEFT));

            ImGui::TableNextColumn();
            ImGui::Text(" R: %5.1f Nm", driveline->GetSpindleTorque(axle, VehicleSide::RIGHT));

            ImGui::TableNextRow();
        }
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
        static bool suspension_visible = true;
        if (ImGui::Checkbox("Suspension", &suspension_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::SUSPENSION);
            m_suspension_visible = !m_suspension_visible;
            SetBodyObjVisibility(m_suspension_visible, tag);
            SetLinkObjVisibility(m_suspension_visible, tag);
            SetSpringVisibility(m_suspension_visible, tag);
            SetSegmentVisibility(m_suspension_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool steering_visible = true;
        if (ImGui::Checkbox("Steering", &steering_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::STEERING);
            m_steering_visible = !m_steering_visible;
            SetBodyObjVisibility(m_steering_visible, tag);
            SetLinkObjVisibility(m_steering_visible, tag);
            SetSpringVisibility(m_steering_visible, tag);
            SetSegmentVisibility(m_steering_visible, tag);
        }

        ImGui::TableNextColumn();
        static bool wheel_visible = true;
        if (ImGui::Checkbox("Wheel", &wheel_visible)) {
            int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::WHEEL);
            m_wheel_visible = !m_wheel_visible;
            SetBodyObjVisibility(m_wheel_visible, tag);
            SetLinkObjVisibility(m_wheel_visible, tag);
        }

        ////ImGui::TableNextColumn();
        ////static bool tire_visible = true;
        ////if (ImGui::Checkbox("Tire", &tire_visible)) {
        ////    int tag = VehicleObjTag::Generate(vtag, VehiclePartTag::TIRE);
        ////    m_tire_visible = !m_tire_visible;
        ////    SetBodyObjVisibility(m_tire_visible, tag);
        ////    SetLinkObjVisibility(m_tire_visible, tag);
        ////    SetFeaMeshVisibility(m_tire_visible, tag);
        ////}

        ImGui::EndTable();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
