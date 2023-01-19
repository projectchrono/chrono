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
    : ChVehicleVisualSystemVSG(), m_wvehicle(nullptr), m_drivenAxles(0) {}

void ChWheeledVehicleVisualSystemVSG::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystemVSG::AttachVehicle(vehicle);

    m_wvehicle = dynamic_cast<ChWheeledVehicle*>(m_vehicle);
    if (!m_wvehicle)
        return;

    //// RADU TODO
    ////   eliminate.  Num. driven axles already available through ChWheeledVehicle!

    if (auto driveline2 = std::dynamic_pointer_cast<ChShaftsDriveline2WD>(m_wvehicle->GetDriveline())) {
        m_drivenAxles = 1;
    } else if (auto driveline4 = std::dynamic_pointer_cast<ChShaftsDriveline4WD>(m_wvehicle->GetDriveline())) {
        m_drivenAxles = 2;
    } else if (auto driveline8 = std::dynamic_pointer_cast<ChShaftsDriveline8WD>(m_wvehicle->GetDriveline())) {
        m_drivenAxles = 4;
    }
}

double ChWheeledVehicleVisualSystemVSG::GetTireTorque(int draxle, int side) {
    if (!m_wvehicle)
        return 0.0;

    if (auto driveline2 = std::dynamic_pointer_cast<ChShaftsDriveline2WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline2->GetDrivenAxleIndexes()[0];
        if (side == 0)
            torque = driveline2->GetSpindleTorque(axle, LEFT);
        else
            torque = driveline2->GetSpindleTorque(axle, RIGHT);
        return torque;
    } else if (auto driveline4 = std::dynamic_pointer_cast<ChShaftsDriveline4WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline4->GetDrivenAxleIndexes()[draxle];
        if (side == 0)
            torque = driveline4->GetSpindleTorque(axle, LEFT);
        else
            torque = driveline4->GetSpindleTorque(axle, RIGHT);
        return torque;
    }
    if (auto driveline8 = std::dynamic_pointer_cast<ChShaftsDriveline8WD>(m_wvehicle->GetDriveline())) {
        double torque;
        int axle = driveline8->GetDrivenAxleIndexes()[draxle];
        if (side == 0)
            torque = driveline8->GetSpindleTorque(axle, LEFT);
        else
            torque = driveline8->GetSpindleTorque(axle, RIGHT);
        return torque;
    } else {
        return 0.0;
    }
}

void ChWheeledVehicleVisualSystemVSG::AppendGUIStats() {
    char label[64];
    int nstr = sizeof(label) - 1;

    //// RADU TODO
    ////  simplify: loop over driven axles and eliminate GetTireTorque()

    if (m_drivenAxles > 0) {
        ImGui::Spacing();

        ImGui::BeginTable("TireTable", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                          ImVec2(0.0f, 0.0f));
        ImGui::TableNextColumn();
        snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", GetTireTorque(0, 0));
        ImGui::Text(label);
        ImGui::TableNextColumn();
        snprintf(label, nstr, " R: %+5.1f Nm", GetTireTorque(0, 1));
        ImGui::Text(label);
        ImGui::TableNextRow();
        if (m_drivenAxles >= 2) {
            ImGui::TableNextColumn();
            snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", GetTireTorque(1, 0));
            ImGui::Text(label);
            ImGui::TableNextColumn();
            snprintf(label, nstr, " R: %+5.1f Nm", GetTireTorque(1, 1));
            ImGui::Text(label);
            ImGui::TableNextRow();
        }
        if (m_drivenAxles >= 4) {
            ImGui::TableNextColumn();
            snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", GetTireTorque(2, 0));
            ImGui::Text(label);
            ImGui::TableNextColumn();
            snprintf(label, nstr, " R: %+5.1f Nm", GetTireTorque(2, 1));
            ImGui::Text(label);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            snprintf(label, nstr, "Torques wheel L: %+5.1f Nm", GetTireTorque(3, 0));
            ImGui::Text(label);
            ImGui::TableNextColumn();
            snprintf(label, nstr, " R: %+5.1f Nm", GetTireTorque(3, 1));
            ImGui::Text(label);
            ImGui::TableNextRow();
        }
        ImGui::EndTable();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
