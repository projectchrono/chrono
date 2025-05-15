// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#include <vsgImGui/RenderImGui.h>
#include <vsgImGui/SendEventsToImGui.h>

#include "chrono_vsg/ChGuiComponentVSG.h"

namespace chrono {
namespace vsg3d {

ChGuiComponentVSG::ChGuiComponentVSG() : m_visible(true) {}

void ChGuiComponentVSG::DrawGauge(float val, float v_min, float v_max) {
    ImGui::PushItemWidth(150.0f);
    ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor(200, 100, 20));
    ImGui::SliderFloat("", &val, v_min, v_max, "%.2f");
    ImGui::PopStyleColor();
    ImGui::PopItemWidth();
}

void ChGuiComponentVSG::Colorbar(vsg::ref_ptr<vsgImGui::Texture> texture,
                                 double min_val,
                                 double max_val,
                                 uint32_t deviceID) {
    float range = max_val - min_val;

    int num_items = 6;
    float width = 300;
    float height = (width * texture->height) / texture->width;
    float item_width = width / (num_items - 1);

    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + item_width / 4);
    ImGui::Image(texture->id(deviceID), ImVec2(width, height));

    ImGui::PushItemWidth(item_width);
    ImGui::Text((min_val < 100) ? "%.2f" : "%6.1e", min_val);
    for (int i = 1; i < 6; i++) {
        ImGui::SameLine(i * item_width);
        double val = min_val + i * range / (num_items - 1);
        ImGui::Text((val < 100) ? "%.2f" : "%6.1e", val);
    }
    ImGui::PopItemWidth();
}

}  // namespace vsg3d
}  // namespace chrono
