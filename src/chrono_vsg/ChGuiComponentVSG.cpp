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

void TextCentered(const char* text, float offset) {
    auto text_width = ImGui::CalcTextSize(text).x;
    ImGui::SetCursorPosX(offset - text_width / 2);
    ImGui::Text(text);
}

void ChGuiComponentVSG::Colorbar(vsg::ref_ptr<vsgImGui::Texture> texture,
                                 const ChVector2d& range,
                                 bool bimodal,
                                 float width,
                                 uint32_t deviceID) {
    float min_val = range[0];
    float max_val = range[1];

    int num_items = 5;
    float height = (width * texture->height) / texture->width;
    float item_width = width / (num_items - 1);

    float offset = ImGui::GetCursorPosX() + item_width / 2;
    ImGui::SetCursorPosX(offset);
    ImGui::Image(texture->id(deviceID), ImVec2(width, height));

    char buffer[100];

    ImGui::PushItemWidth(item_width);
    if (bimodal) {
        float delta_neg = (0 - min_val) / ((num_items - 1) / 2);
        for (int i = 0; i < (num_items + 1) / 2; i++) {
            double val = min_val + i * delta_neg;
            sprintf(buffer, (std::abs(val) < 100) ? "%.2f" : "%6.1e", val);
            TextCentered(buffer, offset + i * item_width);
            ImGui::SameLine();
        }
        float delta_pos = (max_val - 0) / ((num_items - 1) / 2);
        for (int i = (num_items + 1) / 2; i < num_items; i++) {
            double val = min_val + i * delta_pos;
            sprintf(buffer, (std::abs(val) < 100) ? "%.2f" : "%6.1e", val);
            TextCentered(buffer, offset + i * item_width);
            ImGui::SameLine();
        }
    } else {
        float delta = (max_val - min_val) / (num_items - 1);
        for (int i = 0; i < num_items; i++) {
            double val = min_val + i * delta;
            sprintf(buffer, (std::abs(val) < 100) ? "%.2f" : "%6.1e", val);
            TextCentered(buffer, offset + i * item_width);
            ImGui::SameLine();
        }
    }
    ImGui::PopItemWidth();
}

}  // namespace vsg3d
}  // namespace chrono
