// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
//
// Implementation of the base Chrono::VSG GUI components.
//
// =============================================================================
// Rainer Gericke, Radu Serban
// =============================================================================

#include <ctime>

#include "chrono/utils/ChUtils.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/impl/BaseGuiComponents.h"

namespace chrono {
namespace vsg3d {

// -----------------------------------------------------------------------------

ChBaseGuiComponentVSG::ChBaseGuiComponentVSG(ChVisualSystemVSG* app) : m_app(app) {}

void ChBaseGuiComponentVSG::render(vsg::CommandBuffer& cb) {
    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowPos(ImVec2(5.0f, 5.0f));

    ImGuiTableFlags table_flags = ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit;
    ImGuiColorEditFlags color_edit_flags =
        ImGuiColorEditFlags_Float | ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoDragDrop;

    ImGui::Begin("Simulation");

    if (ImGui::BeginTable("SimTable", 2, table_flags, ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Model Time:");
        ImGui::TableNextColumn();
        ImGui::Text("%8.3f s", m_app->GetSimulationTime());

        ImGui::TableNextRow();

        double current_time = double(clock()) / double(CLOCKS_PER_SEC);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Wall Clock Time:");
        ImGui::TableNextColumn();
        ImGui::Text("%8.3f s", current_time - m_app->m_start_time);

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Real Time Factor:");
        ImGui::SameLine();
        ChGuiComponentVSG::HelpMarker(
            "Overall real-time factor.\n"
            "The RTF represents the ratio between the wall clock time elapsed between two render "
            "frames and the duration by which simulation was advanced in this interval.");
        ImGui::TableNextColumn();
        ImGui::Text("%8.3f", m_app->GetRTF());

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Rendering FPS:");
        ImGui::TableNextColumn();
        ImGui::Text("%8.3f", m_app->GetRenderingFPS());

        ImGui::EndTable();
    }

    if (ImGui::BeginTable("Counters", 2, table_flags, ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Counters");
        ImGui::TableNextColumn();

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Num. active bodies:");
        ImGui::TableNextColumn();
        ImGui::Text("%8d", m_app->GetNumBodies());

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Num. active shafts:");
        ImGui::TableNextColumn();
        ImGui::Text("%8d", m_app->GetNumShafts());

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Num. active links:");
        ImGui::TableNextColumn();
        ImGui::Text("%8d", m_app->GetNumLinks());

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Num. FEA meshes:");
        ImGui::TableNextColumn();
        ImGui::Text("%8d", m_app->GetNumMeshes());

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Num. contacts:");
        ImGui::TableNextColumn();
        ImGui::Text("%8d", m_app->GetNumContacts());

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Num. states");
        ImGui::TableNextColumn();
        ImGui::Text("%8d", m_app->GetNumStates());

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Num. constraints:");
        ImGui::TableNextColumn();
        ImGui::Text("%8d", m_app->GetNumConstraints());

        ImGui::EndTable();
    }

    if (m_app->m_show_visibility_controls && ImGui::CollapsingHeader("Components")) {
        if (ImGui::BeginTable("Component visibility", 2, table_flags, ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool body_obj_visible = m_app->m_show_body_objs;
            if (ImGui::Checkbox("Bodies", &body_obj_visible)) {
                m_app->m_show_body_objs = !m_app->m_show_body_objs;
                m_app->SetBodyObjVisibility(m_app->m_show_body_objs, -1);
            }

            ImGui::TableNextColumn();
            static bool link_obj_visible = m_app->m_show_link_objs;
            if (ImGui::Checkbox("Links", &link_obj_visible)) {
                m_app->m_show_link_objs = !m_app->m_show_link_objs;
                m_app->SetLinkObjVisibility(m_app->m_show_link_objs, -1);
            }

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            static bool fea_mesh_visible = m_app->m_show_fea_meshes;
            if (ImGui::Checkbox("FEA meshes", &fea_mesh_visible)) {
                m_app->m_show_fea_meshes = !m_app->m_show_fea_meshes;
                m_app->SetFeaMeshVisibility(m_app->m_show_fea_meshes, -1);
            }

            ImGui::TableNextColumn();
            static bool spring_damper_visible = m_app->m_show_spring_dampers;
            if (ImGui::Checkbox("Spring-dampers", &spring_damper_visible)) {
                m_app->m_show_spring_dampers = !m_app->m_show_spring_dampers;
                m_app->SetSpringVisibility(m_app->m_show_spring_dampers, -1);
                m_app->SetSegmentVisibility(m_app->m_show_spring_dampers, -1);
            }

            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader("Labels")) {
        if (ImGui::BeginTable("Component labels", 3, table_flags, ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool show_body_labels = m_app->m_show_body_labels;
            if (ImGui::Checkbox("Body labels", &show_body_labels))
                m_app->ToggleBodyLabelVisibility();
            ImGui::TableNextColumn();
            ImVec4 body_labels_color(m_app->m_body_labels_color.R, m_app->m_body_labels_color.G,
                                     m_app->m_body_labels_color.B, 0);
            if (ImGui::ColorEdit3("color##body_labels", (float*)&body_labels_color, color_edit_flags))
                m_app->SetBodyLabelsColor(ChColor(body_labels_color.x, body_labels_color.y, body_labels_color.z));
            ImGui::TableNextColumn();
            float body_labels_scale = m_app->m_body_labels_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##body_labels", &body_labels_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_body_labels_scale = body_labels_scale;

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            static bool show_link_labels = m_app->m_show_link_labels;
            if (ImGui::Checkbox("Link labels", &show_link_labels))
                m_app->ToggleLinkLabelVisibility();
            ImGui::TableNextColumn();
            ImVec4 link_labels_color(m_app->m_link_labels_color.R, m_app->m_link_labels_color.G,
                                     m_app->m_link_labels_color.B, 0);
            if (ImGui::ColorEdit3("color##link_labels", (float*)&link_labels_color, color_edit_flags))
                m_app->SetLinkLabelsColor(ChColor(link_labels_color.x, link_labels_color.y, link_labels_color.z));
            ImGui::TableNextColumn();
            float link_labels_scale = m_app->m_link_labels_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##link_labels", &link_labels_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_link_labels_scale = link_labels_scale;

            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader("Frames")) {
        if (ImGui::BeginTable("Frames", 2, table_flags, ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool abs_frame_active = m_app->m_show_abs_frame;
            if (ImGui::Checkbox("Global frame", &abs_frame_active))
                m_app->ToggleAbsFrameVisibility();
            ImGui::TableNextColumn();
            float abs_frame_scale = m_app->m_abs_frame_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##abs", &abs_frame_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_abs_frame_scale = abs_frame_scale;

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            static bool bRef_frame_active = m_app->m_show_ref_frames;
            if (ImGui::Checkbox("Body ref frames", &bRef_frame_active))
                m_app->ToggleRefFrameVisibility();
            ImGui::TableNextColumn();
            float ref_frame_scale = m_app->m_ref_frame_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##ref", &ref_frame_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_ref_frame_scale = ref_frame_scale;

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            ImGui::BeginGroup();
            static bool show_com_frames = m_app->m_show_com_frames;
            if (ImGui::Checkbox("COM frames", &show_com_frames))
                m_app->ToggleCOMFrameVisibility();
            ImGui::SameLine();
            static bool show_com_symbols = m_app->m_show_com_symbols;
            if (ImGui::Checkbox("Symbol", &show_com_symbols))
                m_app->ToggleCOMSymbolVisibility();
            ImGui::EndGroup();

            ImGui::TableNextColumn();
            float com_frame_scale = m_app->m_com_frame_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##com_frame", &com_frame_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            if (com_frame_scale != m_app->m_com_frame_scale) {
                m_app->m_com_frame_scale = com_frame_scale;
                m_app->m_com_size_changed = true;
            }

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            static bool bLink_frame_active = m_app->m_show_link_frames;
            if (ImGui::Checkbox("Link frames", &bLink_frame_active))
                m_app->ToggleLinkFrameVisibility();
            ImGui::TableNextColumn();
            float link_frame_scale = m_app->m_link_frame_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##link", &link_frame_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_link_frame_scale = link_frame_scale;

            ImGui::EndTable();
        }
    }

    if (ImGui::CollapsingHeader("Collision & Contact")) {
        if (ImGui::BeginTable("Collision", 3, table_flags, ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool show_collision = m_app->m_show_collision;
            if (ImGui::Checkbox("Collision shapes", &show_collision)) {
                m_app->m_show_collision = !m_app->m_show_collision;
                m_app->SetCollisionVisibility(m_app->m_show_collision, -1);
            }
            ImGui::TableNextColumn();
            ImVec4 collision_color(m_app->m_collision_color.R, m_app->m_collision_color.G, m_app->m_collision_color.B,
                                   0);
            if (ImGui::ColorEdit3("color##collision", (float*)&collision_color, color_edit_flags)) {
                m_app->SetCollisionColor(ChColor(collision_color.x, collision_color.y, collision_color.z));
            }
            ImGui::TableNextColumn();

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            static bool show_contact_normals = m_app->m_show_contact_normals;
            if (ImGui::Checkbox("Contact normals", &show_contact_normals)) {
                m_app->m_show_contact_normals = !m_app->m_show_contact_normals;
                m_app->SetContactNormalsVisibility(m_app->m_show_contact_normals, -1);
            }
            ImGui::TableNextColumn();
            ImVec4 contact_normals_color(m_app->m_contact_normals_color.R, m_app->m_contact_normals_color.G,
                                         m_app->m_contact_normals_color.B, 0);
            if (ImGui::ColorEdit3("color##contact_normals", (float*)&contact_normals_color, color_edit_flags)) {
                m_app->SetContactNormalsColor(
                    ChColor(contact_normals_color.x, contact_normals_color.y, contact_normals_color.z));
            }
            ImGui::TableNextColumn();
            float contact_normals_scale = m_app->m_contact_normals_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##contact_normals", &contact_normals_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_contact_normals_scale = contact_normals_scale;

            ImGui::TableNextRow();

            ImGui::TableNextColumn();
            static bool show_contact_forces = m_app->m_show_contact_forces;
            if (ImGui::Checkbox("Contact forces", &show_contact_forces)) {
                m_app->m_show_contact_forces = !m_app->m_show_contact_forces;
                m_app->SetContactForcesVisibility(m_app->m_show_contact_forces, -1);
            }
            ImGui::TableNextColumn();
            ImVec4 contact_forces_color(m_app->m_contact_forces_color.R, m_app->m_contact_forces_color.G,
                                        m_app->m_contact_forces_color.B, 0);
            if (ImGui::ColorEdit3("color##contact_forces", (float*)&contact_forces_color, color_edit_flags)) {
                m_app->SetContactForcesColor(
                    ChColor(contact_forces_color.x, contact_forces_color.y, contact_forces_color.z));
            }
            ImGui::TableNextColumn();
            float contact_forces_scale = m_app->m_contact_forces_scale;
            ImGui::PushItemWidth(120.0f);
            ImGui::SliderFloat("scale##contact_forces", &contact_forces_scale, 0.1f, 10.0f);
            ImGui::PopItemWidth();
            m_app->m_contact_forces_scale = contact_forces_scale;

            ImGui::EndTable();
        }
    }

    ImGui::Spacing();

    if (ImGui::Button("Quit"))
        m_app->Quit();

    ImGui::End();
}

ChCameraGuiComponentVSG::ChCameraGuiComponentVSG(ChVisualSystemVSG* app) : m_app(app) {
    m_visible = false;
}

void ChCameraGuiComponentVSG::render(vsg::CommandBuffer& cb) {
    auto p = m_app->GetCameraPosition();
    auto t = m_app->GetCameraTarget();

    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));

    ImGuiTableFlags table_flags = ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit;

    ImGui::Begin("Camera");

    if (ImGui::BeginTable("Location", 4, table_flags, ImVec2(0.0f, 0.0f))) {
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Location");
        for (int i = 0; i < 3; i++) {
            ImGui::TableNextColumn();
            ImGui::Text(" %5.1f", p[i]);
        }

        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Look-at");
        for (int i = 0; i < 3; i++) {
            ImGui::TableNextColumn();
            ImGui::Text(" %5.1f", t[i]);
        }

        ImGui::EndTable();

        ImGui::End();
    }
}

ChColorbarGuiComponentVSG::ChColorbarGuiComponentVSG(const std::string& title,
                                                     const ChVector2d& range,
                                                     ChColormap::Type type,
                                                     bool bimodal,
                                                     float width)
    : m_title(title), m_type(type), m_range(range), m_bimodal(bimodal), m_width(width) {}

void ChColorbarGuiComponentVSG::Initialize() {
    m_texture = m_vsys->GetColormapTexture(m_type);
}

void ChColorbarGuiComponentVSG::render(vsg::CommandBuffer& cb) {
    ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
    ImGui::Begin(m_title.c_str());

    Colorbar(m_texture, m_range, m_bimodal, m_width, cb.deviceID);

    ImGui::End();
}

}  // namespace vsg3d
}  // namespace chrono
