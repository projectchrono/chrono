// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// Screen capture code from https://github.com/vsg-dev/vsgExamples.git
//
// =============================================================================
// Radu Serban, Rainer Gericke
// =============================================================================

#include <algorithm>
#include <cstddef>
#include <cctype>
#include <sstream>
#include <iomanip>

#include "chrono/utils/ChUtils.h"

#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"

#include "chrono_vsg/ChVisualSystemVSG.h"
#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/utils/ChUtilsVSG.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vsg3d {

// -----------------------------------------------------------------------------

// Helper to display a little (?) mark which shows a tooltip when hovered (from ImGui demo).
static void HelpMarker(const char* desc) {
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip()) {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

class ChMainGuiVSG : public vsg::Inherit<vsg::Command, ChMainGuiVSG> {
  public:
    ChMainGuiVSG(ChVisualSystemVSG* app, vsg::ref_ptr<vsg::Options> options = {}, float tex_height = 64)
        : m_app(app), m_tex_height(tex_height) {
        // Create textures
        {
            auto texData = vsg::read_cast<vsg::Data>(m_app->m_logo_filename, options);
            m_app->m_logo_texture = vsgImGui::Texture::create_if(texData, texData);
        }

        for (const auto& cmap_files : ChColormap::GetFilenames()) {
            auto texData = vsg::read_cast<vsg::Data>(cmap_files.second.img_file, options);
            auto texture = vsgImGui::Texture::create_if(texData, texData);
            m_app->m_colormap_textures[cmap_files.first] = texture;
        }
    }

    // Textures must be compiled before we can use them for rendering
    void compile(vsg::Context& context) override {
        m_app->m_logo_texture->compile(context);
        for (const auto& cmap : m_app->m_colormap_textures)
            cmap.second->compile(context);
    }

    // Example here taken from the Dear imgui comments (mostly)
    void record(vsg::CommandBuffer& cb) const override {
        // Display logo first, so gui elements can cover it
        ImVec2 squareUV(1.0f, 1.0f);  // UV in the logo texture - usually rectangular

        if (m_app->m_show_logo) {
            const float sizey = m_tex_height;
            const float sizex =
                sizey * static_cast<float>(m_app->m_logo_texture->width) / m_app->m_logo_texture->height;
            const float pad = 10;

            // Copied from imgui_demo.cpp simple overlay
            ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                            ImGuiWindowFlags_NoNav;
            const ImGuiViewport* viewport = ImGui::GetMainViewport();
            ImVec2 work_pos = viewport->WorkPos;  // Use work area to avoid menu-bar/task-bar, if any!
            ImVec2 work_size = viewport->WorkSize;
            ImVec2 window_pos, window_pos_pivot;
            window_pos.x = work_pos.x + work_size.x - sizex - m_app->m_logo_pos.x() - pad;
            window_pos.y = work_pos.y + sizey + m_app->m_logo_pos.y() + pad;
            window_pos_pivot.x = 0.0f;
            window_pos_pivot.y = 1.0f;
            ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, window_pos_pivot);
            window_flags |= ImGuiWindowFlags_NoMove;
            ImGui::SetNextWindowBgAlpha(0.0f);  // Transparent background
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            ImGui::Begin("vsgCS UI", nullptr, window_flags);

            // Display a rectangle from the VSG logo
            ImGui::Image(m_app->m_logo_texture->id(cb.deviceID), ImVec2(sizex, sizey), ImVec2(0.0f, 0.0f), squareUV);

            ImGui::End();
            ImGui::PopStyleVar();
        }

        // Render GUI
        if (m_app->m_show_gui) {
            for (auto& gui : m_app->m_gui) {
                if (gui->IsVisible())
                    gui->render(cb);
            }
        }
    }

    ChVisualSystemVSG* m_app;
    float m_tex_height;
};

// -----------------------------------------------------------------------------

class ChBaseGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChBaseGuiComponentVSG(ChVisualSystemVSG* app) : m_app(app) {}

    // Example here taken from the Dear imgui comments (mostly)
    virtual void render(vsg::CommandBuffer& cb) override {
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));

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
            HelpMarker(
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
                ImVec4 collision_color(m_app->m_collision_color.R, m_app->m_collision_color.G,
                                       m_app->m_collision_color.B, 0);
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

    ChVisualSystemVSG* m_app;
};

class ChCameraGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChCameraGuiComponentVSG(ChVisualSystemVSG* app) : m_app(app) { m_visible = false; }

    virtual void render(vsg::CommandBuffer& cb) override {
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

    ChVisualSystemVSG* m_app;
};

class ChColorbarGuiComponentVSG : public ChGuiComponentVSG {
  public:
    ChColorbarGuiComponentVSG(const std::string& title,
                              const ChVector2d& range,
                              ChColormap::Type type,
                              bool bimodal,
                              float width = 400)
        : m_title(title), m_type(type), m_range(range), m_bimodal(bimodal), m_width(width) {}

    virtual void Initialize() override { m_texture = m_vsys->GetColormapTexture(m_type); }

    virtual void render(vsg::CommandBuffer& cb) override {
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin(m_title.c_str());

        Colorbar(m_texture, m_range, m_bimodal, m_width, cb.deviceID);

        ImGui::End();
    }

  private:
    std::string m_title;
    ChColormap::Type m_type;
    bool m_bimodal;
    vsg::ref_ptr<vsgImGui::Texture> m_texture;
    float m_width;
    ChVector2d m_range;
};

// -----------------------------------------------------------------------------

class EventHandlerWrapper : public vsg::Inherit<vsg::Visitor, EventHandlerWrapper> {
  public:
    EventHandlerWrapper(std::shared_ptr<ChEventHandlerVSG> component, ChVisualSystemVSG* app)
        : m_component(component), m_app(app) {}

    void apply(vsg::KeyPressEvent& keyPress) override { m_component->process(keyPress); }

  private:
    std::shared_ptr<ChEventHandlerVSG> m_component;
    ChVisualSystemVSG* m_app;
};

class ChBaseEventHandlerVSG : public ChEventHandlerVSG {
  public:
    ChBaseEventHandlerVSG(ChVisualSystemVSG* app) : m_app(app) {}

    virtual void process(vsg::KeyPressEvent& keyPress) override {
        if (keyPress.keyBase == 'm' || keyPress.keyModified == 'm') {
            m_app->ToggleGuiVisibility();
        }
        if (keyPress.keyBase == 'n' || keyPress.keyModified == 'n') {
            m_app->GetGuiComponent(m_app->m_camera_gui)->ToggleVisibility();
        }
        if (keyPress.keyBase == vsg::KEY_Escape || keyPress.keyModified == 65307) {
            m_app->Quit();
        }
    }

    ChVisualSystemVSG* m_app;
};

// -----------------------------------------------------------------------------

// Utility visitor class for accessing the vec3 data in the N-th vertex buffer of an object.
// Note: since VSG v.1.0.8 VertexIndexDraw is used instead of BindVertexBuffers!
template <int N>
class FindVec3BufferData : public vsg::Visitor {
  public:
    FindVec3BufferData() : m_buffer(nullptr) {}
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::BindVertexBuffers& bvd) override {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexDraw& vd) override {
        if (vd.arrays.empty())
            return;
        vd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexIndexDraw& vid) override {
        if (vid.arrays.empty())
            return;
        vid.arrays[N]->data->accept(*this);
    }
    void apply(vsg::vec3Array& vertices) override {
        if (!m_buffer)
            m_buffer = &vertices;
    }
    vsg::ref_ptr<vsg::vec3Array> getBufferData() {
        vsg::ref_ptr<vsg::vec3Array> data;
        data = const_cast<vsg::vec3Array*>(m_buffer);
        return data;
    }
    vsg::vec3Array* m_buffer;
};

// Utility visitor class for accessing the vec4 data in the N-th vertex buffer of an object.
// Note: since VSG v.1.0.8 VertexIndexDraw is used instead of BindVertexBuffers!
template <int N>
class FindVec4BufferData : public vsg::Visitor {
  public:
    FindVec4BufferData() : m_buffer(nullptr) {}
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::BindVertexBuffers& bvd) override {
        if (bvd.arrays.empty())
            return;
        bvd.arrays[N]->data->accept(*this);
    }
    void apply(vsg::VertexIndexDraw& vid) override {
        if (vid.arrays.empty())
            return;
        vid.arrays[N]->data->accept(*this);
    }
    void apply(vsg::vec4Array& vertices) override {
        if (!m_buffer)
            m_buffer = &vertices;
    }
    vsg::ref_ptr<vsg::vec4Array> getBufferData() {
        vsg::ref_ptr<vsg::vec4Array> data;
        data = const_cast<vsg::vec4Array*>(m_buffer);
        return data;
    }
    vsg::vec4Array* m_buffer;
};

// Custom VertexIndexDraw variant that can request extra buffer usage flags (e.g., storage writes for GPU colouring)
class ChronoVertexIndexDraw : public vsg::Inherit<vsg::VertexIndexDraw, ChronoVertexIndexDraw> {
  public:
    ChronoVertexIndexDraw() = default;
    ChronoVertexIndexDraw(const vsg::VertexIndexDraw& rhs, const vsg::CopyOp& copyop = {}) : Inherit(rhs, copyop) {}
    ChronoVertexIndexDraw(const ChronoVertexIndexDraw& rhs, const vsg::CopyOp& copyop = {})
        : Inherit(rhs, copyop), m_extraUsage(rhs.m_extraUsage) {}

    void setExtraUsage(VkBufferUsageFlags extraUsage) { m_extraUsage = extraUsage; }
    VkBufferUsageFlags getExtraUsage() const { return m_extraUsage; }

    void compile(vsg::Context& context) override {
        if (arrays.empty() || !indices)
            return;

        const auto deviceID = context.deviceID;

        bool requiresCreateAndCopy = !indices->buffer;
        if (!requiresCreateAndCopy) {
            if (indices->requiresCopy(deviceID)) {
                requiresCreateAndCopy = true;
            } else {
                for (auto& array : arrays) {
                    if (!array->buffer || array->requiresCopy(deviceID)) {
                        requiresCreateAndCopy = true;
                        break;
                    }
                }
            }
        }

        if (requiresCreateAndCopy) {
            // When the original VSG node only asked for vertex/index usage, rebuild the combined buffer with any
            // extra usage bits so compute shaders can write into the same allocation the renderer reads from
            // otherwise the colourmapping wont be accurate for particles
            vsg::BufferInfoList combinedBufferInfos(arrays);
            combinedBufferInfos.push_back(indices);

            VkBufferUsageFlags usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT | m_extraUsage;
            createBufferAndTransferData(context, combinedBufferInfos, usage, VK_SHARING_MODE_EXCLUSIVE);
        }

        assignVulkanArrayData(deviceID, arrays, _vulkanData[deviceID]);
    }

  private:
    VkBufferUsageFlags m_extraUsage = 0;
};

// Utility visitor for fetching the BufferInfo used by the N-th vertex array binding.
template <int N>
class FindVertexArrayBufferInfo : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) override {
        // Continue traversal until the target binding is discovered
        if (!bufferInfo)
            object.traverse(*this);
    }

    void apply(vsg::BindVertexBuffers& bvb) override {
        // Grab the BufferInfo from legacy BindVertexBuffers nodes when slot N exists
        if (bufferInfo || bvb.arrays.size() <= N)
            return;
        bufferInfo = bvb.arrays[N].cast<vsg::BufferInfo>();
    }

    void apply(vsg::VertexIndexDraw& vid) override {
        // Support modern VertexIndexDraw nodes that replaced BindVertexBuffers in newer VSG
        if (bufferInfo || vid.arrays.size() <= N)
            return;
        bufferInfo = vid.arrays[N].cast<vsg::BufferInfo>();
    }

    vsg::ref_ptr<vsg::BufferInfo> bufferInfo;
};

class ReplaceVertexIndexDraw : public vsg::Inherit<vsg::Visitor, ReplaceVertexIndexDraw> {
  public:
    explicit ReplaceVertexIndexDraw(VkBufferUsageFlags extraUsage = 0) : m_extraUsage(extraUsage) {}

    vsg::ref_ptr<ChronoVertexIndexDraw> replaced_node;

    void apply(vsg::Object& object) override {
        if (replaced_node)
            return;
        object.traverse(*this);
    }

    void apply(vsg::Node& node) override {
        if (replaced_node)
            return;
        node.traverse(*this);
    }

    void apply(vsg::Group& group) override {
        if (replaced_node)
            return;
        for (auto& child : group.children) {
            if (!child)
                continue;

            if (auto vid = child.cast<vsg::VertexIndexDraw>()) {
                // Clone the original draw node but request extra usage flags so the rebuilt buffers support
                // compute shader writes (needed for GPU particle colouring!)
                auto chrono_vid = ChronoVertexIndexDraw::create(*vid);
                chrono_vid->setExtraUsage(m_extraUsage);
                for (auto& array : chrono_vid->arrays) {
                    if (array)
                        array->release();
                }
                if (chrono_vid->indices)
                    chrono_vid->indices->release();
                child = chrono_vid;
                replaced_node = chrono_vid;
                return;
            }

            child->accept(*this);
            if (replaced_node)
                return;
        }
    }

  private:
    VkBufferUsageFlags m_extraUsage;
};

// -----------------------------------------------------------------------------

struct Merge : public vsg::Inherit<vsg::Operation, Merge> {
    Merge(const vsg::Path& in_path,
          vsg::observer_ptr<vsg::Viewer> in_viewer,
          vsg::ref_ptr<vsg::Group> in_attachmentPoint,
          vsg::ref_ptr<vsg::Node> in_node,
          const vsg::CompileResult& in_compileResult)
        : path(in_path),
          viewer(in_viewer),
          attachmentPoint(in_attachmentPoint),
          node(in_node),
          compileResult(in_compileResult) {}

    vsg::Path path;
    vsg::observer_ptr<vsg::Viewer> viewer;
    vsg::ref_ptr<vsg::Group> attachmentPoint;
    vsg::ref_ptr<vsg::Node> node;
    vsg::CompileResult compileResult;

    void run() override {
        // std::cout << "Merge::run() path = " << path << ", " << attachmentPoint << ", " << node << std::endl;

        vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
        if (ref_viewer) {
            updateViewer(*ref_viewer, compileResult);
        }

        attachmentPoint->addChild(node);
    }
};

struct LoadOperation : public vsg::Inherit<vsg::Operation, LoadOperation> {
    LoadOperation(vsg::ref_ptr<vsg::Viewer> in_viewer,
                  vsg::ref_ptr<vsg::Group> in_attachmentPoint,
                  const vsg::Path& in_filename,
                  vsg::ref_ptr<vsg::Options> in_options)
        : viewer(in_viewer), attachmentPoint(in_attachmentPoint), filename(in_filename), options(in_options) {}

    vsg::observer_ptr<vsg::Viewer> viewer;
    vsg::ref_ptr<vsg::Group> attachmentPoint;
    vsg::Path filename;
    vsg::ref_ptr<vsg::Options> options;

    void run() override {
        vsg::ref_ptr<vsg::Viewer> ref_viewer = viewer;
        if (auto node = vsg::read_cast<vsg::Node>(filename, options)) {
            auto result = ref_viewer->compileManager->compile(node);
            if (result)
                ref_viewer->addUpdateOperation(Merge::create(filename, viewer, attachmentPoint, node, result));
        }
    }
};

// =============================================================================

ChVisualSystemVSG::ChVisualSystemVSG(int num_divs)
    : m_show_logo(true),
      m_logo_pos({10, 10}),
      m_logo_height(64),
      m_yup(false),
      m_use_skybox(false),
      m_use_shadows(false),
      m_use_fullscreen(false),
      m_camera_trackball(true),
      m_capture_image(false),
      //
      m_show_gui(true),
      m_show_base_gui(true),
      m_show_visibility_controls(true),
      //
      m_show_body_objs(true),
      m_show_link_objs(true),
      m_show_spring_dampers(true),
      m_show_fea_meshes(true),
      //
      m_show_collision(false),
      m_collision_color(ChColor(0.9f, 0.4f, 0.2f)),
      m_collision_color_changed(false),
      //
      m_max_num_contacts(200),
      //
      m_show_contact_normals(false),
      m_contact_normals_color(ChColor(0.16f, 0.96f, 0.23f)),
      m_contact_normals_color_changed(false),
      m_contact_normals_scale(1),
      //
      m_show_contact_forces(false),
      m_contact_forces_color(ChColor(0.94f, 0.96f, 0.16f)),
      m_contact_forces_color_changed(false),
      m_contact_forces_scale(1),
      //
      m_show_abs_frame(false),
      m_show_ref_frames(false),
      m_show_com_frames(false),
      m_show_com_symbols(false),
      m_show_link_frames(false),
      m_abs_frame_scale(1),
      m_ref_frame_scale(1),
      m_com_frame_scale(1),
      m_com_symbol_ratio(0.15),
      m_link_frame_scale(1),
      m_com_size_changed(false),
      m_com_symbols_empty(false),
      //
      m_label_size(0.2),
      m_show_body_labels(false),
      m_show_link_labels(false),
      m_body_labels_color(ChColor(0.8f, 0.1f, 0.1f)),
      m_link_labels_color(ChColor(0.1f, 0.1f, 0.8f)),
      m_body_labels_scale(1),
      m_link_labels_scale(1),
      //
      m_frame_number(0),
      m_start_time(0),
      m_time_total(0),
      m_old_time(0),
      m_current_time(0),
      m_fps(0),
      m_target_render_fps(0),
      m_last_render_time(0) {
    m_windowTitle = std::string("Window Title");
    ////m_skyboxPath = std::string("vsg/textures/chrono_skybox.ktx2");
    m_skyboxPath = std::string("vsg/textures/vsg_skybox.ktx");
    m_labelFontPath = std::string("vsg/fonts/OpenSans-Bold.vsgb");
    m_cameraUpVector = vsg::dvec3(0, 0, 1);

    m_logo_filename = GetChronoDataFile("logo_chrono_alpha.png");

    // creation here allows to set entries before initialize
    m_objScene = vsg::Switch::create();
    m_pointpointScene = vsg::Switch::create();
    m_deformableScene = vsg::Switch::create();
    m_particleScene = vsg::Switch::create();
    m_collisionScene = vsg::Switch::create();
    m_contactNormalsScene = vsg::Switch::create();
    m_contactForcesScene = vsg::Switch::create();
    m_absFrameScene = vsg::Switch::create();
    m_refFrameScene = vsg::Switch::create();
    m_comFrameScene = vsg::Switch::create();
    m_comSymbolScene = vsg::Switch::create();
    m_bodyLabelScene = vsg::Switch::create();
    m_linkLabelScene = vsg::Switch::create();
    m_linkFrameScene = vsg::Switch::create();
    m_decoScene = vsg::Group::create();

    // set up defaults and read command line arguments to override them
    m_options = vsg::Options::create();
    m_options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    m_options->paths.push_back(GetChronoDataPath());

    m_options->setValue("image_format", vsg::CoordinateSpace::LINEAR);
    m_options->setValue("vertex_color_space", vsg::CoordinateSpace::sRGB);
    m_options->setValue("material_color_space", vsg::CoordinateSpace::LINEAR);

    // add vsgXchange's support for reading and writing 3rd party file formats, mandatory for chrono_vsg!
    m_options->add(vsgXchange::all::create());
    m_options->sharedObjects = vsg::SharedObjects::create();
    m_shapeBuilder = ShapeBuilder::create(m_options, num_divs);

    // vsg builder is used for particle visualization
    // for particles (spheres) we use phong shaders only
    m_vsgBuilder = vsg::Builder::create();

    {
        // for COM symbols (quads) we want to use flat shaders without Z-buffering
        // we setup a custom flat shader set
        auto flatShaderSet = vsg::createFlatShadedShaderSet();
        auto depthStencilState = vsg::DepthStencilState::create();
        depthStencilState->depthTestEnable = VK_FALSE;
        flatShaderSet->defaultGraphicsPipelineStates.push_back(depthStencilState);
        m_options->shaderSets["flat"] = flatShaderSet;
    }
    {
        // for COM labels (dynamic text) we want to use text shaders without Z-buffering
        auto shaderSet = m_options->shaderSets["text"] = vsg::createTextShaderSet(m_options);
        auto depthStencilState = vsg::DepthStencilState::create();
        depthStencilState->depthTestEnable = VK_FALSE;
        shaderSet->defaultGraphicsPipelineStates.push_back(depthStencilState);
    }
    m_vsgBuilder->options = m_options;

    // default settings
    SetWindowTitle("");
    SetWindowSize(ChVector2i(800, 600));
    SetWindowPosition(ChVector2i(50, 50));
    SetCameraAngleDeg(40);
    SetLightIntensity(1.0);
    SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
#ifdef __APPLE__
    SetGuiFontSize(20.0);
#else
    SetGuiFontSize(13.0);
#endif
}

ChVisualSystemVSG::~ChVisualSystemVSG() {}

void ChVisualSystemVSG::SetOutputScreen(int screenNum) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetOutputScreen must be used before initialization!" << std::endl;
        return;
    }
    int maxNum = vsg::Device::maxNumDevices();
    std::cout << "Screens found: " << maxNum << std::endl;
    if (screenNum >= 0 && screenNum < maxNum) {
        m_screen_num = screenNum;
    } else {
        std::cerr << "Screen #" << screenNum << " cannot be used on this computer!" << std::endl;
        throw std::runtime_error("Screen #" + std::to_string(screenNum) + " cannot be used on this computer!");
    }
}

void ChVisualSystemVSG::EnableFullscreen(bool val) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::EnableFullscreen must be used before initialization!" << std::endl;
        return;
    }
    m_use_fullscreen = val;
}

size_t ChVisualSystemVSG::AddGuiComponent(std::shared_ptr<ChGuiComponentVSG> gc) {
    gc->m_vsys = this;
    m_gui.push_back(gc);
    return m_gui.size() - 1;
}

size_t ChVisualSystemVSG::AddGuiColorbar(const std::string& title,
                                         const ChVector2d& range,
                                         ChColormap::Type type,
                                         bool bimodal,
                                         float width) {
    if (m_initialized) {
        std::cout << "Error: Attempt to create a VSG colorbar after initialization of the VSG visual system."
                  << std::endl;

        throw std::runtime_error("Attempt to create a VSG colorbar after initialization of the VSG visual system");
    }

    auto gc = chrono_types::make_shared<ChColorbarGuiComponentVSG>(title, range, type, bimodal, width);
    gc->m_vsys = this;
    m_gui.push_back(gc);
    return m_gui.size() - 1;
}

std::shared_ptr<ChGuiComponentVSG> ChVisualSystemVSG::GetGuiComponent(size_t id) {
    return m_gui.at(id);
}

void ChVisualSystemVSG::SetBaseGuiVisibility(bool show_gui) {
    m_show_base_gui = show_gui;
    if (m_initialized)
        m_base_gui->SetVisibility(m_show_base_gui);
}

void ChVisualSystemVSG::ToggleBaseGuiVisibility() {
    m_show_base_gui = !m_show_base_gui;
    if (m_initialized)
        m_base_gui->SetVisibility(m_show_base_gui);
}

void ChVisualSystemVSG::AddEventHandler(std::shared_ptr<ChEventHandlerVSG> eh) {
    m_evhandler.push_back(eh);
}

void ChVisualSystemVSG::AddComputeCommands(vsg::ref_ptr<vsg::Commands> commands) {
    if (!commands || !m_computeCommandGraph || !m_viewer)
        return;

    // Schedule plugin compute work on the dedicated command graph so it executes before the render graph
    m_computeCommandGraph->addChild(commands);

    // Compile the incoming commands immediately to match current pipeline setup
    auto compileTraversal = vsg::CompileTraversal::create(*m_viewer);
    commands->accept(*compileTraversal);
}

void ChVisualSystemVSG::AttachPlugin(std::shared_ptr<ChVisualSystemVSGPlugin> plugin) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::AttachPlugin can only be called before initialization!" << std::endl;
        return;
    }
    plugin->m_vsys = this;
    plugin->OnAttach();
    m_plugins.push_back(plugin);
}

void ChVisualSystemVSG::Quit() {
    m_viewer->close();
}

void ChVisualSystemVSG::SetGuiFontSize(float theSize) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetGuiFontSize can only be called before initialization!"
                  << std::endl;
        return;
    }
    m_guiFontSize = theSize;
}

void ChVisualSystemVSG::SetWindowSize(const ChVector2i& size) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowSize can only be called before initialization!" << std::endl;
        return;
    }
    m_windowWidth = size[0];
    m_windowHeight = size[1];
}

void ChVisualSystemVSG::SetWindowSize(int width, int height) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowSize can only be called before initialization!" << std::endl;
        return;
    }
    m_windowWidth = width;
    m_windowHeight = height;
}

void ChVisualSystemVSG::SetWindowPosition(const ChVector2i& pos) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowPosition can only be called before initialization!"
                  << std::endl;
        return;
    }
    m_windowX = pos[0];
    m_windowY = pos[1];
}

void ChVisualSystemVSG::SetWindowPosition(int from_left, int from_top) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowPosition can only be called before initialization!"
                  << std::endl;
        return;
    }
    m_windowX = from_left;
    m_windowY = from_top;
}

void ChVisualSystemVSG::SetWindowTitle(const std::string& title) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetWindowTitle can only be called before initialization!"
                  << std::endl;
        return;
    }
    m_windowTitle = title;
}

void ChVisualSystemVSG::EnableSkyBox(bool val) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::EnableSkyBox can only be called before initialization!" << std::endl;
        return;
    }
    m_use_skybox = val;
}

void ChVisualSystemVSG::SetSkyBoxTexture(const std::string& filename) {
    m_skyboxPath = filename;
}

int ChVisualSystemVSG::AddCamera(const ChVector3d& pos, ChVector3d targ) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::AddCamera can only be called before initialization!" << std::endl;
        return 1;
    }

    ChVector3d test = pos - targ;
    if (test.Length() == 0.0) {
        std::cerr << "Function ChVisualSystemVSG::AddCamera Camera Pos and Target cannot be identical!" << std::endl;
        std::cerr << "  pos    = { " << pos.x() << " ; " << pos.y() << " ; " << pos.z() << " }" << std::endl;
        std::cerr << "  target = { " << targ.x() << " ; " << targ.y() << " ; " << targ.z() << " }" << std::endl;
        throw std::runtime_error("Function ChVisualSystemVSG::AddCamera Camera Pos and Target cannot be identical!");
    }
    if (m_yup) {
        if (pos.x() == 0.0 && pos.z() == 0.0) {
            std::cout << "Function ChVisualSystemVSG::AddCamera Line of sight is parallel to upvector! -> Corrected!!"
                      << std::endl;
            m_vsg_cameraEye = vsg::dvec3(pos.x() + 1.0, pos.y(), pos.z() + 1.0);
        } else {
            m_vsg_cameraEye = vsg::dvec3(pos.x(), pos.y(), pos.z());
        }
    } else {
        if (pos.x() == 0.0 && pos.y() == 0.0) {
            std::cout << "Function ChVisualSystemVSG::AddCamera Line of sight is parallel to upvector! -> Corrected!!"
                      << std::endl;
            m_vsg_cameraEye = vsg::dvec3(pos.x() + 1.0, pos.y() + 1.0, pos.z());
        } else {
            m_vsg_cameraEye = vsg::dvec3(pos.x(), pos.y(), pos.z());
        }
    }
    m_vsg_cameraTarget = vsg::dvec3(targ.x(), targ.y(), targ.z());

    return 0;
}

void ChVisualSystemVSG::SetCameraPosition(int id, const ChVector3d& pos) {
    m_lookAt->eye = vsg::dvec3(pos.x(), pos.y(), pos.z());
}

void ChVisualSystemVSG::SetCameraTarget(int id, const ChVector3d& target) {
    m_lookAt->center = vsg::dvec3(target.x(), target.y(), target.z());
}

void ChVisualSystemVSG::SetCameraPosition(const ChVector3d& pos) {
    m_lookAt->eye = vsg::dvec3(pos.x(), pos.y(), pos.z());
}

void ChVisualSystemVSG::SetCameraTarget(const ChVector3d& target) {
    m_lookAt->center = vsg::dvec3(target.x(), target.y(), target.z());
}

ChVector3d ChVisualSystemVSG::GetCameraPosition() const {
    const auto& p = m_lookAt->eye;
    return ChVector3d(p.x, p.y, p.z);
}

ChVector3d ChVisualSystemVSG::GetCameraTarget() const {
    const auto& p = m_lookAt->center;
    return ChVector3d(p.x, p.y, p.z);
}

void ChVisualSystemVSG::SetCameraVertical(CameraVerticalDir upDir) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetCameraVertical can only be called before initialization!"
                  << std::endl;
        return;
    }
    switch (upDir) {
        case CameraVerticalDir::Y:
            m_cameraUpVector = vsg::dvec3(0, 1, 0);
            m_yup = true;
            break;
        case CameraVerticalDir::Z:
            m_cameraUpVector = vsg::dvec3(0, 0, 1);
            m_yup = false;
            break;
    }
}

void ChVisualSystemVSG::SetLightIntensity(float intensity) {
    m_lightIntensity = ChClamp(intensity, 0.0f, 1.0f);
}

void ChVisualSystemVSG::SetLightDirection(double azimuth, double elevation) {
    if (m_initialized) {
        std::cerr << "Function ChVisualSystemVSG::SetLightDirection can only be called before initialization!"
                  << std::endl;
        return;
    }
    m_azimuth = ChClamp(azimuth, -CH_PI, CH_PI);
    m_elevation = ChClamp(elevation, 0.0, CH_PI_2);
}

void ChVisualSystemVSG::Initialize() {
    if (m_initialized)
        return;

    // Let any plugins perform pre-initialization operations
    for (auto& plugin : m_plugins)
        plugin->OnInitialize();

    auto builder = vsg::Builder::create();
    builder->options = m_options;

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = m_windowTitle;
    windowTraits->width = m_windowWidth;
    windowTraits->height = m_windowHeight;
    windowTraits->x = m_windowX;
    windowTraits->y = m_windowY;
    windowTraits->debugLayer = false;
    windowTraits->deviceExtensionNames = {VK_KHR_MULTIVIEW_EXTENSION_NAME, VK_KHR_MAINTENANCE2_EXTENSION_NAME,
                                          VK_KHR_CREATE_RENDERPASS_2_EXTENSION_NAME,
                                          VK_KHR_DEPTH_STENCIL_RESOLVE_EXTENSION_NAME};
    windowTraits->swapchainPreferences.imageUsage =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
    windowTraits->depthImageUsage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    windowTraits->fullscreen = m_use_fullscreen;
    windowTraits->screenNum = m_screen_num;

    m_labelFont = vsg::read_cast<vsg::Font>(m_labelFontPath, m_options);
    if (!m_labelFont) {
        std::cout << "Failed to read font : " << m_labelFontPath << std::endl;
        return;
    }

    m_scene = vsg::Group::create();

    double radius = 50.0;
    vsg::dbox bound;

    if (m_use_skybox) {
        vsg::Path fileName(m_skyboxPath);
        auto skyPtr = createSkybox(fileName, m_options, m_yup);
        if (skyPtr)
            m_scene->addChild(skyPtr);
        else
            m_use_skybox = false;
    }

    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0f, 1.0f, 1.0f);
    ambientLight->intensity = 0.1f * m_lightIntensity;  // before sRGB

    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "sun light";
    directionalLight->color.set(1.0f, 1.0f, 1.0f);
    directionalLight->intensity = vsg::linear_to_sRGB(m_lightIntensity);
    if (m_use_shadows) {
        uint32_t numShadowsPerLight = 10;
        auto shadowSettings = vsg::HardShadows::create(numShadowsPerLight);
        directionalLight->shadowSettings = shadowSettings;
        directionalLight->intensity = 0.8f * m_lightIntensity;  // try to avoid saturation due to additional lights
    }

    double se = std::sin(m_elevation);
    double ce = std::cos(m_elevation);
    double sa = std::sin(m_azimuth);
    double ca = std::cos(m_azimuth);
    if (m_yup)
        directionalLight->direction.set(-ce * ca, -se, -ce * sa);
    else
        directionalLight->direction.set(-ce * ca, -ce * sa, -se);

    /* Head Light, moves with camera orientation
    auto absoluteTransform = vsg::AbsoluteTransform::create();
    absoluteTransform->addChild(ambientLight);
    absoluteTransform->addChild(directionalLight);
    m_scene->addChild(absoluteTransform);
     */
    // Directional (Sun) Light, moves with object orientation
    m_scene->addChild(ambientLight);
    m_scene->addChild(directionalLight);
    if (m_use_shadows) {
        // helper light to improve quality of the dark side
        auto overheadLight = vsg::DirectionalLight::create();
        overheadLight->name = "head light";
        overheadLight->color.set(1.0f, 1.0f, 1.0f);
        overheadLight->intensity = 0.2f * m_lightIntensity;
        if (m_yup)
            overheadLight->direction.set(-ce * ca, -se, -ce * sa);
        else
            overheadLight->direction.set(-ce * ca, -ce * sa, -se);
        auto absoluteTransform = vsg::AbsoluteTransform::create();
        absoluteTransform->addChild(overheadLight);
        m_scene->addChild(absoluteTransform);
    }
    m_scene->addChild(m_objScene);
    m_scene->addChild(m_pointpointScene);
    m_scene->addChild(m_particleScene);
    m_scene->addChild(m_deformableScene);
    m_scene->addChild(m_collisionScene);
    m_scene->addChild(m_contactNormalsScene);
    m_scene->addChild(m_contactForcesScene);
    m_scene->addChild(m_absFrameScene);
    m_scene->addChild(m_refFrameScene);
    m_scene->addChild(m_comFrameScene);
    m_scene->addChild(m_comSymbolScene);
    m_scene->addChild(m_bodyLabelScene);
    m_scene->addChild(m_linkLabelScene);
    m_scene->addChild(m_linkFrameScene);
    m_scene->addChild(m_decoScene);

    // Let any plugins perform pre-bind operations
    for (auto& plugin : m_plugins)
        plugin->OnBindAssets();

    BindAll();
    CreateContacts();

    // create the viewer and assign window(s) to it
    m_viewer = vsg::Viewer::create();

    m_window = vsg::Window::create(windowTraits);
    if (!m_window) {
        std::cout << "Could not create window." << std::endl;
        return;
    }

    ////auto& limits = m_window->getOrCreatePhysicalDevice()->getProperties().limits;  // VkPhysicalDeviceLimits
    const auto& prop = m_window->getOrCreatePhysicalDevice()->getProperties();

    if (m_verbose) {
        std::cout << "----------------------------------------------------" << std::endl;
        std::cout << "* Chrono::VSG Vulkan Scene Graph 3D-Visualization" << std::endl;
        std::cout << "* GPU Name: " << prop.deviceName << std::endl;
        switch (prop.deviceType) {
            default:
            case VK_PHYSICAL_DEVICE_TYPE_OTHER:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_OTHER" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU" << std::endl;
                break;
            case VK_PHYSICAL_DEVICE_TYPE_CPU:
                std::cout << "* GPU Type: VK_PHYSICAL_DEVICE_TYPE_CPU" << std::endl;
                break;
        }
        std::cout << "* Vulkan Version: " << VK_VERSION_MAJOR(VK_HEADER_VERSION_COMPLETE) << "."
                  << VK_VERSION_MINOR(VK_HEADER_VERSION_COMPLETE) << "."
                  << VK_API_VERSION_PATCH(VK_HEADER_VERSION_COMPLETE) << std::endl;
        std::cout << "* Vulkan Scene Graph Version: " << VSG_VERSION_STRING << std::endl;
        std::cout << "* Graphic Output Possible on: " << vsg::Device::maxNumDevices() << " Screens." << std::endl;
        std::cout << "----------------------------------------------------" << std::endl;
    }

    m_window->clearColor() = vsg::vec4CH(m_background_color, 1.0f);
    m_viewer->addWindow(m_window);

    // set up the camera
    m_lookAt = vsg::LookAt::create(m_vsg_cameraEye, m_vsg_cameraTarget, m_cameraUpVector);

    double nearFarRatio = 0.001;
    auto perspective = vsg::Perspective::create(
        m_cameraAngleDeg,
        static_cast<double>(m_window->extent2D().width) / static_cast<double>(m_window->extent2D().height),
        nearFarRatio * radius, radius * 10.0);

    m_vsg_camera = vsg::Camera::create(perspective, m_lookAt, vsg::ViewportState::create(m_window->extent2D()));

    // default sets automatic directional light
    // auto renderGraph = vsg::RenderGraph::create(m_window, m_view);
    // switches off automatic directional light setting

    m_renderGraph =
        vsg::createRenderGraphForView(m_window, m_vsg_camera, m_scene, VK_SUBPASS_CONTENTS_INLINE, false);
    // extend for seperate render and compute graphs
    m_renderCommandGraph = vsg::CommandGraph::create(m_window, m_renderGraph);
    m_computeCommandGraph = vsg::CommandGraph::create(m_window);

    // initialize ImGui
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = "../data/vsg/imgui.ini";

#ifdef __APPLE__
    // application runs on retina display by default (window 800*600 generates a viewport 1600*1200)
    // there can be reasons to switch to standard resolution
    // 1) there is no retina display
    // 2) standard resolution is demanded by MacOS (in our Irrlicht/VSG examples), set in the app bundle
    // in this case the desired font size is too big. We take the standard font instead.
    if (m_window->traits()->width != m_window->extent2D().width) {
#endif
        auto foundFontFile = vsg::findFile("vsg/fonts/Ubuntu_Mono/UbuntuMono-Regular.ttf", m_options);
        if (foundFontFile) {
            // convert native filename to UTF8 string that is compatible with ImuGUi.
            std::string c_fontFile = foundFontFile.string();

            // read the font via ImGui, which will then be current when vsgImGui::RenderImGui initializes the rest of
            // ImGui/Vulkan below
            ImGuiIO& io = ImGui::GetIO();
            auto imguiFont = io.Fonts->AddFontFromFileTTF(c_fontFile.c_str(), m_guiFontSize);
            if (!imguiFont) {
                std::cout << "Failed to load font: " << c_fontFile << std::endl;
                return;
            }
        }
#ifdef __APPLE__
    } else {
        m_logo_height /= 2.0f;
        // ignore loadable ttf font
        std::cout << "App runs with standard resolution on the Mac. Font size setting ignored." << std::endl;
    }
#endif

    auto renderImGui = vsgImGui::RenderImGui::create(m_window, ChMainGuiVSG::create(this, m_options, m_logo_height));
    m_renderGraph->addChild(renderImGui);

    // Use the ImGui dark (default) style, with adjusted transparency
    ImGui::StyleColorsDark();
    auto& style = ImGui::GetStyle();
    ImVec4 bg_color = style.Colors[ImGuiCol_WindowBg];
    bg_color.w = 0.75f;
    style.Colors[ImGuiCol_WindowBg] = bg_color;
    style.Colors[ImGuiCol_ChildBg] = bg_color;
    style.Colors[ImGuiCol_TitleBg] = bg_color;

    // Create main GUI window
    m_base_gui = chrono_types::make_shared<ChBaseGuiComponentVSG>(this);
    m_base_gui->SetVisibility(m_show_base_gui);
    AddGuiComponent(m_base_gui);

    // Add the camera info GUI component (initially invisible)
    m_camera_gui = AddGuiComponent(chrono_types::make_shared<ChCameraGuiComponentVSG>(this));

    // ImGui events shall have priority to other events
    m_viewer->addEventHandler(vsgImGui::SendEventsToImGui::create());

    // Add the base keyboard event handler
    auto base_kbhandler = chrono_types::make_shared<ChBaseEventHandlerVSG>(this);
    auto base_kbhandler_wrapper = EventHandlerWrapper::create(base_kbhandler, this);

    m_viewer->addEventHandler(base_kbhandler_wrapper);

    // Add all user-specified event handlers
    for (const auto& eh : m_evhandler) {
        auto evhandler_wrapper = EventHandlerWrapper::create(eh, this);
        m_viewer->addEventHandler(evhandler_wrapper);
    }

    // Let any plugins add their event handlers
    for (auto& plugin : m_plugins) {
        for (const auto& eh : plugin->m_evhandler) {
            auto evhandler_wrapper = EventHandlerWrapper::create(eh, this);
        }
    }

    // Add event handler for window close events
    m_viewer->addEventHandler(vsg::CloseHandler::create(m_viewer));

    // Add event handler for mouse camera view manipulation
    if (m_camera_trackball)
        m_viewer->addEventHandler(vsg::Trackball::create(m_vsg_camera));

    // assign both compute and render command graphs to the viewer
    m_viewer->assignRecordAndSubmitTaskAndPresentation({m_computeCommandGraph, m_renderCommandGraph});

    // Assign a CompileTraversal to the Builders that will compile for all the views assigned to the viewer.
    // Must be done after Viewer.assignRecordAndSubmitTasksAndPresentations()
    auto compileTraversal = vsg::CompileTraversal::create(*m_viewer);
    m_shapeBuilder->assignCompileTraversal(compileTraversal);
    m_vsgBuilder->assignCompileTraversal(compileTraversal);
    vsg::ref_ptr<vsg::ResourceHints> resourceHints;
    if (!resourceHints) {
        // To help reduce the number of vsg::DescriptorPool that need to be allocated we'll provide a minimum
        // requirement via ResourceHints.
        resourceHints = vsg::ResourceHints::create();
        resourceHints->numDescriptorSets = 256;
        resourceHints->descriptorPoolSizes.push_back(
            VkDescriptorPoolSize{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 256});
    }

    m_viewer->compile(resourceHints);

    // Create contact creation handler
    m_contact_creator = chrono_types::make_shared<CreateContactsVSG>(this);

    // Prepare reading 3d files
    m_loadThreads = vsg::OperationThreads::create(m_numThreads, m_viewer->status);

    // Let all GUI components initialize themselves
    for (auto& gui : m_gui)
        gui->Initialize();

    m_initialized = true;
}

bool ChVisualSystemVSG::Run() {
    return m_viewer->active();
}

void ChVisualSystemVSG::Render() {
    // Frame rate limiting - decouples the simulation time step from the rendering frame rate and reduces cpu overhead
    // if set to 0, no frame rate limiting is applied
    if (m_target_render_fps > 0 && m_frame_number > 0) {
        m_timer_render.stop();
        double elapsed = m_timer_render();
        double min_frame_time = 1.0 / m_target_render_fps;
        
        if (elapsed < min_frame_time) {
            // Not enough wall-clock time passed - skip this frame entirely
            m_timer_render.start();
            ChVisualSystem::Render();
            return;
        }
        // Frame gate passed - timer already stopped, will be reset/started at bottom
    }
    
    if (m_write_images && m_frame_number > 0) {
        // Zero-pad frame numbers in file names for postprocessing
        std::ostringstream filename;
        filename << m_image_dir << "/img_" << std::setw(5) << std::setfill('0') << m_frame_number << ".png";
        WriteImageToFile(filename.str());
    }    
    
    if (m_frame_number == 0)
        m_start_time = double(clock()) / double(CLOCKS_PER_SEC);

    // Let any plugins perform pre-rendering operations
    for (auto& plugin : m_plugins)
        plugin->OnRender();

    Update();

    if (!m_viewer->advanceToNextFrame()) {
        return;
    }

    // Let the viewer handle any events
    m_viewer->handleEvents();

    m_viewer->update();

    // Dynamic data transfer CPU->GPU for COM symbol size and body labels
    // Only update if COM symbols are actually visible to avoid unecessary cpu to gpu data transfers
    // otherwise this is effectively marking dirty even if the symbols are hidden! (extra work)
    if (m_show_com_symbols && !m_com_symbols_empty) {
        auto symbol_size = m_com_frame_scale * m_com_symbol_ratio;

        std::vector<ChVector3d> c_pos;
        for (auto sys : m_systems)
            CollectActiveBodyCOMPositions(sys->GetAssembly(), c_pos);
        assert(!c_pos.empty());

        ConvertPositions(c_pos, m_com_symbol_positions, symbol_size);
        m_com_symbol_positions->dirty();

        if (m_com_size_changed) {
            m_com_symbol_vertices->set(0, vsg::vec3(-symbol_size / 2, -symbol_size / 2, 0));
            m_com_symbol_vertices->set(1, vsg::vec3(+symbol_size / 2, -symbol_size / 2, 0));
            m_com_symbol_vertices->set(2, vsg::vec3(+symbol_size / 2, +symbol_size / 2, 0));
            m_com_symbol_vertices->set(3, vsg::vec3(-symbol_size / 2, +symbol_size / 2, 0));
            m_com_symbol_vertices->dirty();
            m_com_size_changed = false;
        }

        assert(!m_body_labels.empty());
        auto label_size = m_body_labels_scale * m_label_size;

        for (size_t iPos = 0; iPos < c_pos.size(); iPos++) {
            m_body_labels_layout[iPos]->horizontal = vsg::vec3(label_size, 0, 0);
            m_body_labels_layout[iPos]->vertical = vsg::vec3(0, label_size, 0);
            m_body_labels_layout[iPos]->position =
                vsg::vec3(c_pos[iPos].x(), c_pos[iPos].y() - label_size / 2, c_pos[iPos].z());
            m_body_labels_layout[iPos]->color = vsg::vec4CH(m_body_labels_color, 1.0f);
            m_body_labels_text[iPos]->setup(0, m_options);
        }
    }

    // Dynamic data transfer CPU->GPU for link labels  
    // Only update if link labels are actually visible to avoid expensive CPU work - the !m_link_labels_empty test doesn't condition this
    // code comprehensively. If the link labels are empty, the for ipos won't run, but the CollectLinkFramePositions will still be called needlessly.
    if (m_show_link_labels) {
        auto label_size = m_link_labels_scale * m_label_size;

        std::vector<ChVector3d> c_pos;
        for (auto sys : m_systems)
            CollectLinkFramePositions(sys->GetAssembly(), c_pos);
        assert(!c_pos.empty());

        for (size_t iPos = 0; iPos < c_pos.size(); iPos++) {
            m_link_labels_layout[iPos]->horizontal = vsg::vec3(label_size, 0, 0);
            m_link_labels_layout[iPos]->vertical = vsg::vec3(0, label_size, 0);
            m_link_labels_layout[iPos]->position =
                vsg::vec3(c_pos[iPos].x(), c_pos[iPos].y() - label_size / 2, c_pos[iPos].z());
            m_link_labels_layout[iPos]->color = vsg::vec4CH(m_link_labels_color, 1.0f);
            m_link_labels_text[iPos]->setup(0, m_options);
        }
    }

    // Dynamic data transfer CPU->GPU for line models
    if (m_collision_color_changed) {
        for (const auto& colors : m_collision_colors) {
            for (auto& c : *colors)
                c = vsg::vec4CH(m_collision_color);
            colors->dirty();
        }
        m_collision_color_changed = false;
    }

    // Dynamic data transfer for contact colors
    if (m_contact_normals_color_changed) {
        for (const auto& colors : m_contact_normals_colors) {
            for (auto& c : *colors)
                c = vsg::vec3CH(m_contact_normals_color);
            colors->dirty();
        }
        m_contact_normals_color_changed = false;
    }

    if (m_contact_forces_color_changed) {
        for (const auto& colors : m_contact_forces_colors) {
            for (auto& c : *colors)
                c = vsg::vec3CH(m_contact_forces_color);
            colors->dirty();
        }
        m_contact_forces_color_changed = false;
    }

    // Dynamic data transfer CPU->GPU for point clouds
    // use direct pointer access to avoid temporary object construction
    // Dynamic colours are handled by the vulkan compute shader
    if (!m_clouds.empty()) {
        auto hide_pos = m_lookAt->eye - (m_lookAt->center - m_lookAt->eye) * 0.1;
        for (const auto& cloud : m_clouds) {
            if (cloud.dynamic_positions) {
                // Write particle positions in bulk via raw pointers to avoid per-element temporary objects
                const size_t count = cloud.positions->size();
                auto* pos_data = cloud.positions->data();

                for (size_t k = 0; k < count; ++k) {
                    if (cloud.pcloud->IsVisible(static_cast<unsigned int>(k))) {
                        const auto& src = cloud.pcloud->GetParticlePos(static_cast<unsigned int>(k));
                        pos_data[k].set(static_cast<float>(src.x()), static_cast<float>(src.y()),
                                        static_cast<float>(src.z()));
                    } else {
                        pos_data[k] = hide_pos;
                    }
                }
                cloud.positions->dirty();
            }
        }
    }


    // Dynamic data transfer CPU->GPU for deformable meshes
    // To speed up cpu-gpu - treat ChVector3d arrays as contiguous arrays for bulk conversion over to gpu
    for (auto& def_mesh : m_def_meshes) {
        if (def_mesh.dynamic_vertices) {
            const auto& new_vertices =
                def_mesh.mesh_soup ? def_mesh.trimesh->getFaceVertices() : def_mesh.trimesh->GetCoordsVertices();
            assert(def_mesh.vertices->size() == new_vertices.size());

            const size_t count = new_vertices.size();
            if (count > 0) {
                // ChVector3d stores 3 doubles contiguously, cast to raw double* and float* with less overhead
                const double* src_ptr = new_vertices[0].data();
                float* dst_ptr = reinterpret_cast<float*>(def_mesh.vertices->data());

                // convert 3*count doubles to floats with tight loop
                const size_t total_components = count * 3;
                for (size_t i = 0; i < total_components; ++i) {
                    dst_ptr[i] = static_cast<float>(src_ptr[i]);
                }

                def_mesh.vertices->dirty();
            }
        }

        if (def_mesh.dynamic_normals) {
            const auto& new_normals =
                def_mesh.mesh_soup ? def_mesh.trimesh->getFaceNormals() : def_mesh.trimesh->getAverageNormals();
            assert(def_mesh.normals->size() == new_normals.size());

            const size_t count = new_normals.size();
            if (count > 0) {
                const double* src_ptr = new_normals[0].data();
                float* dst_ptr = reinterpret_cast<float*>(def_mesh.normals->data());

                const size_t total_components = count * 3;
                for (size_t i = 0; i < total_components; ++i) {
                    dst_ptr[i] = static_cast<float>(src_ptr[i]);
                }

                def_mesh.normals->dirty();
            }
        }

        // TODO: - could be converted to the VSG compute shader which particles use, but would only benefit with
        // large vertice mesh when this loop is significant compared to the rest of the frame time
        if (def_mesh.dynamic_colors) {
            const auto& new_colors =
                def_mesh.mesh_soup ? def_mesh.trimesh->getFaceColors() : def_mesh.trimesh->GetCoordsColors();
            assert(def_mesh.colors->size() == new_colors.size());

            const size_t count = new_colors.size();
            if (count > 0) {
                // ChColor is 12 bytes (3 floats), but need to give to the gpu with vec4 (16 bytes) for alignment
                // copy element-wise with manual unroll
                const ChColor* src_ptr = new_colors.data();
                float* dst_ptr = reinterpret_cast<float*>(def_mesh.colors->data());

                // Manual unroll (RGBA = 4 components)
                for (size_t k = 0; k < count; ++k) {
                    const size_t idx = k * 4;
                    dst_ptr[idx + 0] = src_ptr[k].R;
                    dst_ptr[idx + 1] = src_ptr[k].G;
                    dst_ptr[idx + 2] = src_ptr[k].B;
                    dst_ptr[idx + 3] = 1.0f;  // Alpha channel (ChColor has no transparency)
                }
                def_mesh.colors->dirty();
            }
        }
    }


    m_viewer->recordAndSubmit();

    if (m_capture_image) {
        ExportScreenImage();
        m_capture_image = false;
    }

    m_viewer->present();
    m_frame_number++;

    if (m_frame_number > 1) {
        m_timer_render.stop();
        m_time_total = .5 * m_timer_render() + .5 * m_time_total;
        m_current_time = m_time_total;
        m_current_time = m_current_time * 0.5 + m_old_time * 0.5;
        m_old_time = m_current_time;
        m_fps = 1.0 / m_current_time;
    }

    ChVisualSystem::Render();

    m_timer_render.reset();
    m_timer_render.start();

}

void ChVisualSystemVSG::SetBodyObjVisibility(bool vis, int tag) {
    if (!m_initialized)
        return;

    for (auto& child : m_objScene->children) {
        ObjectType type;
        int c_tag;
        child.node->getValue("Type", type);
        child.node->getValue("Tag", c_tag);
        if (type == ObjectType::BODY && (c_tag == tag || tag == -1))
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetLinkObjVisibility(bool vis, int tag) {
    if (!m_initialized)
        return;

    for (auto& child : m_objScene->children) {
        ObjectType type;
        int c_tag;
        child.node->getValue("Type", type);
        child.node->getValue("Tag", c_tag);
        if (type == ObjectType::LINK && (c_tag == tag || tag == -1))
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetFeaMeshVisibility(bool vis, int tag) {
    if (!m_initialized)
        return;

    for (auto& child : m_deformableScene->children) {
        DeformableType type;
        int c_tag;
        child.node->getValue("Type", type);
        child.node->getValue("Tag", c_tag);
        if (type == DeformableType::FEA && (c_tag == tag || tag == -1))
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetSpringVisibility(bool vis, int tag) {
    if (!m_initialized)
        return;

    for (auto& child : m_pointpointScene->children) {
        PointPointType type;
        int c_tag;
        child.node->getValue("Type", type);
        child.node->getValue("Tag", c_tag);
        if (type == PointPointType::SPRING && (c_tag == tag || tag == -1))
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetSegmentVisibility(bool vis, int tag) {
    if (!m_initialized)
        return;

    for (auto& child : m_pointpointScene->children) {
        PointPointType type;
        int c_tag;
        child.node->getValue("Type", type);
        child.node->getValue("Tag", c_tag);
        if (type == PointPointType::SEGMENT && (c_tag == tag || tag == -1))
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetParticleCloudVisibility(bool vis, int tag) {
    // Remember requested visibility even before the scene graph is constructed so late clouds inherit it
    // otherwise it causees issues with clouds added after initialisation
    if (tag == -1) {
        m_default_cloud_visibility = vis;
        m_cloud_visibility_overrides.clear();
    } else {
        m_cloud_visibility_overrides[tag] = vis;
    }

    if (!m_initialized)
        return;

    for (auto& child : m_particleScene->children) {
        int c_tag;
        if (!child.node->getValue("Tag", c_tag))
            continue;

        if (tag == -1) {
            // Apply the stored preference to every cloud when toggling the global state
            child.mask = GetDesiredCloudVisibility(c_tag);
        } else if (c_tag == tag) {
            child.mask = vis;
        }
    }
}

bool ChVisualSystemVSG::GetDesiredCloudVisibility(int tag) const {
    // Resolve the desired visibility using the tag override, falling back to global default
    auto it = m_cloud_visibility_overrides.find(tag);
    if (it != m_cloud_visibility_overrides.end())
        return it->second;
    return m_default_cloud_visibility;
}

void ChVisualSystemVSG::SetCollisionVisibility(bool vis, int tag) {
    m_show_collision = vis;
    
    if (!m_initialized)
        return;

    for (auto& child : m_collisionScene->children) {
        int c_tag;
        child.node->getValue("Tag", c_tag);
        if (c_tag == tag || tag == -1)
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetCollisionColor(const ChColor& color) {
    m_collision_color = color;

    if (m_initialized)
        m_collision_color_changed = true;
}

void ChVisualSystemVSG::SetContactNormalsVisibility(bool vis, int tag) {
    m_show_contact_normals = vis;

    if (!m_initialized)
        return;

    for (auto& child : m_contactNormalsScene->children) {
        int c_tag;
        child.node->getValue("Tag", c_tag);
        if (c_tag == tag || tag == -1)
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetContactNormalsColor(const ChColor& color) {
    m_contact_normals_color = color;

    if (m_initialized)
        m_contact_normals_color_changed = true;
}

void ChVisualSystemVSG::SetContactNormalsScale(double length) {
    m_contact_normals_scale = length;
}

void ChVisualSystemVSG::SetContactForcesVisibility(bool vis, int tag) {
    m_show_contact_forces = vis;

    if (!m_initialized)
        return;

    for (auto& child : m_contactForcesScene->children) {
        int c_tag;
        child.node->getValue("Tag", c_tag);
        if (c_tag == tag || tag == -1)
            child.mask = vis;
    }
}

void ChVisualSystemVSG::SetContactForcesColor(const ChColor& color) {
    m_contact_forces_color = color;

    if (m_initialized)
        m_contact_forces_color_changed = true;
}

void ChVisualSystemVSG::SetContactForcesScale(double length) {
    m_contact_forces_scale = length;
}

void ChVisualSystemVSG::SetAbsFrameScale(double axis_length) {
    m_abs_frame_scale = axis_length;
}

void ChVisualSystemVSG::ToggleAbsFrameVisibility() {
    m_show_abs_frame = !m_show_abs_frame;

    if (m_initialized) {
        for (auto& child : m_absFrameScene->children)
            child.mask = m_show_abs_frame;
    }
}

void ChVisualSystemVSG::RenderRefFrames(double axis_length) {
    m_ref_frame_scale = axis_length;
    m_show_ref_frames = true;

    if (m_initialized) {
        for (auto& child : m_refFrameScene->children)
            child.mask = m_show_ref_frames;
    }
}

void ChVisualSystemVSG::SetRefFrameScale(double axis_length) {
    m_ref_frame_scale = axis_length;
}

void ChVisualSystemVSG::ToggleRefFrameVisibility() {
    m_show_ref_frames = !m_show_ref_frames;

    if (m_initialized) {
        for (auto& child : m_refFrameScene->children)
            child.mask = m_show_ref_frames;
    }
}

void ChVisualSystemVSG::SetCOMFrameScale(double axis_length) {
    m_com_frame_scale = axis_length;
}

void ChVisualSystemVSG::ToggleCOMFrameVisibility() {
    m_show_com_frames = !m_show_com_frames;

    if (m_initialized) {
        for (auto& child : m_comFrameScene->children)
            child.mask = m_show_com_frames;
    }
}

void ChVisualSystemVSG::ToggleCOMSymbolVisibility() {
    m_show_com_symbols = !m_show_com_symbols;
    m_comSymbolScene->setAllChildren(m_show_com_symbols);
}

void ChVisualSystemVSG::SetLinkFrameScale(double axis_length) {
    m_link_frame_scale = axis_length;
}

void ChVisualSystemVSG::ToggleLinkFrameVisibility() {
    m_show_link_frames = !m_show_link_frames;

    if (m_initialized) {
        for (auto& child : m_linkFrameScene->children)
            child.mask = m_show_link_frames;
    }
}

void ChVisualSystemVSG::ToggleBodyLabelVisibility() {
    m_show_body_labels = !m_show_body_labels;
    m_bodyLabelScene->setAllChildren(m_show_body_labels);
}

void ChVisualSystemVSG::SetBodyLabelsColor(const ChColor& color) {
    m_body_labels_color = color;
}

void ChVisualSystemVSG::SetBodyLabelsScale(double length) {
    m_body_labels_scale = length;
}

void ChVisualSystemVSG::ToggleLinkLabelVisibility() {
    m_show_link_labels = !m_show_link_labels;
    m_linkLabelScene->setAllChildren(m_show_link_labels);
}

void ChVisualSystemVSG::SetLinkLabelsColor(const ChColor& color) {
    m_link_labels_color = color;
}

void ChVisualSystemVSG::SetLinkLabelsScale(double length) {
    m_link_labels_scale = length;
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::WriteImageToFile(const std::string& filename) {
    m_imageFilename = filename;
    m_capture_image = true;
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::CollectActiveBodyCOMPositions(const ChAssembly& assembly, std::vector<ChVector3d>& positions) {
    std::vector<ChVector3d> local_positions;
    for (const auto& body : assembly.GetBodies()) {
        if (body->IsActive())
            local_positions.push_back(body->GetFrameCOMToAbs().GetPos());
    }
    positions.insert(positions.end(), local_positions.begin(), local_positions.end());

    for (const auto& item : assembly.GetOtherPhysicsItems()) {
        if (const auto& assmbly = std::dynamic_pointer_cast<ChAssembly>(item))
            CollectActiveBodyCOMPositions(*assmbly, positions);
    }
}

void ChVisualSystemVSG::CollectLinkFramePositions(const ChAssembly& assembly, std::vector<ChVector3d>& positions) {
    std::vector<ChVector3d> local_positions;
    for (const auto& link : assembly.GetLinks()) {
        local_positions.push_back(link->GetFrame2Abs().GetPos());
    }
    positions.insert(positions.end(), local_positions.begin(), local_positions.end());

    for (const auto& item : assembly.GetOtherPhysicsItems()) {
        if (const auto& assmbly = std::dynamic_pointer_cast<ChAssembly>(item))
            CollectLinkFramePositions(*assmbly, positions);
    }
}

void ChVisualSystemVSG::ConvertPositions(const std::vector<ChVector3d>& c, vsg::ref_ptr<vsg::vec4Array> v, double w) {
    assert(c.size() == v->size());
    for (size_t i = 0; i < c.size(); i++)
        v->set(i, vsg::vec4CH(c[i], w));
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::BindCOMSymbols() {
    auto symbol_texture_filename = GetChronoDataFile("vsg/textures/COM_symbol.png");
    auto symbol_size = m_com_frame_scale * m_com_symbol_ratio;

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(symbol_size, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, symbol_size, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 1.0f);

    vsg::StateInfo stateInfo;
    stateInfo.blending = true;
    stateInfo.billboard = true;
    stateInfo.lighting = false;
    stateInfo.image = vsg::read_cast<vsg::Data>(symbol_texture_filename, m_options);

    // collect COM positions from all active bodies
    std::vector<ChVector3d> c_pos;
    for (auto sys : m_systems)
        CollectActiveBodyCOMPositions(sys->GetAssembly(), c_pos);

    if (c_pos.empty()) {
        m_com_symbols_empty = true;
        return;
    }

    // convert to VSG array
    auto v_pos = vsg::vec4Array::create(c_pos.size());
    ConvertPositions(c_pos, v_pos, symbol_size);
    geomInfo.positions = v_pos;

    auto node = m_vsgBuilder->createQuad(geomInfo, stateInfo);
    m_comSymbolScene->addChild(m_show_com_symbols, node);

    // find vertices of the symbol quad, to set the size dynamically, there is no transform matrix
    m_com_symbol_vertices = vsg::visit<FindVec3BufferData<0>>(node).getBufferData();
    m_com_symbol_vertices->properties.dataVariance = vsg::DYNAMIC_DATA;

    // find positions of the symbol instances, to update later on
    m_com_symbol_positions = vsg::visit<FindVec4BufferData<4>>(node).getBufferData();
    m_com_symbol_positions->properties.dataVariance = vsg::DYNAMIC_DATA;
}

void ChVisualSystemVSG::BindLabels() {
    for (const auto& text : m_body_labels) {
        auto layout = vsg::StandardLayout::create();
        auto dynamic_text = vsg::Text::create();
        dynamic_text->technique = vsg::GpuLayoutTechnique::create();
        layout->billboard = true;
        layout->horizontalAlignment = vsg::StandardLayout::CENTER_ALIGNMENT;
        layout->verticalAlignment = vsg::StandardLayout::TOP_ALIGNMENT;
        layout->position = vsg::vec3(0.0, 0.0, 0.0);
        layout->horizontal = vsg::vec3(m_label_size, 0.0, 0.0);
        layout->vertical = vsg::vec3(0.0, m_label_size, 0.0);
        layout->color = vsg::vec4CH(m_body_labels_color, 1.0f);
        dynamic_text->text = text;
        dynamic_text->font = m_labelFont;
        dynamic_text->layout = layout;
        dynamic_text->setup(0, m_options);

        m_body_labels_layout.push_back(layout);
        m_body_labels_text.push_back(dynamic_text);
        m_bodyLabelScene->addChild(m_show_body_labels, dynamic_text);
    }

    for (const auto& text : m_link_labels) {
        auto layout = vsg::StandardLayout::create();
        auto dynamic_text = vsg::Text::create();
        dynamic_text->technique = vsg::GpuLayoutTechnique::create();
        layout->billboard = true;
        layout->horizontalAlignment = vsg::StandardLayout::CENTER_ALIGNMENT;
        layout->verticalAlignment = vsg::StandardLayout::TOP_ALIGNMENT;
        layout->position = vsg::vec3(0.0, 0.0, 0.0);
        layout->horizontal = vsg::vec3(m_label_size, 0.0, 0.0);
        layout->vertical = vsg::vec3(0.0, m_label_size, 0.0);
        layout->color = vsg::vec4CH(m_link_labels_color, 1.0f);
        dynamic_text->text = text;
        dynamic_text->font = m_labelFont;
        dynamic_text->layout = layout;
        dynamic_text->setup(0, m_options);

        m_link_labels_layout.push_back(layout);
        m_link_labels_text.push_back(dynamic_text);
        m_linkLabelScene->addChild(m_show_link_labels, dynamic_text);
    }
}

void ChVisualSystemVSG::BindItem(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        BindBody(body);
        return;
    }

    if (auto link = std::dynamic_pointer_cast<ChLinkBase>(item)) {
        BindLink(link);
        return;
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        BindMesh(mesh);
        return;
    }

    if (const auto& pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
        BindParticleCloud(pcloud);
        return;
    }

    if (const auto& assmbly = std::dynamic_pointer_cast<ChAssembly>(item)) {
        BindAssembly(*assmbly);
        return;
    }

    if (item->GetVisualModel()) {
        BindDeformableMesh(item, DeformableType::OTHER);
        BindPointPoint(item);
    }
}

void ChVisualSystemVSG::BindAll() {
    {
        auto transform = vsg::MatrixTransform::create();
        transform->matrix = vsg::dmat4CH(ChFramed(), m_abs_frame_scale);
        vsg::Mask mask = m_show_abs_frame;
        auto node = m_shapeBuilder->createFrameSymbol(transform, 1.0f, 2.0f);
        node->setValue("Transform", transform);
        m_absFrameScene->addChild(mask, node);
    }

    for (auto sys : m_systems) {
        BindAssembly(sys->GetAssembly());
    }

    BindCOMSymbols();

    // Use default body and link names if not user provided
    int body_num = 0;
    int link_num = 0;
    for (auto sys : m_systems) {
        for (auto body : sys->GetBodies()) {
            if (!body->IsActive())
                continue;
            std::string body_name = body->GetName();
            if (body_name.empty()) {
                body_num++;
                body_name = "_body_" + std::to_string(body_num);
            }
            m_body_labels.push_back(vsg::stringValue::create(body_name));
        }
        for (auto link : sys->GetLinks()) {
            std::string link_name = link->GetName();
            if (link_name.empty()) {
                link_num++;
                link_name = "_link_" + std::to_string(body_num);
            }
            m_link_labels.push_back(vsg::stringValue::create(link_name));
        }
    }

    BindLabels();
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::BindBody(const std::shared_ptr<ChBody>& body) {
    if (!body->IsFixed()) {
        BindReferenceFrame(body);
        BindCOMFrame(body);
    }
    BindObjectVisualModel(body, ObjectType::BODY);
    BindObjectCollisionModel(body, body->GetTag());
}

void ChVisualSystemVSG::BindLink(const std::shared_ptr<ChLinkBase>& link) {
    BindLinkFrame(link);
    BindPointPoint(link);
    BindObjectVisualModel(link, ObjectType::LINK);
}

void ChVisualSystemVSG::BindMesh(const std::shared_ptr<fea::ChMesh>& mesh) {
    mesh->UpdateVisualModel();
    BindDeformableMesh(mesh, DeformableType::FEA);
}

void ChVisualSystemVSG::BindAssembly(const ChAssembly& assembly) {
    for (const auto& body : assembly.GetBodies())
        BindBody(body);

    for (const auto& link : assembly.GetLinks())
        BindLink(link);

    for (const auto& mesh : assembly.GetMeshes())
        BindMesh(mesh);

    for (const auto& item : assembly.GetOtherPhysicsItems()) {
        BindDeformableMesh(item, DeformableType::OTHER);
        BindPointPoint(item);
        if (const auto& pcloud = std::dynamic_pointer_cast<ChParticleCloud>(item))
            BindParticleCloud(pcloud);
        if (const auto& assmbly = std::dynamic_pointer_cast<ChAssembly>(item))
            BindAssembly(*assmbly);
    }
}

void ChVisualSystemVSG::BindObjectVisualModel(const std::shared_ptr<ChObj>& obj, ObjectType type) {
    const auto& vis_model = obj->GetVisualModel();
    const auto& vis_frame = obj->GetVisualModelFrame();

    if (!vis_model)
        return;

    // Important for update: keep the correct scenegraph hierarchy
    //     modelGroup->model_transform->shapes_group

    auto vis_model_group = vsg::Group::create();

    // Create a group to hold the shapes with their subtransforms
    auto vis_shapes_group = vsg::Group::create();

    // Populate the group with shapes in the visual model
    PopulateVisGroup(vis_shapes_group, vis_model);

    // Create transform and initialize with current frame
    auto vis_model_transform = vsg::MatrixTransform::create();
    vis_model_transform->matrix = vsg::dmat4CH(vis_frame, 1.0);
     // Enable frustum culling means we're not wasting rendering resources on things out of frame
    vis_model_transform->subgraphRequiresLocalFrustum = true; 
    
    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(vis_model_group);
        m_options->sharedObjects->share(vis_model_transform);
    }
    vis_model_transform->addChild(vis_shapes_group);
    vis_model_group->addChild(vis_model_transform);

    vis_model_group->setValue("Object", obj);
    vis_model_group->setValue("Type", type);
    vis_model_group->setValue("Tag", obj->GetTag());
    vis_model_group->setValue("Transform", vis_model_transform);

    // Add the group to the global holder
    vsg::Mask mask;
    switch (type) {
        case ObjectType::BODY:
            mask = m_show_body_objs;
            break;
        case ObjectType::LINK:
            mask = m_show_link_objs;
            break;
        default:
            mask = true;
            break;
    }
    m_objScene->addChild(mask, vis_model_group);
}

void ChVisualSystemVSG::BindObjectCollisionModel(const std::shared_ptr<ChContactable>& obj, int tag) {
    const auto& coll_model = obj->GetCollisionModel();
    const auto& coll_frame = obj->GetCollisionModelFrame();

    if (!coll_model)
        return;

    if (coll_model->GetShapeInstances().empty())
        return;

    // Important for update: keep the correct scenegraph hierarchy
    //     modelGroup->model_transform->shapes_group

    // Create a group to hold this visual model
    auto coll_model_group = vsg::Group::create();

    // Create a group to hold the shapes with their subtransforms
    auto coll_shapes_group = vsg::Group::create();

    // Populate the group with shapes in the visual model
    PopulateCollGroup(coll_shapes_group, coll_model);

    // Attach a transform to the group and initialize it with the body current position
    auto vis_model_transform = vsg::MatrixTransform::create();
    vis_model_transform->matrix = vsg::dmat4CH(coll_frame, 1.0);
    vis_model_transform->subgraphRequiresLocalFrustum = true;  // Enable frustum culling to reduce recordAndSubmit overhead
    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(coll_model_group);
        m_options->sharedObjects->share(vis_model_transform);
    }
    vis_model_transform->addChild(coll_shapes_group);
    coll_model_group->addChild(vis_model_transform);

    // Set group properties
    coll_model_group->setValue("Object", obj);
    coll_model_group->setValue("Tag", tag);
    coll_model_group->setValue("Transform", vis_model_transform);

    // Find colors array in current collision model group and set them to dynamic.
    auto colors = vsg::visit<FindVec4BufferData<3>>(coll_model_group).getBufferData();
    colors->properties.dataVariance = vsg::DYNAMIC_DATA;
    m_collision_colors.push_back(colors);

    // Add the group to the global holder
    vsg::Mask mask = m_show_collision;
    m_collisionScene->addChild(mask, coll_model_group);
}

void ChVisualSystemVSG::BindDeformableMesh(const std::shared_ptr<ChPhysicsItem>& item, DeformableType type) {
    const auto& vis_model = item->GetVisualModel();

    if (!vis_model)
        return;

    for (auto& shape_instance : vis_model->GetShapeInstances()) {
        auto& shape = shape_instance.shape;

        //// RADU TODO: process glyphs
        ////            for now, only treat the trimeshes in the visual model
        auto trimesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape);
        if (!trimesh)
            continue;

        if (trimesh->GetMesh()->GetNumVertices() == 0)
            continue;

        DeformableMesh def_mesh;
        def_mesh.trimesh = trimesh->GetMesh();

        auto transform = vsg::MatrixTransform::create();
        auto child = (trimesh->GetNumMaterials() > 0)
                         ? m_shapeBuilder->CreateTrimeshPbrMatShape(trimesh->GetMesh(), transform,
                                                                    trimesh->GetMaterials(), trimesh->IsWireframe())
                         : m_shapeBuilder->CreateTrimeshColShape(trimesh->GetMesh(), transform, trimesh->GetColor(),
                                                                 trimesh->IsWireframe());
        child->setValue("Type", type);
        child->setValue("Tag", item->GetTag());
        vsg::Mask mask;
        switch (type) {
            case DeformableType::FEA:
                mask = m_show_fea_meshes;
                break;
            default:
                mask = true;
                break;
        }
        m_deformableScene->addChild(mask, child);

        def_mesh.mesh_soup = true;

        def_mesh.vertices = vsg::visit<FindVec3BufferData<0>>(child).getBufferData();
        assert(def_mesh.vertices->size() == 3 * trimesh->GetMesh()->GetNumTriangles());
        def_mesh.vertices->properties.dataVariance = vsg::DYNAMIC_DATA;
        def_mesh.dynamic_vertices = true;

        if (!trimesh->IsWireframe()) {
            def_mesh.normals = vsg::visit<FindVec3BufferData<1>>(child).getBufferData();
            assert(def_mesh.normals->size() == def_mesh.vertices->size());
            def_mesh.normals->properties.dataVariance = vsg::DYNAMIC_DATA;
            def_mesh.dynamic_normals = true;
        } else {
            def_mesh.dynamic_normals = false;
        }

        if (trimesh->GetNumMaterials() == 0) {
            def_mesh.colors = vsg::visit<FindVec4BufferData<3>>(child).getBufferData();
            assert(def_mesh.colors->size() == def_mesh.vertices->size());
            def_mesh.colors->properties.dataVariance = vsg::DYNAMIC_DATA;
            def_mesh.dynamic_colors = true;
        } else {
            def_mesh.dynamic_colors = false;
        }

        m_def_meshes.push_back(def_mesh);
    }
}

// Utility function for creating a frame with its X axis defined by 2 points.
ChFrame<> PointPointFrame(const ChVector3d& P1, const ChVector3d& P2, double& dist) {
    ChVector3d dir = P2 - P1;
    dist = dir.Length();
    dir.Normalize();
    ChVector3d mx, my, mz;
    dir.GetDirectionAxesAsX(my, mz, mx);
    ChMatrix33<> R_CS;
    R_CS.SetFromDirectionAxes(mx, my, mz);

    return ChFrame<>(0.5 * (P2 + P1), R_CS);
}

void ChVisualSystemVSG::BindPointPoint(const std::shared_ptr<ChPhysicsItem>& item) {
    const auto& vis_model = item->GetVisualModel();

    if (!vis_model)
        return;

    vsg::Mask mask_segments = m_show_spring_dampers;
    vsg::Mask mask_springs = m_show_spring_dampers;

    for (auto& shape_instance : vis_model->GetShapeInstances()) {
        auto& shape = shape_instance.shape;

        if (auto segshape = std::dynamic_pointer_cast<ChVisualShapeSegment>(shape)) {
            double length;
            auto X = PointPointFrame(segshape->GetPoint1Abs(), segshape->GetPoint2Abs(), length);
            std::shared_ptr<ChVisualMaterial> material =
                shape->GetMaterials().empty() ? ChVisualMaterial::Default() : shape->GetMaterial(0);

            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X, ChVector3d(1, length, 1));
            auto group = m_shapeBuilder->CreateUnitSegment(material, transform, 2.0f);
            group->setValue("Type", PointPointType::SEGMENT);
            group->setValue("Tag", item->GetTag());
            group->setValue("Shape", shape);
            group->setValue("Transform", transform);
            m_pointpointScene->addChild(mask_segments, group);
        } else if (auto sprshape = std::dynamic_pointer_cast<ChVisualShapeSpring>(shape)) {
            // VSG generates spring coils procedurally on GPU, so disable expensive CPU geometry updates
            sprshape->SetGeometryUpdatesDisabled(true);
            
            auto rad = sprshape->GetRadius();
            auto turns = sprshape->GetTurns();
            auto resolution = sprshape->GetResolution();
            double length;
            auto X = PointPointFrame(sprshape->GetPoint1Abs(), sprshape->GetPoint2Abs(), length);
            std::shared_ptr<ChVisualMaterial> material =
                shape->GetMaterials().empty() ? ChVisualMaterial::Default() : shape->GetMaterial(0);

            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X, ChVector3d(rad, length, rad));
            auto group = m_shapeBuilder->CreateSpringShape(material, transform, resolution, turns, 2.0f);
            group->setValue("Type", PointPointType::SPRING);
            group->setValue("Tag", item->GetTag());
            group->setValue("Shape", shape);
            group->setValue("Transform", transform);
            m_pointpointScene->addChild(mask_springs, group);
        }
    }
}

void ChVisualSystemVSG::BindParticleCloud(const std::shared_ptr<ChParticleCloud>& pcloud) {
    const auto& vis_model = pcloud->GetVisualModel();
    auto num_particles = pcloud->GetNumParticles();

    if (!vis_model)
        return;

    // Search for an appropriate rendering shape
    typedef ChGeometry::Type ShapeType;
    auto shape = vis_model->GetShape(0);
    bool wireframe = vis_model->UseWireframe(0);
    ShapeType shape_type = ShapeType::NONE;
    ChVector3d shape_size(0);
    if (auto sph = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
        shape_type = ShapeType::SPHERE;
        shape_size = ChVector3d(2 * sph->GetRadius());
    } else if (auto ell = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
        shape_type = ShapeType::ELLIPSOID;
        shape_size = ell->GetAxes();
    } else if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
        shape_type = ShapeType::BOX;
        shape_size = box->GetLengths();
    } else if (auto cap = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
        double rad = cap->GetRadius();
        double height = cap->GetHeight();
        shape_type = ShapeType::CAPSULE;
        shape_size = ChVector3d(2 * rad, 2 * rad, height);
    } else if (auto cyl = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
        double rad = cyl->GetRadius();
        double height = cyl->GetHeight();
        shape_type = ShapeType::CYLINDER;
        shape_size = ChVector3d(2 * rad, 2 * rad, height);
    } else if (auto cone = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
        double rad = cone->GetRadius();
        double height = cone->GetHeight();
        shape_type = ShapeType::CONE;
        shape_size = ChVector3d(2 * rad, 2 * rad, height);
    }

    if (shape_type == ShapeType::NONE)
        return;

    // Search for the base color
    ChColor shape_color;
    if (shape->GetNumMaterials() > 0)
        shape_color = shape->GetMaterial(0)->GetDiffuseColor();
    else
        shape_color = ChVisualMaterial::Default()->GetDiffuseColor();

    // Create an new entry in the set of Chrono::VSG particle clouds
    ParticleCloud cloud;
    cloud.pcloud = pcloud;
    cloud.dynamic_positions = pcloud->IsActive();
    cloud.dynamic_colors = pcloud->UseDynamicColors();

    // Set up geometry and state info for vsgBuilder
    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set((float)shape_size.x(), 0, 0);
    geomInfo.dy.set(0, (float)shape_size.y(), 0);
    geomInfo.dz.set(0, 0, (float)shape_size.z());

    if (cloud.dynamic_colors) {
        cloud.colors = vsg::vec4Array::create(num_particles);
        geomInfo.colors = cloud.colors;
        for (size_t k = 0; k < num_particles; k++)
            cloud.colors->set(k, vsg::vec4(0, 0, 0, 1));
        cloud.colors->properties.dataVariance = vsg::DYNAMIC_DATA;
    } else {
        geomInfo.color.set(shape_color.R, shape_color.G, shape_color.B, 1.0);
    }

    cloud.positions = vsg::vec3Array::create(num_particles);
    geomInfo.positions = cloud.positions;
    for (unsigned int k = 0; k < num_particles; k++)
        cloud.positions->set(k, vsg::vec3CH(pcloud->GetParticlePos(k)));
    if (cloud.dynamic_positions) {
        cloud.positions->properties.dataVariance = vsg::DYNAMIC_DATA;
    }

    vsg::StateInfo stateInfo;
    stateInfo.wireframe = wireframe;
    stateInfo.instance_positions_vec3 = true;

    // Add child node for this cloud
    vsg::ref_ptr<vsg::Node> node = nullptr;
    switch (shape_type) {
        case ShapeType::SPHERE:
        case ShapeType::ELLIPSOID:
            node = m_vsgBuilder->createSphere(geomInfo, stateInfo);
            break;
        case ShapeType::BOX:
            node = m_vsgBuilder->createBox(geomInfo, stateInfo);
            break;
        case ShapeType::CAPSULE:
            node = m_vsgBuilder->createCapsule(geomInfo, stateInfo);
            break;
        case ShapeType::CYLINDER:
            node = m_vsgBuilder->createCylinder(geomInfo, stateInfo);
            break;
        case ShapeType::CONE:
            node = m_vsgBuilder->createCone(geomInfo, stateInfo);
            break;
        default:
            break;
    }

    if (node) {
        VkBufferUsageFlags extraUsage = 0;
        if (cloud.dynamic_colors)
            extraUsage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

        if (extraUsage != 0) {
            // We need colour buffers that are both vertex inputs and storage buffers so the compute shader
            // can overwrite particle colours; standard VSG nodes were only request vertex usage, so these get replaced here
            ReplaceVertexIndexDraw replacer(extraUsage);
            node->accept(replacer);

            if (replacer.replaced_node) {
                if (m_vsgBuilder->compileTraversal) {
                    m_vsgBuilder->compileTraversal->compile(node);
                } else if (m_viewer) {
                    replacer.replaced_node->accept(*vsg::CompileTraversal::create(*m_viewer));
                }
            }
        }

        node->setValue("Tag", pcloud->GetTag());
        // Seed the scene graph node with the cached visibility choice for its tag
        // this is primarily all for the bce particle markers - so that the visibility is consistent
        // (since OnInitialize calls SetParticleCloudVisibility before the VSG scene has bound geometry)
        vsg::Mask mask = GetDesiredCloudVisibility(pcloud->GetTag());
        m_particleScene->addChild(mask, node);

        cloud.geometry_node = node;

        // Capture BufferInfo handles for instance data so plugins can share GPU buffers
        FindVertexArrayBufferInfo<4> positionVisitor;
        node->accept(positionVisitor);
        cloud.position_bufferInfo = positionVisitor.bufferInfo;
        if (cloud.dynamic_colors) {
            FindVertexArrayBufferInfo<3> colorVisitor;
            node->accept(colorVisitor);
            cloud.color_bufferInfo = colorVisitor.bufferInfo;
        }
    }
    m_clouds.push_back(cloud);
}

void ChVisualSystemVSG::BindReferenceFrame(const std::shared_ptr<ChObj>& obj) {
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::dmat4CH(obj->GetVisualModelFrame(), m_ref_frame_scale);
    vsg::Mask mask = m_show_ref_frames;
    auto node = m_shapeBuilder->createFrameSymbol(transform, 1.0f, 2.0f);
    node->setValue("Object", obj);
    node->setValue("Transform", transform);
    m_refFrameScene->addChild(mask, node);
}

void ChVisualSystemVSG::BindCOMFrame(const std::shared_ptr<ChBody>& body) {
    auto com_transform = vsg::MatrixTransform::create();
    com_transform->matrix = vsg::dmat4CH(body->GetFrameCOMToAbs(), m_com_frame_scale);
    vsg::Mask mask = m_show_com_frames;
    auto com_node = m_shapeBuilder->createFrameSymbol(com_transform, 1.0f, 2.0f, true);
    com_node->setValue("Body", body);
    com_node->setValue("MobilizedBody", nullptr);
    com_node->setValue("Transform", com_transform);
    m_comFrameScene->addChild(mask, com_node);
}

void ChVisualSystemVSG::BindLinkFrame(const std::shared_ptr<ChLinkBase>& link) {
    vsg::Mask mask = m_show_link_frames;
    {
        auto link_transform = vsg::MatrixTransform::create();
        link_transform->matrix = vsg::dmat4CH(link->GetFrame1Abs(), m_link_frame_scale);
        auto link_node = m_shapeBuilder->createFrameSymbol(link_transform, 0.75f, 1.0f, true);
        link_node->setValue("Link", link);
        link_node->setValue("Body", 1);
        link_node->setValue("Transform", link_transform);
        m_linkFrameScene->addChild(mask, link_node);
    }
    {
        auto link_transform = vsg::MatrixTransform::create();
        link_transform->matrix = vsg::dmat4CH(link->GetFrame2Abs(), m_link_frame_scale);
        auto link_node = m_shapeBuilder->createFrameSymbol(link_transform, 0.5f, 1.0f, true);
        link_node->setValue("Link", link);
        link_node->setValue("Body", 2);
        link_node->setValue("Transform", link_transform);
        m_linkFrameScene->addChild(mask, link_node);
    }
}

// -----------------------------------------------------------------------------

// Utility function to populate a VSG group with visualization shapes (from the given visual model).
void ChVisualSystemVSG::PopulateVisGroup(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChVisualModel> model) {
    for (const auto& shape_instance : model->GetShapeInstances()) {
        const auto& shape = shape_instance.shape;
        const auto& X_SM = shape_instance.frame;
        bool wireframe = shape_instance.wireframe;

        if (!shape->IsVisible())
            continue;

        // Material for primitive shapes (assumed at most one defined)
        std::shared_ptr<ChVisualMaterial> material =
            shape->GetMaterials().empty() ? ChVisualMaterial::Default() : shape->GetMaterial(0);

        if (auto box = std::dynamic_pointer_cast<ChVisualShapeBox>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, box->GetHalflengths());

            // We have boxes and dice. Dice take cubetextures, boxes take 6 identical textures.
            // Use a die if a kd map exists and its name contains "cubetexture". Otherwise, use a box.
            auto grp =
                !material->GetKdTexture().empty() && material->GetKdTexture().find("cubetexture") != std::string::npos
                    ? m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::DIE, material, transform, wireframe)
                    : m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::BOX, material, transform, wireframe);
            group->addChild(grp);
        } else if (auto sphere = std::dynamic_pointer_cast<ChVisualShapeSphere>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, sphere->GetRadius());
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::SPHERE, material, transform, wireframe);
            group->addChild(grp);
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChVisualShapeEllipsoid>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ellipsoid->GetSemiaxes());
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::SPHERE, material, transform, wireframe);
            group->addChild(grp);
        } else if (auto cylinder = std::dynamic_pointer_cast<ChVisualShapeCylinder>(shape)) {
            double rad = cylinder->GetRadius();
            double height = cylinder->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, height));
            auto grp =
                m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CYLINDER, material, transform, wireframe);
            group->addChild(grp);
        } else if (auto capsule = std::dynamic_pointer_cast<ChVisualShapeCapsule>(shape)) {
            double rad = capsule->GetRadius();
            double height = capsule->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, rad / 2 + height / 4));
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CAPSULE, material, transform, wireframe);
            group->addChild(grp);
        } else if (auto barrel = std::dynamic_pointer_cast<ChVisualShapeBarrel>(shape)) {
            //// TODO
        } else if (auto cone = std::dynamic_pointer_cast<ChVisualShapeCone>(shape)) {
            double rad = cone->GetRadius();
            double height = cone->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, height));
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CONE, material, transform, wireframe);
            group->addChild(grp);
        } else if (auto trimesh = std::dynamic_pointer_cast<ChVisualShapeTriangleMesh>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, trimesh->GetScale());
            auto grp = trimesh->GetNumMaterials() > 0
                           ? m_shapeBuilder->CreateTrimeshPbrMatShape(trimesh->GetMesh(), transform,
                                                                      trimesh->GetMaterials(), wireframe)
                           : m_shapeBuilder->CreateTrimeshColShape(trimesh->GetMesh(), transform, trimesh->GetColor(),
                                                                   wireframe);
            group->addChild(grp);
        } else if (auto surface = std::dynamic_pointer_cast<ChVisualShapeSurface>(shape)) {
            auto geometry = surface->GetSurfaceGeometry();
            auto resolution_u = surface->GetResolutionU();
            auto resolution_v = surface->GetResolutionV();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, 1.0);
            auto grp = m_shapeBuilder->CreatePbrSurfaceShape(geometry, material, transform, resolution_u, resolution_v,
                                                             wireframe);
            group->addChild(grp);
        } else if (auto model_file = std::dynamic_pointer_cast<ChVisualShapeModelFile>(shape)) {
            const auto& filename = model_file->GetFilename();
            const auto& scale = model_file->GetScale();

            auto ext = filesystem::path(filename).extension();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);
            ChQuaterniond rot = (ext == "OBJ" || ext == "STL") ? QUNIT : QuatFromAngleX(-CH_PI_2);

            size_t objHashValue = m_stringHash(filename);
            auto grp = vsg::Group::create();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(ChFrame<>(X_SM.GetPos(), X_SM.GetRot() * rot), scale);
            grp->addChild(transform);
            // needed, when BindAll() is called after Initialization
            // vsg::observer_ptr<vsg::Viewer> observer_viewer(m_viewer);
            // m_loadThreads->add(LoadOperation::create(observer_viewer, transform, filename, m_options));
            std::map<size_t, vsg::ref_ptr<vsg::Node>>::iterator objIt;
            objIt = m_objCache.find(objHashValue);
            if (objIt == m_objCache.end()) {
                auto node = vsg::read_cast<vsg::Node>(filename, m_options);
                if (node) {
                    transform->addChild(node);
                    group->addChild(grp);
                    m_objCache[objHashValue] = node;
                }
            } else {
                transform->addChild(m_objCache[objHashValue]);
                group->addChild(grp);
            }
        } else if (auto line = std::dynamic_pointer_cast<ChVisualShapeLine>(shape)) {
            auto geometry = line->GetLineGeometry();
            auto num_points = line->GetNumRenderPoints();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, 1.0);
            group->addChild(m_shapeBuilder->CreateLineShape(geometry, material, transform, num_points));
        } else if (auto path = std::dynamic_pointer_cast<ChVisualShapePath>(shape)) {
            auto geometry = path->GetPathGeometry();
            auto num_points = path->GetNumRenderPoints();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, 1.0);
            group->addChild(m_shapeBuilder->CreatePathShape(geometry, material, transform, num_points));
        }

    }  // end loop over visual shapes
}

// Utility function to populate a VSG group with collision shapes (from the given collision model).
void ChVisualSystemVSG::PopulateCollGroup(vsg::ref_ptr<vsg::Group> group, std::shared_ptr<ChCollisionModel> model) {
    // Default visualization material for collision shapes
    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(m_collision_color);

    for (const auto& shape_instance : model->GetShapeInstances()) {
        const auto& shape = shape_instance.shape;
        const auto& X_SM = shape_instance.frame;

        if (auto box = std::dynamic_pointer_cast<ChCollisionShapeBox>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, box->GetHalflengths());
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::BOX, material, transform, true);
            group->addChild(grp);
        } else if (auto sphere = std::dynamic_pointer_cast<ChCollisionShapeSphere>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, sphere->GetRadius());
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::SPHERE, material, transform, true);
            group->addChild(grp);
        } else if (auto ellipsoid = std::dynamic_pointer_cast<ChCollisionShapeEllipsoid>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ellipsoid->GetSemiaxes());
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::SPHERE, material, transform, true);
            group->addChild(grp);
        } else if (auto cylinder = std::dynamic_pointer_cast<ChCollisionShapeCylinder>(shape)) {
            double rad = cylinder->GetRadius();
            double height = cylinder->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, height));
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CYLINDER, material, transform, true);
            group->addChild(grp);
        } else if (auto capsule = std::dynamic_pointer_cast<ChCollisionShapeCapsule>(shape)) {
            double rad = capsule->GetRadius();
            double height = capsule->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, rad / 2 + height / 4));
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CAPSULE, material, transform, true);
            group->addChild(grp);
        } else if (auto cone = std::dynamic_pointer_cast<ChCollisionShapeCone>(shape)) {
            double rad = cone->GetRadius();
            double height = cone->GetHeight();
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(rad, rad, height));
            auto grp = m_shapeBuilder->CreatePbrShape(ShapeBuilder::ShapeType::CONE, material, transform, true);
            group->addChild(grp);
        } else if (auto trimesh = std::dynamic_pointer_cast<ChCollisionShapeTriangleMesh>(shape)) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(1, 1, 1));
            auto trimesh_connected = std::dynamic_pointer_cast<ChTriangleMeshConnected>(trimesh->GetMesh());
            if (!trimesh_connected)  //// TODO: ChTriangleMeshSoup
                continue;
            auto grp = m_shapeBuilder->CreateTrimeshColShape(trimesh_connected, transform, m_collision_color, true);
            group->addChild(grp);
        } else if (auto hull = std::dynamic_pointer_cast<ChCollisionShapeConvexHull>(shape)) {
            auto trimesh_connected = chrono_types::make_shared<ChTriangleMeshConnected>();
            bt_utils::ChConvexHullLibraryWrapper lh;
            lh.ComputeHull(hull->GetPoints(), *trimesh_connected);
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X_SM, ChVector3d(1, 1, 1));
            auto grp = m_shapeBuilder->CreateTrimeshColShape(trimesh_connected, transform, m_collision_color, true);
            group->addChild(grp);
        }
    }
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::Update() {
    if (m_show_abs_frame) {
        for (auto& child : m_absFrameScene->children) {
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            if (!child.node->getValue("Transform", transform))
                continue;

            transform->matrix = vsg::dmat4CH(ChFramed(), m_abs_frame_scale);
        }
    }

    // Update VSG nodes for object reference frame visualization
    if (m_show_ref_frames) {
        for (auto& child : m_refFrameScene->children) {
            std::shared_ptr<ChObj> obj;
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            if (!child.node->getValue("Object", obj))
                continue;
            if (!child.node->getValue("Transform", transform))
                continue;

            transform->matrix = vsg::dmat4CH(obj->GetVisualModelFrame(), m_ref_frame_scale);
        }
    }

    // Update VSG nodes for body COM visualization
    if (m_show_com_frames) {
        for (auto& child : m_comFrameScene->children) {
            std::shared_ptr<ChBody> body;
            vsg::ref_ptr<vsg::MatrixTransform> transform;

            if (!child.node->getValue("Transform", transform))
                continue;

            if (child.node->getValue("Body", body))
                transform->matrix = vsg::dmat4CH(body->GetFrameCOMToAbs(), m_com_frame_scale);
            else
                continue;
        }
    }

    // Update VSG nodes for link frame visualization
    if (m_show_link_frames) {
        for (auto& child : m_linkFrameScene->children) {
            std::shared_ptr<ChLinkBase> link;
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            int body;
            if (!child.node->getValue("Link", link))
                continue;
            if (!child.node->getValue("Transform", transform))
                continue;
            if (!child.node->getValue("Body", body))
                continue;

            if (body == 1)
                transform->matrix = vsg::dmat4CH(link->GetFrame1Abs(), m_link_frame_scale);
            else
                transform->matrix = vsg::dmat4CH(link->GetFrame2Abs(), m_link_frame_scale);
        }
    }

    // Update all VSG nodes with object visualization assets
    for (const auto& child : m_objScene->children) {
        std::shared_ptr<ChObj> obj;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child.node->getValue("Object", obj))
            continue;
        if (!child.node->getValue("Transform", transform))
            continue;
        transform->matrix = vsg::dmat4CH(obj->GetVisualModelFrame(), 1.0);
    }

    // Update all VSG nodes with point-point visualization assets
    for (const auto& child : m_pointpointScene->children) {
        std::shared_ptr<ChVisualShape> shape;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child.node->getValue("Shape", shape))
            continue;
        if (!child.node->getValue("Transform", transform))
            continue;

        if (auto segshape = std::dynamic_pointer_cast<ChVisualShapeSegment>(shape)) {
            double length;
            auto X = PointPointFrame(segshape->GetPoint1Abs(), segshape->GetPoint2Abs(), length);
            transform->matrix = vsg::dmat4CH(X, ChVector3d(0, length, 0));
        } else if (auto sprshape = std::dynamic_pointer_cast<ChVisualShapeSpring>(shape)) {
            double rad = sprshape->GetRadius();
            double length;
            auto X = PointPointFrame(sprshape->GetPoint1Abs(), sprshape->GetPoint2Abs(), length);
            transform->matrix = vsg::dmat4CH(X, ChVector3d(rad, length, rad));
        }
    }

    // Update all VSG nodes with collision visualization
    for (const auto& child : m_collisionScene->children) {
        std::shared_ptr<ChContactable> obj;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child.node->getValue("Object", obj))
            continue;
        if (!child.node->getValue("Transform", transform))
            continue;
        transform->matrix = vsg::dmat4CH(obj->GetCollisionModelFrame(), 1.0);
    }

    // Update all VSG nodes with contact visualization

    if (m_show_contact_normals || m_show_contact_forces) {
        // Reset contact drawer
        m_contact_creator->Reset();

        // Traverse contacts and update VSG nodes
        unsigned int num_contacts = 0;
        for (auto sys : m_systems) {
            sys->GetContactContainer()->ReportAllContacts(m_contact_creator);
            num_contacts += sys->GetNumContacts();
        }

        // Set visibility of VSG contact nodes (always hide the "extra" contact nodes)
        for (unsigned int i = 0; i < m_max_num_contacts; i++) {
            vsg::Mask mask_normals = m_show_contact_normals;
            vsg::Mask mask_forces = m_show_contact_forces;
            if (i >= num_contacts) {
                mask_normals = false;
                mask_forces = false;
            }
            m_contactNormalsScene->children[i].mask = mask_normals;
            m_contactForcesScene->children[i].mask = mask_forces;
        }
    }
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::CreateContacts() {
    ChFramed X;
    vsg::ref_ptr<vsg::MatrixTransform> transform;

    for (unsigned int i = 0; i < m_max_num_contacts; i++) {
        auto material = chrono_types::make_shared<ChVisualMaterial>();
        material->SetDiffuseColor(m_contact_normals_color);
        vsg::Mask mask = m_show_contact_normals;

        transform = vsg::MatrixTransform::create();
        transform->matrix = vsg::dmat4CH(X, ChVector3d(0, 1, 0));
        auto group = m_shapeBuilder->CreateUnitSegment(material, transform, 1.0f, true);
        group->setValue("Transform", transform);

        auto colors = vsg::visit<FindVec3BufferData<1>>(group).getBufferData();
        colors->properties.dataVariance = vsg::DYNAMIC_DATA;
        m_contact_normals_colors.push_back(colors);

        m_contactNormalsScene->addChild(mask, group);
    }

    for (unsigned int i = 0; i < m_max_num_contacts; i++) {
        auto material = chrono_types::make_shared<ChVisualMaterial>();
        material->SetDiffuseColor(m_contact_forces_color);
        vsg::Mask mask = m_show_contact_forces;

        transform = vsg::MatrixTransform::create();
        transform->matrix = vsg::dmat4CH(X, ChVector3d(0, 1, 0));
        auto group = m_shapeBuilder->CreateUnitSegment(material, transform, 1.0f, true);
        group->setValue("Transform", transform);

        auto colors = vsg::visit<FindVec3BufferData<1>>(group).getBufferData();
        colors->properties.dataVariance = vsg::DYNAMIC_DATA;
        m_contact_forces_colors.push_back(colors);

        m_contactForcesScene->addChild(mask, group);
    }
}

ChVisualSystemVSG::CreateContactsVSG::CreateContactsVSG(ChVisualSystemVSG* app) : m_app(app), m_crt_contact(0) {}

void ChVisualSystemVSG::CreateContactsVSG::Reset() {
    m_crt_contact = 0;
}

bool ChVisualSystemVSG::CreateContactsVSG::OnReportContact(const ChVector3d& pA,
                                                           const ChVector3d& pB,
                                                           const ChMatrix33<>& plane_coord,
                                                           double distance,
                                                           double eff_Radius,
                                                           const ChVector3d& react_forces,
                                                           const ChVector3d& react_torques,
                                                           ChContactable* modA,
                                                           ChContactable* modB,
                                                           int constraint_offset) {
    // If we reached the alloted number of contact nodes, return now and stop scanning contacts
    if (m_crt_contact >= m_app->m_max_num_contacts)
        return false;

    if (m_app->m_show_contact_normals) {
        auto len = m_app->m_contact_normals_scale;
        auto X = PointPointFrame(pB, pB - plane_coord.GetAxisX() * len, len);

        auto child = m_app->m_contactNormalsScene->children[m_crt_contact];

        vsg::ref_ptr<vsg::MatrixTransform> transform;
        child.node->getValue("Transform", transform);
        transform->matrix = vsg::dmat4CH(X, ChVector3d(0, len, 0));
    }

    if (m_app->m_show_contact_forces) {
        auto scaled_force = react_forces / 1000;
        auto vec = plane_coord * (scaled_force * m_app->m_contact_forces_scale);
        auto len = vec.Length();
        auto X = PointPointFrame(pA, pA + vec, len);

        auto child = m_app->m_contactForcesScene->children[m_crt_contact];

        vsg::ref_ptr<vsg::MatrixTransform> transform;
        child.node->getValue("Transform", transform);
        transform->matrix = vsg::dmat4CH(X, ChVector3d(0, len, 0));
    }

    m_crt_contact++;

    // Continue scanning contacts
    return true;
}

/*
 *
 * TODO: this version does not work with current VSG if shadows are enabled.
 * This is because there are issues with creating nodes after initialization of the shadow processing!
 *

ChVisualSystemVSG::CreateContactsVSG::CreateContactsVSG(ChVisualSystemVSG* app)
    : m_app(app), m_num_existing_normals_nodes(0), m_num_existing_forces_nodes(0) {
    m_mat_normals = chrono_types::make_shared<ChVisualMaterial>();
    m_mat_normals->SetDiffuseColor(m_app->m_contact_normals_color);
    m_mat_forces = chrono_types::make_shared<ChVisualMaterial>();
    m_mat_forces->SetDiffuseColor(m_app->m_contact_forces_color);
}

void ChVisualSystemVSG::CreateContactsVSG::Reset() {
    m_num_existing_normals_nodes = m_app->m_contactNormalsScene->children.size();
    m_num_existing_forces_nodes = m_app->m_contactForcesScene->children.size();
    m_crt_normals_node = 0;
    m_crt_forces_node = 0;
}

bool ChVisualSystemVSG::CreateContactsVSG::OnReportContact(const ChVector3d& pA,
                                                           const ChVector3d& pB,
                                                           const ChMatrix33<>& plane_coord,
                                                           double distance,
                                                           double eff_Radius,
                                                           const ChVector3d& react_forces,
                                                           const ChVector3d& react_torques,
                                                           ChContactable* modA,
                                                           ChContactable* modB) {
    if (m_app->m_show_contact_normals) {
        auto len = m_app->m_contact_normals_scale;
        auto X = PointPointFrame(pB, pB - plane_coord.GetAxisX() * len, len);
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        vsg::Mask mask = m_app->m_show_contact_normals;

        if (m_crt_normals_node < m_num_existing_normals_nodes) {
            auto child = m_app->m_contactNormalsScene->children[m_crt_normals_node];
            child.node->getValue("Transform", transform);
            transform->matrix = vsg::dmat4CH(X, ChVector3d(0, len, 0));
        } else {
            transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X, ChVector3d(0, len, 0));
            auto group = m_app->m_shapeBuilder->CreateUnitSegment(m_mat_normals, transform, 1.0f);
            group->setValue("Transform", transform);
            m_app->m_contactNormalsScene->addChild(mask, group);
        }

        m_crt_normals_node++;
    }

    if (m_app->m_show_contact_forces) {
        auto vec = plane_coord * (react_forces * m_app->m_contact_forces_scale);
        auto len = vec.Length();
        auto X = PointPointFrame(pA, pA + vec, len);
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        vsg::Mask mask = m_app->m_show_contact_forces;

        if (m_crt_forces_node < m_num_existing_forces_nodes) {
            auto child = m_app->m_contactForcesScene->children[m_crt_normals_node];
            child.node->getValue("Transform", transform);
            transform->matrix = vsg::dmat4CH(X, ChVector3d(0, len, 0));
        } else {
            transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::dmat4CH(X, ChVector3d(0, len, 0));
            auto group = m_app->m_shapeBuilder->CreateUnitSegment(m_mat_forces, transform, 1.0f);
            group->setValue("Transform", transform);
            m_app->m_contactForcesScene->addChild(mask, group);
        }

        m_crt_forces_node++;
    }

    // Continue scanning contacts
    return true;
}
*/

void ChVisualSystemVSG::OnSetup(ChSystem* sys) {
    //// RADU TODO
    ////    delete VSG elements associated with physics items no longer present in the system
}

int ChVisualSystemVSG::AddVisualModel(std::shared_ptr<ChVisualModel> model, const ChFrame<>& frame) {
    // Important for update: keep the correct scenegraph hierarchy
    //     model_group->model_transform->shapes_group

    // Create a group to hold this visual model
    auto vis_model_group = vsg::Group::create();

    // Create a group to hold the shapes with their subtransforms
    auto vis_shapes_group = vsg::Group::create();

    // Populate the group with shapes in the visual model
    PopulateVisGroup(vis_shapes_group, model);

    // Attach a transform to the group and initialize it with the provided frame
    auto vis_model_transform = vsg::MatrixTransform::create();
    vis_model_transform->matrix = vsg::dmat4CH(frame, 1.0);
    vis_model_transform->subgraphRequiresLocalFrustum = true;  // Enable frustum culling to reduce recordAndSubmit overhead
    if (m_options->sharedObjects) {
        m_options->sharedObjects->share(vis_model_group);
        m_options->sharedObjects->share(vis_model_transform);
    }
    vis_model_transform->addChild(vis_shapes_group);
    vis_model_group->addChild(vis_model_transform);

    // Set group properties
    vis_model_group->setValue("Transform", vis_model_transform);

    // Add the group to the global holder
    m_decoScene->addChild(vis_model_group);

    return m_decoScene->children.size() - 1;
}

int ChVisualSystemVSG::AddVisualModel(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame) {
    auto model = chrono_types::make_shared<ChVisualModel>();
    model->AddShape(shape);
    return AddVisualModel(model, frame);
}

void ChVisualSystemVSG::UpdateVisualModel(int id, const ChFrame<>& frame) {
    if (id == -1 || id >= m_decoScene->children.size()) {
        return;
    }

    auto model_group = m_decoScene->children[id];
    vsg::ref_ptr<vsg::MatrixTransform> transform;
    if (!model_group->getValue("Transform", transform))
        return;

    transform->matrix = vsg::dmat4CH(frame, 1.0);
}

// -----------------------------------------------------------------------------

void ChVisualSystemVSG::AddGrid(double x_step, double y_step, int nx, int ny, ChCoordsys<> pos, ChColor col) {
    m_decoScene->addChild(m_shapeBuilder->CreateGrid(x_step, y_step, nx, ny, pos, col));
}

void ChVisualSystemVSG::ExportScreenImage() {
    // code taken from vsgExamples vsgscreenshot.cpp

    auto width = m_window->extent2D().width;
    auto height = m_window->extent2D().height;

    auto device = m_window->getDevice();
    auto physicalDevice = m_window->getPhysicalDevice();
    auto swapchain = m_window->getSwapchain();

    // get the colour buffer image of the previous rendered frame as the current frame hasn't been rendered yet.  The 1
    // in window->imageIndex(1) means image from 1 frame ago.
    auto sourceImage = m_window->imageView(m_window->imageIndex(1))->image;

    VkFormat sourceImageFormat = swapchain->getImageFormat();
    VkFormat targetImageFormat = sourceImageFormat;

    //
    // 1) Check to see if Blit is supported.
    //
    VkFormatProperties srcFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), sourceImageFormat, &srcFormatProperties);

    VkFormatProperties destFormatProperties;
    vkGetPhysicalDeviceFormatProperties(*(physicalDevice), VK_FORMAT_R8G8B8A8_SRGB, &destFormatProperties);

    bool supportsBlit = ((srcFormatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT) != 0) &&
                        ((destFormatProperties.linearTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT) != 0);

    if (supportsBlit) {
        // we can automatically convert the image format when blit, so take advantage of it to ensure RGBA
        targetImageFormat = VK_FORMAT_R8G8B8A8_SRGB;
    }

    //
    // 2) create image to write to
    //
    auto destinationImage = vsg::Image::create();
    destinationImage->imageType = VK_IMAGE_TYPE_2D;
    destinationImage->format = targetImageFormat;
    destinationImage->extent.width = width;
    destinationImage->extent.height = height;
    destinationImage->extent.depth = 1;
    destinationImage->arrayLayers = 1;
    destinationImage->mipLevels = 1;
    destinationImage->initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    destinationImage->samples = VK_SAMPLE_COUNT_1_BIT;
    destinationImage->tiling = VK_IMAGE_TILING_LINEAR;
    destinationImage->usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    destinationImage->compile(device);

    auto deviceMemory =
        vsg::DeviceMemory::create(device, destinationImage->getMemoryRequirements(device->deviceID),
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    destinationImage->bind(deviceMemory, 0);

    //
    // 3) create command buffer and submit to graphics queue
    //
    auto commands = vsg::Commands::create();

    // 3.a) transition destinationImage to transfer destination initialLayout
    auto transitionDestinationImageToDestinationLayoutBarrier = vsg::ImageMemoryBarrier::create(
        0,                                                              // srcAccessMask
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // dstAccessMask
        VK_IMAGE_LAYOUT_UNDEFINED,                                      // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.b) transition swapChainImage from present to transfer source initialLayout
    auto transitionSourceImageToTransferSourceLayoutBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_MEMORY_READ_BIT,                                      // srcAccessMask
        VK_ACCESS_TRANSFER_READ_BIT,                                    // dstAccessMask
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // oldLayout
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionForTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                        // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                        // dstStageMask
                                     0,                                                     // dependencyFlags
                                     transitionDestinationImageToDestinationLayoutBarrier,  // barrier
                                     transitionSourceImageToTransferSourceLayoutBarrier     // barrier
        );

    commands->addChild(cmd_transitionForTransferBarrier);

    if (supportsBlit) {
        // 3.c.1) if blit using vkCmdBlitImage
        VkImageBlit region{};
        region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.srcSubresource.layerCount = 1;
        region.srcOffsets[0] = VkOffset3D{0, 0, 0};
        region.srcOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 1};
        region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.dstSubresource.layerCount = 1;
        region.dstOffsets[0] = VkOffset3D{0, 0, 0};
        region.dstOffsets[1] = VkOffset3D{static_cast<int32_t>(width), static_cast<int32_t>(height), 1};

        auto blitImage = vsg::BlitImage::create();
        blitImage->srcImage = sourceImage;
        blitImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        blitImage->dstImage = destinationImage;
        blitImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        blitImage->regions.push_back(region);
        blitImage->filter = VK_FILTER_NEAREST;

        commands->addChild(blitImage);
    } else {
        // 3.c.2) else use vkCmdCopyImage

        VkImageCopy region{};
        region.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.srcSubresource.layerCount = 1;
        region.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.dstSubresource.layerCount = 1;
        region.extent.width = width;
        region.extent.height = height;
        region.extent.depth = 1;

        auto copyImage = vsg::CopyImage::create();
        copyImage->srcImage = sourceImage;
        copyImage->srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        copyImage->dstImage = destinationImage;
        copyImage->dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        copyImage->regions.push_back(region);

        commands->addChild(copyImage);
    }

    // 3.d) transition destination image from transfer destination layout to general layout to enable mapping to image
    // DeviceMemory
    auto transitionDestinationImageToMemoryReadBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_WRITE_BIT,                                   // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_GENERAL,                                        // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        destinationImage,                                               // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    // 3.e) transition swap chain image back to present
    auto transitionSourceImageBackToPresentBarrier = vsg::ImageMemoryBarrier::create(
        VK_ACCESS_TRANSFER_READ_BIT,                                    // srcAccessMask
        VK_ACCESS_MEMORY_READ_BIT,                                      // dstAccessMask
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,                           // oldLayout
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,                                // newLayout
        VK_QUEUE_FAMILY_IGNORED,                                        // srcQueueFamilyIndex
        VK_QUEUE_FAMILY_IGNORED,                                        // dstQueueFamilyIndex
        sourceImage,                                                    // image
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}  // subresourceRange
    );

    auto cmd_transitionFromTransferBarrier =
        vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_TRANSFER_BIT,                 // srcStageMask
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,                 // dstStageMask
                                     0,                                              // dependencyFlags
                                     transitionDestinationImageToMemoryReadBarrier,  // barrier
                                     transitionSourceImageBackToPresentBarrier       // barrier
        );

    commands->addChild(cmd_transitionFromTransferBarrier);

    auto fence = vsg::Fence::create(device);
    auto queueFamilyIndex = physicalDevice->getQueueFamily(VK_QUEUE_GRAPHICS_BIT);
    auto commandPool = vsg::CommandPool::create(device, queueFamilyIndex);
    auto queue = device->getQueue(queueFamilyIndex);

    vsg::submitCommandsToQueue(commandPool, fence, 100000000000, queue,
                               [&](vsg::CommandBuffer& commandBuffer) { commands->record(commandBuffer); });

    //
    // 4) map image and copy
    //
    VkImageSubresource subResource{VK_IMAGE_ASPECT_COLOR_BIT, 0, 0};
    VkSubresourceLayout subResourceLayout;
    vkGetImageSubresourceLayout(*device, destinationImage->vk(device->deviceID), &subResource, &subResourceLayout);

    size_t destRowWidth = width * sizeof(vsg::ubvec4);
    vsg::ref_ptr<vsg::Data> imageData;
    if (destRowWidth == subResourceLayout.rowPitch) {
        imageData = vsg::MappedData<vsg::ubvec4Array2D>::create(deviceMemory, subResourceLayout.offset, 0,
                                                                vsg::Data::Properties{targetImageFormat}, width,
                                                                height);  // deviceMemory, offset, flags and dimensions
    } else {
        // Map the buffer memory and assign as a ubyteArray that will automatically unmap itself on destruction.
        // A ubyteArray is used as the graphics buffer memory is not contiguous like vsg::Array2D, so map to a flat
        // buffer first then copy to Array2D.
        auto mappedData = vsg::MappedData<vsg::ubyteArray>::create(deviceMemory, subResourceLayout.offset, 0,
                                                                   vsg::Data::Properties{targetImageFormat},
                                                                   subResourceLayout.rowPitch * height);
        imageData = vsg::ubvec4Array2D::create(width, height, vsg::Data::Properties{targetImageFormat});
        for (uint32_t row = 0; row < height; ++row) {
            std::memcpy(imageData->dataPointer(row * width), mappedData->dataPointer(row * subResourceLayout.rowPitch),
                        destRowWidth);
        }
    }

    if (!vsg::write(imageData, m_imageFilename, m_options)) {
        std::cout << "Failed to write color buffer to " << m_imageFilename << std::endl;
    }
}

}  // namespace vsg3d
}  // namespace chrono
