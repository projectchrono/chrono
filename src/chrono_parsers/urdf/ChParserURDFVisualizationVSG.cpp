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
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <algorithm>

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vsg/utils/ChDataUtilsVSG.h"
#include "chrono_vsg/utils/ChShapeBuilderVSG.h"

#include "chrono_parsers/urdf/ChParserURDFVisualizationVSG.h"

using std::cout;
using std::endl;

namespace chrono {
namespace parsers {

// -----------------------------------------------------------------------------

// Custom stats overlay
class ChParserURDFStats : public vsg3d::ChGuiComponentVSG {
  public:
    ChParserURDFStats(ChParserURDFVisualizationVSG* vsysURDF) : vsysURDF(vsysURDF) {
        const auto& urdf = vsysURDF->m_urdf;

        int body_tag = 0;
        for (const auto& body : urdf.m_bodies) {
            BodySettings settings = {body->GetName(), body_tag++, true, false, false};
            body_settings.push_back(settings);
        }

        int joint_tag = 0;
        for (const auto& joint : urdf.m_joints) {
            JointSettings settings = {joint->GetName(), joint_tag++, false};
            joint_settings.push_back(settings);
        }
    }

    virtual void render(vsg::CommandBuffer& cb) override {
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("URDF Model");

        ImGui::Text("Model name: %s           ", vsysURDF->m_urdf.GetName().c_str());

        ImGui::Spacing();

        ImGuiComboFlags combo_flags = ImGuiComboFlags_WidthFitPreview;
        ImGuiTableFlags table_flags = ImGuiTableFlags_BordersOuter;

        auto nbodies = body_settings.size();
        auto njoints = joint_settings.size();

        if (nbodies > 0 && ImGui::CollapsingHeader("Bodies", ImGuiTreeNodeFlags_SpanAvailWidth)) {
            static int ibody = 0;
            if (ImGui::BeginCombo("##name##body", body_settings[ibody].name.c_str(), combo_flags)) {
                for (int n = 0; n < nbodies; n++) {
                    const bool is_selected = (ibody == n);
                    if (ImGui::Selectable(body_settings[n].name.c_str(), is_selected))
                        ibody = n;
                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            if (ImGui::BeginTable("Body visualization settings", 1, table_flags, ImVec2(0.0f, 0.0f))) {
                ImGui::TableNextColumn();
                if (ImGui::Checkbox("Visible", &body_settings[ibody].visible)) {
                    vsysURDF->m_vsys->SetBodyObjVisibility(body_settings[ibody].visible, body_settings[ibody].tag);
                }

                ImGui::TableNextColumn();
                if (ImGui::Checkbox("Ref frame", &body_settings[ibody].ref_frame)) {
                    vsysURDF->m_vsys->SetRefFrameVisibility(body_settings[ibody].ref_frame, body_settings[ibody].tag);
                }

                ImGui::TableNextColumn();
                if (ImGui::Checkbox("COM frame", &body_settings[ibody].com_frame)) {
                    vsysURDF->m_vsys->SetCOMFrameVisibility(body_settings[ibody].com_frame, body_settings[ibody].tag);
                }

                ImGui::EndTable();
            }
        }

        if (njoints > 0 && ImGui::CollapsingHeader("Joints", ImGuiTreeNodeFlags_SpanAvailWidth)) {
            static int ijoint = 0;
            if (ImGui::BeginCombo("##name##joint", joint_settings[ijoint].name.c_str(), combo_flags)) {
                for (int n = 0; n < njoints; n++) {
                    const bool is_selected = (ijoint == n);
                    if (ImGui::Selectable(joint_settings[n].name.c_str(), is_selected))
                        ijoint = n;
                    // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            if (ImGui::BeginTable("Joint visualization settings", 1, table_flags, ImVec2(0.0f, 0.0f))) {
                ImGui::TableNextColumn();
                if (ImGui::Checkbox("Frames", &joint_settings[ijoint].frames)) {
                    vsysURDF->m_vsys->SetLinkFrameVisibility(joint_settings[ijoint].frames, joint_settings[ijoint].tag);
                }
                ImGui::EndTable();
            }
        }

        ImGui::End();
    }

  private:
    struct BodySettings {
        std::string name;
        int tag;
        bool visible;
        bool ref_frame;
        bool com_frame;
    };

    struct JointSettings {
        std::string name;
        int tag;
        bool frames;
    };

    ChParserURDFVisualizationVSG* vsysURDF;
    std::vector<BodySettings> body_settings;
    std::vector<JointSettings> joint_settings;
};

// ---------------------------------------------------------------------------

ChParserURDFVisualizationVSG::ChParserURDFVisualizationVSG(const ChParserURDF& urdf) : m_urdf(urdf), m_sys(urdf.m_sys), m_write_images(false), m_image_dir(".") {}

ChParserURDFVisualizationVSG::~ChParserURDFVisualizationVSG() {}

void ChParserURDFVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sys);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChParserURDFVisualizationVSG::OnInitialize() {
    // Create custom GUI for the RoboSimian plugin
    auto fsi_states = chrono_types::make_shared<ChParserURDFStats>(this);
    m_vsys->AddGuiComponent(fsi_states);

    m_vsys->SetImageOutput(m_write_images);
    m_vsys->SetImageOutputDirectory(m_image_dir);
}

void ChParserURDFVisualizationVSG::OnBindAssets() {}

void ChParserURDFVisualizationVSG::OnRender() {}

}  // namespace parsers
}  // namespace chrono
