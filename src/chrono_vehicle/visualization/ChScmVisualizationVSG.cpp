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
// Authors: Radu Serban
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/visualization/ChScmVisualizationVSG.h"

#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/shapes/ShapeBuilder.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

// Custom stats overlay
class SCMStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    SCMStatsVSG(ChScmVisualizationVSG* vsysFSI) : m_vsysSCM(vsysFSI) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("SCM");

        if (ImGui::BeginTable("SCM parameters", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Grid resolution:");
            ImGui::TableNextColumn();
            ImGui::Text("%6.2e", m_vsysSCM->m_scm_resolution);

            if (m_vsysSCM->m_plot_type != SCMTerrain::PLOT_NONE &&
                m_vsysSCM->m_plot_type != SCMTerrain::PLOT_ISLAND_ID &&
                m_vsysSCM->m_plot_type != SCMTerrain::PLOT_IS_TOUCHED) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(m_vsysSCM->m_plot_label.c_str());
                ImGui::TableNextColumn();
                Colorbar(m_vsysSCM->m_vsys->GetColormapTexture(m_vsysSCM->m_scm->GetColormapType()),
                         {m_vsysSCM->m_plot_min, m_vsysSCM->m_plot_max}, false, 300.0f, cb.deviceID);
            }

            ImGui::EndTable();
        }

        if (ImGui::BeginTable("ActiveBoxes", 1, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            std::string text = "Active domains ";
            text += (m_vsysSCM->m_user_active_boxes ? "(user)" : "(default)");

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            static bool boxes_visible = m_vsysSCM->m_active_boxes;
            if (ImGui::Checkbox(text.c_str(), &boxes_visible)) {
                m_vsysSCM->m_active_boxes = !m_vsysSCM->m_active_boxes;
                m_vsysSCM->SetActiveBoxVisibility(m_vsysSCM->m_active_boxes, -1);
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

  private:
    ChScmVisualizationVSG* m_vsysSCM;
};

// ---------------------------------------------------------------------------

ChScmVisualizationVSG::ChScmVisualizationVSG(SCMTerrain* scm)
    : m_scm(scm),
      m_active_boxes(false),
      m_active_box_color(ChColor(1.0f, 1.0f, 0.0f)),
      m_write_images(false),
      m_image_dir(".") {
    m_sys = new ChSystemSMC("FSI_internal_system");
    m_activeBoxScene = vsg::Switch::create();
}

ChScmVisualizationVSG::~ChScmVisualizationVSG() {
    delete m_sys;
}

void ChScmVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sys);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChScmVisualizationVSG::OnInitialize() {
    // Cache information about SCM terrain
    m_scm_resolution = m_scm->m_loader->m_delta;

    m_user_active_boxes = m_scm->m_loader->m_user_domains;
    if (!m_user_active_boxes) {
        // Note: we cannot use the SCM's m_active_domains[0] because that may not be set when we need it the first time!
        m_default_domain.m_center = VNULL;
        m_default_domain.m_hdims = {0.1, 0.1, 0.1};
    }

    m_plot_type = m_scm->m_loader->m_plot_type;
    m_plot_min = m_scm->m_loader->m_plot_v_min;
    m_plot_max = m_scm->m_loader->m_plot_v_max;
    switch (m_plot_type) {
        case SCMTerrain::PLOT_SINKAGE:
            m_plot_label = "Sinkage (m)";
            break;
        case SCMTerrain::PLOT_PRESSURE_YIELD:
            m_plot_label = "Yield pressure (N/m2)";
            break;
        case SCMTerrain::PLOT_PRESSURE:
            m_plot_label = "Pressure (N/m2)";
            break;
    }

    // Create custom GUI for the FSI plugin
    auto fsi_states = chrono_types::make_shared<SCMStatsVSG>(this);
    m_vsys->AddGuiComponent(fsi_states);

    m_vsys->SetImageOutput(m_write_images);
    m_vsys->SetImageOutputDirectory(m_image_dir);

    // Issue performance warning if shadows are enabled for the containing visualization system
    if (m_vsys->AreShadowsEnabled()) {
        std::cerr << "WARNING:  Shadow rendering is enabled for the associated VSG visualization system.\n";
        std::cerr << "          This negatively affects rendering performance, especially for large particle systems."
                  << std::endl;
    }
}

void ChScmVisualizationVSG::OnBindAssets() {
    // Create the VSG group for the active domains
    m_vsys->GetVSGScene()->addChild(m_activeBoxScene);

    if (m_user_active_boxes) {
        // Loop over all SCM active domains and bind a model for its box
        for (auto& domain : m_scm->m_loader->m_active_domains)
            BindActiveBox(domain);
    } else {
        // Bind a single box for the default active domain
        BindDefaultActiveBox();
    }
}

void ChScmVisualizationVSG::SetActiveBoxVisibility(bool vis, int tag) {
    if (!m_vsys->IsInitialized())
        return;

    for (auto& child : m_activeBoxScene->children) {
        int c_tag;
        child.node->getValue("Tag", c_tag);
        if (c_tag == tag || tag == -1)
            child.mask = vis;
    }
}

void ChScmVisualizationVSG::BindActiveBox(SCMLoader::ActiveDomainInfo& domain) {
    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(m_active_box_color);

    auto transform = vsg::MatrixTransform::create();
    auto bframe = ChFramed(domain.m_body->GetFrameRefToAbs());
    auto dframe = bframe.TransformLocalToParent(ChFramed(domain.m_center));
    transform->matrix = vsg::dmat4CH(dframe, domain.m_hdims);
    auto group =
        m_vsys->GetVSGShapeBuilder()->CreatePbrShape(vsg3d::ShapeBuilder::ShapeType::BOX, material, transform, true);

    // Set group properties
    group->setValue("Object", &domain);
    group->setValue("Tag", domain.m_body->GetTag());
    group->setValue("Transform", transform);

    // Add the group to the global holder
    vsg::Mask mask = m_active_boxes;
    m_activeBoxScene->addChild(mask, group);
}

void ChScmVisualizationVSG::BindDefaultActiveBox() {
    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(m_active_box_color);

    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::dmat4CH(ChFramed(m_default_domain.m_center, QUNIT), m_default_domain.m_hdims);
    auto group =
        m_vsys->GetVSGShapeBuilder()->CreatePbrShape(vsg3d::ShapeBuilder::ShapeType::BOX, material, transform, true);

    // Set group properties
    group->setValue("Object", &m_default_domain);
    group->setValue("Tag", -2);
    group->setValue("Transform", transform);

    // Add the group to the global holder
    vsg::Mask mask = m_active_boxes;
    m_activeBoxScene->addChild(mask, group);
}

void ChScmVisualizationVSG::OnRender() {
    // Update default active domain if necessary
    if (!m_user_active_boxes && !m_scm->m_loader->m_active_domains.empty()) {
        m_default_domain.m_center = m_scm->m_loader->m_active_domains[0].m_center;
        m_default_domain.m_hdims = m_scm->m_loader->m_active_domains[0].m_hdims;
    }

    // Update positions of all active boxes
    for (const auto& child : m_activeBoxScene->children) {
        SCMLoader::ActiveDomainInfo* obj;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child.node->getValue("Object", obj))
            continue;
        if (!child.node->getValue("Transform", transform))
            continue;

        if (m_user_active_boxes) {
            // Body-associated SCM active domain
            auto bframe = ChFramed(obj->m_body->GetFrameRefToAbs());
            auto dframe = bframe.TransformLocalToParent(ChFramed(obj->m_center));
            transform->matrix = vsg::dmat4CH(dframe, obj->m_hdims);
        } else {
            // Default SCM active domain
            transform->matrix = vsg::dmat4CH(ChFramed(m_default_domain.m_center), m_default_domain.m_hdims);
        }
    }
}

}  // namespace vehicle
}  // namespace chrono
