// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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

#include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"
#include "chrono_fsi/sph/utils/UtilsTypeConvert.cuh"

#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/shapes/ShapeBuilder.h"

namespace chrono {
namespace fsi {
namespace sph {

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSIStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    FSIStatsVSG(ChFsiVisualizationVSG* vsysFSI) : m_vsysFSI(vsysFSI) {}

    virtual void render() override {
        vsg3d::ChVisualSystemVSG& vsys = m_vsysFSI->GetVisualSystemVSG();

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin(m_vsysFSI->m_sysSPH->GetPhysicsProblemString().c_str());

        if (ImGui::BeginTable("SPH", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("SPH particles:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumFluidMarkers()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Boundary BCE:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumBoundaryMarkers()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Rigid body BCE:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumRigidBodyMarkers()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Flex body BCE:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysSPH->GetNumFlexBodyMarkers()));

            ImGui::TableNextRow();

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(m_vsysFSI->m_sysSPH->GetSphIntegrationSchemeString().c_str());

            if (m_vsysFSI->m_sysFSI) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Step size:");
                ImGui::TableNextColumn();
                ImGui::Text("%8.1e", m_vsysFSI->m_sysFSI->GetStepSizeCFD());
            } else {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Step size:");
                ImGui::TableNextColumn();
                ImGui::Text("%8.1e", m_vsysFSI->m_sysSPH->GetStepSize());
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("RTF (fluid):");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f", m_vsysFSI->m_sysSPH->GetRtf());

            if (m_vsysFSI->m_sysFSI) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("RTF (solid):");
                ImGui::TableNextColumn();
                ImGui::Text("%8.3f", m_vsysFSI->m_sysFSI->GetRtfMBD());
            }

            if (m_vsysFSI->m_sysFSI) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("MBS/CFD ratio:");
                ImGui::TableNextColumn();
                ImGui::Text("%8.3f", m_vsysFSI->m_sysFSI->GetRatioMBD());
            }

            ImGui::EndTable();
        }

        if (ImGui::BeginTable("Particles", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool sph_visible = m_vsysFSI->m_sph_markers;
            if (ImGui::Checkbox("SPH", &sph_visible)) {
                m_vsysFSI->m_sph_markers = !m_vsysFSI->m_sph_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_sph_markers, ChFsiVisualizationVSG::ParticleCloudTag::SPH);
            }

            ImGui::TableNextColumn();
            static bool bce_wall_visible = m_vsysFSI->m_bndry_bce_markers;
            if (ImGui::Checkbox("BCE wall", &bce_wall_visible)) {
                m_vsysFSI->m_bndry_bce_markers = !m_vsysFSI->m_bndry_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_bndry_bce_markers,
                                                ChFsiVisualizationVSG::ParticleCloudTag::BCE_WALL);
            }

            ImGui::TableNextColumn();
            static bool bce_rigid_visible = m_vsysFSI->m_rigid_bce_markers;
            if (ImGui::Checkbox("BCE rigid", &bce_rigid_visible)) {
                m_vsysFSI->m_rigid_bce_markers = !m_vsysFSI->m_rigid_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_rigid_bce_markers,
                                                ChFsiVisualizationVSG::ParticleCloudTag::BCE_RIGID);
            }

            ImGui::TableNextColumn();
            static bool bce_flex_visible = m_vsysFSI->m_flex_bce_markers;
            if (ImGui::Checkbox("BCE flex", &bce_flex_visible)) {
                m_vsysFSI->m_flex_bce_markers = !m_vsysFSI->m_flex_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_flex_bce_markers,
                                                ChFsiVisualizationVSG::ParticleCloudTag::BCE_FLEX);
            }

            ImGui::EndTable();
        }

        if (ImGui::BeginTable("ActiveBoxes", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool boxes_visible = m_vsysFSI->m_active_boxes;
            if (ImGui::Checkbox("Active Domains", &boxes_visible)) {
                m_vsysFSI->m_active_boxes = !m_vsysFSI->m_active_boxes;
                m_vsysFSI->SetActiveBoxVisibility(m_vsysFSI->m_active_boxes, -1);
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

  private:
    ChFsiVisualizationVSG* m_vsysFSI;
};

// ---------------------------------------------------------------------------

ChFsiVisualizationVSG::ChFsiVisualizationVSG(ChFsiSystemSPH* sysFSI)
    : m_sysFSI(sysFSI),
      m_sysSPH(&sysFSI->GetFluidSystemSPH()),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_active_boxes(false),
      m_sph_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_bndry_bce_color(ChColor(0.65f, 0.30f, 0.03f)),
      m_rigid_bce_color(ChColor(0.10f, 1.0f, 0.30f)),
      m_flex_bce_color(ChColor(1.0f, 1.0f, 0.4f)),
      m_active_box_color(ChColor(1.0f, 1.0f, 0.0f)),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC("FSI_internal_system");
    m_activeBoxScene = vsg::Switch::create();
}

ChFsiVisualizationVSG::ChFsiVisualizationVSG(ChFsiFluidSystemSPH* sysSPH)
    : m_sysFSI(nullptr),
      m_sysSPH(sysSPH),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_sph_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_bndry_bce_color(ChColor(0.65f, 0.30f, 0.03f)),
      m_rigid_bce_color(ChColor(0.10f, 1.0f, 0.30f)),
      m_flex_bce_color(ChColor(1.0f, 1.0f, 0.4f)),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC("FSI_internal_system");
}

ChFsiVisualizationVSG::~ChFsiVisualizationVSG() {
    delete m_sysMBS;
}

void ChFsiVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sysMBS);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChFsiVisualizationVSG::OnInitialize() {
    // Create particle clouds for SPH particles, as well as wall, rigid, and flex BCE markers
    // Initialize their visibility flag
    {
        m_sph_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_sph_cloud->SetName("sph_particles");
        m_sph_cloud->SetTag(ParticleCloudTag::SPH);
        m_sph_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumFluidMarkers(); i++) {
            m_sph_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 2);
        sphere->SetColor(ChColor(0.10f, 0.40f, 0.65f));
        m_sph_cloud->AddVisualShape(sphere);
        m_sph_cloud->RegisterColorCallback(m_color_fun);
        m_sph_cloud->RegisterVisibilityCallback(m_vis_sph_fun);
        m_sysMBS->Add(m_sph_cloud);

        m_vsys->SetParticleCloudVisibility(m_sph_markers, ParticleCloudTag::SPH);
    }

    {
        m_bndry_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_bndry_bce_cloud->SetName("bce_boundary");
        m_bndry_bce_cloud->SetTag(ParticleCloudTag::BCE_WALL);
        m_bndry_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumBoundaryMarkers(); i++) {
            m_bndry_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_bndry_bce_color);
        m_bndry_bce_cloud->AddVisualShape(sphere);
        m_bndry_bce_cloud->RegisterVisibilityCallback(m_vis_bndry_fun);
        m_sysMBS->Add(m_bndry_bce_cloud);

        m_vsys->SetParticleCloudVisibility(m_bndry_bce_markers, ParticleCloudTag::BCE_WALL);
    }

    {
        m_rigid_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_rigid_bce_cloud->SetName("bce_rigid");
        m_rigid_bce_cloud->SetTag(ParticleCloudTag::BCE_RIGID);
        m_rigid_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumRigidBodyMarkers(); i++) {
            m_rigid_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_rigid_bce_color);
        m_rigid_bce_cloud->AddVisualShape(sphere);
        m_sysMBS->Add(m_rigid_bce_cloud);

        m_vsys->SetParticleCloudVisibility(m_rigid_bce_markers, ParticleCloudTag::BCE_RIGID);
    }

    {
        m_flex_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_flex_bce_cloud->SetName("bce_flex");
        m_flex_bce_cloud->SetTag(ParticleCloudTag::BCE_FLEX);
        m_flex_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumFlexBodyMarkers(); i++) {
            m_flex_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_flex_bce_color);
        m_flex_bce_cloud->AddVisualShape(sphere);
        m_sysMBS->Add(m_flex_bce_cloud);

        m_vsys->SetParticleCloudVisibility(m_flex_bce_markers, ParticleCloudTag::BCE_FLEX);
    }

    // Cache information about active domains
    m_use_active_boxes = m_sysSPH->GetParams().use_active_domain;
    m_active_box_hsize = ToChVector(m_sysSPH->GetParams().bodyActiveDomain);

    // Create custom GUI for the FSI plugin
    auto fsi_states = chrono_types::make_shared<FSIStatsVSG>(this);
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

void ChFsiVisualizationVSG::OnBindAssets() {
    // Create the VSG group for the active domains
    m_vsys->GetVSGScene()->addChild(m_activeBoxScene);

    // Create the box for the computational domain
    BindComputationalDomain();

    if (!m_use_active_boxes)
        return;

    // Loop over all FSI bodies and bind a model for its active box
    for (const auto& fsi_body : m_sysFSI->GetBodies())
        BindActiveBox(fsi_body.body, fsi_body.body->GetTag());
}

void ChFsiVisualizationVSG::SetActiveBoxVisibility(bool vis, int tag) {
    if (!m_vsys->IsInitialized())
        return;

    for (auto& child : m_activeBoxScene->children) {
        int c_tag;
        child.node->getValue("Tag", c_tag);
        if (c_tag == tag || tag == -1)
            child.mask = vis;
    }
}

void ChFsiVisualizationVSG::BindComputationalDomain() {
    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(m_active_box_color);

    auto hsize = m_sysSPH->GetComputationalDomain().Size() / 2;

    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::dmat4CH(ChFramed(m_sysSPH->GetComputationalDomain().Center(), QUNIT), hsize);
    auto group =
        m_vsys->GetVSGShapeBuilder()->CreatePbrShape(vsg3d::ShapeBuilder::ShapeType::BOX, material, transform, true);

    // Set group properties
    group->setValue("Object", nullptr);
    group->setValue("Tag", -1);
    group->setValue("Transform", transform);

    // Add the group to the global holder
    vsg::Mask mask = m_active_boxes;
    m_activeBoxScene->addChild(mask, group);
}

void ChFsiVisualizationVSG::BindActiveBox(const std::shared_ptr<ChBody>& obj, int tag) {
    auto material = chrono_types::make_shared<ChVisualMaterial>();
    material->SetDiffuseColor(m_active_box_color);

    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::dmat4CH(ChFramed(obj->GetPos(), QUNIT), m_active_box_hsize);
    auto group =
        m_vsys->GetVSGShapeBuilder()->CreatePbrShape(vsg3d::ShapeBuilder::ShapeType::BOX, material, transform, true);

    // Set group properties
    group->setValue("Object", obj);
    group->setValue("Tag", tag);
    group->setValue("Transform", transform);

    // Add the group to the global holder
    vsg::Mask mask = m_active_boxes;
    m_activeBoxScene->addChild(mask, group);
}

void ChFsiVisualizationVSG::OnRender() {
    // Copy SPH particle positions from device to host
    m_pos.clear();
    m_pos = m_sysSPH->GetPositions();
    if (m_color_fun) {
        m_vel.clear();
        m_vel = m_sysSPH->GetVelocities();
        ////m_acc.clear();
        ////m_acc = m_sysSPH->GetAccelerations();
        ////m_frc.clear();
        ////m_frc = m_sysSPH->GetForces();
        m_prop.clear();
        m_prop = m_sysSPH->GetProperties();
    }

    // Set members for the callback functors (if defined)
    if (m_color_fun) {
        m_color_fun->pos = m_pos.data();
        m_color_fun->vel = m_vel.data();
        m_color_fun->prop = m_prop.data();
    }
    if (m_vis_sph_fun) {
        m_vis_sph_fun->pos = m_pos.data();
    }
    if (m_vis_bndry_fun) {
        const auto n = m_sysSPH->GetNumFluidMarkers();
        m_vis_bndry_fun->pos = &m_pos.data()[n];
    }

    // For display in VSG GUI
    if (m_sysFSI) {
        m_sysMBS->SetChTime(m_sysFSI->GetSimTime());
        m_sysMBS->SetRTF(m_sysFSI->GetRtf());
    } else {
        m_sysMBS->SetChTime(m_sysSPH->GetSimTime());
        m_sysMBS->SetRTF(m_sysSPH->GetRtf());
    }

    // Set particle positions in the various particle clouds
    size_t p = 0;

    if (m_sph_markers) {
        for (unsigned int i = 0; i < m_sysSPH->GetNumFluidMarkers(); i++) {
            m_sph_cloud->Particle(i).SetPos(ToChVector(m_pos[p + i]));
        }
    }
    p += m_sysSPH->GetNumFluidMarkers();

    if (m_bndry_bce_markers) {
        for (unsigned int i = 0; i < m_sysSPH->GetNumBoundaryMarkers(); i++) {
            m_bndry_bce_cloud->Particle(i).SetPos(ToChVector(m_pos[p + i]));
        }
    }
    p += m_sysSPH->GetNumBoundaryMarkers();

    if (m_rigid_bce_markers) {
        for (unsigned int i = 0; i < m_sysSPH->GetNumRigidBodyMarkers(); i++) {
            m_rigid_bce_cloud->Particle(i).SetPos(ToChVector(m_pos[p + i]));
        }
    }
    p += m_sysSPH->GetNumRigidBodyMarkers();

    if (m_flex_bce_markers) {
        for (unsigned int i = 0; i < m_sysSPH->GetNumFlexBodyMarkers(); i++) {
            m_flex_bce_cloud->Particle(i).SetPos(ToChVector(m_pos[p + i]));
        }
    }

    // Update positions of all active boxes
    for (const auto& child : m_activeBoxScene->children) {
        std::shared_ptr<ChBody> obj;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        if (!child.node->getValue("Object", obj))
            continue;
        if (!child.node->getValue("Transform", transform))
            continue;
        if (obj == nullptr) {
            auto hsize = m_sysSPH->GetComputationalDomain().Size() / 2;
            transform->matrix = vsg::dmat4CH(ChFramed(m_sysSPH->GetComputationalDomain().Center(), QUNIT), hsize);
        } else {
            transform->matrix = vsg::dmat4CH(ChFramed(obj->GetPos(), QUNIT), m_active_box_hsize);
        }
    }
}

// ---------------------------------------------------------------------------

ParticleHeightColorCallback::ParticleHeightColorCallback(double hmin, double hmax, const ChVector3d& up)
    : m_monochrome(false), m_hmin(hmin), m_hmax(hmax), m_up(ToReal3(up)) {}

ParticleHeightColorCallback::ParticleHeightColorCallback(const ChColor& base_color,
                                                         double hmin,
                                                         double hmax,
                                                         const ChVector3d& up)
    : m_monochrome(true), m_base_color(base_color), m_hmin(hmin), m_hmax(hmax), m_up(ToReal3(up)) {}

ChColor ParticleHeightColorCallback::get(unsigned int n) const {
    double h = dot(pos[n], m_up);  // particle height
    if (m_monochrome) {
        float factor = (float)((h - m_hmin) / (m_hmax - m_hmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else
        return ChColor::ComputeFalseColor(h, m_hmin, m_hmax);
}

ParticleVelocityColorCallback::ParticleVelocityColorCallback(double vmin, double vmax, Component component)
    : m_monochrome(false), m_vmin(vmin), m_vmax(vmax), m_component(component) {}

ParticleVelocityColorCallback::ParticleVelocityColorCallback(const ChColor& base_color,
                                                             double vmin,
                                                             double vmax,
                                                             Component component)
    : m_monochrome(true), m_base_color(base_color), m_vmin(vmin), m_vmax(vmax), m_component(component) {}

ChColor ParticleVelocityColorCallback::get(unsigned int n) const {
    double v = 0;
    switch (m_component) {
        case Component::NORM:
            v = length(vel[n]);
            break;
        case Component::X:
            v = std::abs(vel[n].x);
            break;
        case Component::Y:
            v = std::abs(vel[n].y);
            break;
        case Component::Z:
            v = std::abs(vel[n].z);
            break;
    }

    if (m_monochrome) {
        float factor = (float)((v - m_vmin) / (m_vmax - m_vmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else
        return ChColor::ComputeFalseColor(v, m_vmin, m_vmax);
}

ParticleDensityColorCallback::ParticleDensityColorCallback(double dmin, double dmax)
    : m_monochrome(false), m_dmin(dmin), m_dmax(dmax) {}

ParticleDensityColorCallback::ParticleDensityColorCallback(const ChColor& base_color, double dmin, double dmax)
    : m_monochrome(true), m_base_color(base_color), m_dmin(dmin), m_dmax(dmax) {}

ChColor ParticleDensityColorCallback::get(unsigned int n) const {
    double d = prop[n].x;

    if (m_monochrome) {
        float factor = (float)((d - m_dmin) / (m_dmax - m_dmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else
        return ChColor::ComputeFalseColor(d, m_dmin, m_dmax);
}

ParticlePressureColorCallback::ParticlePressureColorCallback(double pmin, double pmax)
    : m_monochrome(false), m_bichrome(false), m_pmin(pmin), m_pmax(pmax) {}

ParticlePressureColorCallback::ParticlePressureColorCallback(const ChColor& base_color, double pmin, double pmax)
    : m_monochrome(true), m_bichrome(false), m_base_color(base_color), m_pmin(pmin), m_pmax(pmax) {}

ParticlePressureColorCallback::ParticlePressureColorCallback(const ChColor& base_color_neg,
                                                             const ChColor& base_color_pos,
                                                             double pmin,
                                                             double pmax)
    : m_monochrome(false),
      m_bichrome(true),
      m_base_color_neg(base_color_neg),
      m_base_color_pos(base_color_pos),
      m_pmin(pmin),
      m_pmax(pmax) {
    assert(m_pmin < 0);
}

ChColor ParticlePressureColorCallback::get(unsigned int n) const {
    double p = prop[n].y;

    if (m_monochrome) {
        float factor = (float)((p - m_pmin) / (m_pmax - m_pmin));  // color scaling factor (0,1)
        return ChColor(factor * m_base_color.R, factor * m_base_color.G, factor * m_base_color.B);
    } else if (m_bichrome) {
        if (p < 0) {
            float factor = (float)(p / m_pmin);  // color scaling factor (0,1)
            return ChColor(factor * m_base_color_neg.R, factor * m_base_color_neg.G, factor * m_base_color_neg.B);
        } else {
            float factor = (float)(+p / m_pmax);  // color scaling factor (0,1)
            return ChColor(factor * m_base_color_pos.R, factor * m_base_color_pos.G, factor * m_base_color_pos.B);
        }
    } else
        return ChColor::ComputeFalseColor(p, m_pmin, m_pmax);
}

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
