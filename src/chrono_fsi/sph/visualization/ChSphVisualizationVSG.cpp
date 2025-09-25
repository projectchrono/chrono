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

#include <algorithm>

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#include "chrono_fsi/sph/physics/SphDataManager.cuh"
#include "chrono_fsi/sph/utils/SphUtilsTypeConvert.cuh"

#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/shapes/ShapeBuilder.h"

namespace chrono {
namespace fsi {
namespace sph {

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSIStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    FSIStatsVSG(ChSphVisualizationVSG* vsysFSI) : m_vsysFSI(vsysFSI) {}

    virtual void render(vsg::CommandBuffer& cb) override {
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
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_sph_markers, ChSphVisualizationVSG::ParticleCloudTag::SPH);
            }

            ImGui::TableNextColumn();
            static bool bce_wall_visible = m_vsysFSI->m_bndry_bce_markers;
            if (ImGui::Checkbox("BCE wall", &bce_wall_visible)) {
                m_vsysFSI->m_bndry_bce_markers = !m_vsysFSI->m_bndry_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_bndry_bce_markers,
                                                ChSphVisualizationVSG::ParticleCloudTag::BCE_WALL);
            }

            ImGui::TableNextColumn();
            static bool bce_rigid_visible = m_vsysFSI->m_rigid_bce_markers;
            if (ImGui::Checkbox("BCE rigid", &bce_rigid_visible)) {
                m_vsysFSI->m_rigid_bce_markers = !m_vsysFSI->m_rigid_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_rigid_bce_markers,
                                                ChSphVisualizationVSG::ParticleCloudTag::BCE_RIGID);
            }

            ImGui::TableNextColumn();
            static bool bce_flex_visible = m_vsysFSI->m_flex_bce_markers;
            if (ImGui::Checkbox("BCE flex", &bce_flex_visible)) {
                m_vsysFSI->m_flex_bce_markers = !m_vsysFSI->m_flex_bce_markers;
                vsys.SetParticleCloudVisibility(m_vsysFSI->m_flex_bce_markers,
                                                ChSphVisualizationVSG::ParticleCloudTag::BCE_FLEX);
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
    ChSphVisualizationVSG* m_vsysFSI;
};

// ---------------------------------------------------------------------------

ChSphVisualizationVSG::ChSphVisualizationVSG(ChFsiSystemSPH* sysFSI)
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
      m_colormap_type(ChColormap::Type::JET),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC("FSI_internal_system");
    m_activeBoxScene = vsg::Switch::create();
}

ChSphVisualizationVSG::ChSphVisualizationVSG(ChFsiFluidSystemSPH* sysSPH)
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

ChSphVisualizationVSG::~ChSphVisualizationVSG() {
    auto& systems = m_vsys->GetSystems();
    auto index = std::find(systems.begin(), systems.end(), m_sysMBS);
    if (index != systems.end())
        systems.erase(index);

    delete m_sysMBS;
}

ChColormap::Type ChSphVisualizationVSG::GetColormapType() const {
    return m_colormap_type;
}

const ChColormap& ChSphVisualizationVSG::GetColormap() const {
    return *m_colormap;
}

void ChSphVisualizationVSG::SetSPHColorCallback(std::shared_ptr<ParticleColorCallback> functor, ChColormap::Type type) {
    m_color_fun = functor;
    m_color_fun->m_vsys = this;

    m_colormap_type = type;
    if (m_colormap) {
        m_colormap->Load(type);
    }
}

void ChSphVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sysMBS);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChSphVisualizationVSG::OnInitialize() {
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

    // Create colormap
    m_colormap = chrono_types::make_unique<ChColormap>(m_colormap_type);

    // Create custom GUI for the FSI plugin
    auto fsi_states = chrono_types::make_shared<FSIStatsVSG>(this);
    m_vsys->AddGuiComponent(fsi_states);

    // Add colorbar GUI
    if (m_color_fun) {
        m_vsys->AddGuiColorbar(m_color_fun->GetTile(), m_color_fun->GetDataRange(), m_colormap_type,
                               m_color_fun->IsBimodal(), 400.0f);
    }

    m_vsys->SetImageOutput(m_write_images);
    m_vsys->SetImageOutputDirectory(m_image_dir);

    // Issue performance warning if shadows are enabled for the containing visualization system
    if (m_vsys->AreShadowsEnabled()) {
        std::cerr << "WARNING:  Shadow rendering is enabled for the associated VSG visualization system.\n";
        std::cerr << "          This negatively affects rendering performance, especially for large particle systems."
                  << std::endl;
    }
}

void ChSphVisualizationVSG::OnBindAssets() {
    // Create the VSG group for the active domains
    m_vsys->GetVSGScene()->addChild(m_activeBoxScene);

    // Create the box for the computational domain
    BindComputationalDomain();

    if (!m_use_active_boxes)
        return;

    // Loop over all FSI bodies and bind a model for its active box
    for (const auto& fsi_body : m_sysFSI->GetBodies())
        BindActiveBox(fsi_body->body, fsi_body->body->GetTag());
}

void ChSphVisualizationVSG::SetActiveBoxVisibility(bool vis, int tag) {
    if (!m_vsys->IsInitialized())
        return;

    for (auto& child : m_activeBoxScene->children) {
        int c_tag;
        child.node->getValue("Tag", c_tag);
        if (c_tag == tag || tag == -1)
            child.mask = vis;
    }
}

void ChSphVisualizationVSG::BindComputationalDomain() {
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

void ChSphVisualizationVSG::BindActiveBox(const std::shared_ptr<ChBody>& obj, int tag) {
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

void ChSphVisualizationVSG::OnRender() {
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

MarkerPlanesVisibilityCallback::MarkerPlanesVisibilityCallback(const std::vector<Plane>& planes, Mode mode) : m_planes(planes), m_mode(mode) {}

bool MarkerPlanesVisibilityCallback::get(unsigned int n) const {
    switch (m_mode) {
        case Mode::ANY:
            // Marker in front of any plane is not visible
            for (const auto& plane : m_planes) {
                if (Vdot(ToChVector(pos[n]) - plane.point, plane.normal) > 0)
                    return false;
            }
            return true;
        case Mode::ALL:
            // Marker behind any plane is visible
            for (const auto& plane : m_planes) {
                if (Vdot(ToChVector(pos[n]) - plane.point, plane.normal) <= 0)
                    return true;
            }
            return false;
    }

    return true;
}

// ---------------------------------------------------------------------------

ParticleHeightColorCallback::ParticleHeightColorCallback(double hmin, double hmax, const ChVector3d& up)
    : m_hmin(hmin), m_hmax(hmax), m_up(ToReal3(up)) {}

std::string ParticleHeightColorCallback::GetTile() const {
    return "Height (m)";
}

ChVector2d ParticleHeightColorCallback::GetDataRange() const {
    return ChVector2d(m_hmin, m_hmax);
}

ChColor ParticleHeightColorCallback::GetColor(unsigned int n) const {
    double h = dot(pos[n], m_up);  // particle height
    return m_vsys->GetColormap().Get(h, m_hmin, m_hmax);
}

ParticleVelocityColorCallback::ParticleVelocityColorCallback(double vmin, double vmax, Component component)
    : m_vmin(vmin), m_vmax(vmax), m_component(component) {}

std::string ParticleVelocityColorCallback::GetTile() const {
    return "Velocity (m/s)";
}

ChVector2d ParticleVelocityColorCallback::GetDataRange() const {
    return ChVector2d(m_vmin, m_vmax);
}

ChColor ParticleVelocityColorCallback::GetColor(unsigned int n) const {
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

    return m_vsys->GetColormap().Get(v, m_vmin, m_vmax);
}

ParticleDensityColorCallback::ParticleDensityColorCallback(double dmin, double dmax) : m_dmin(dmin), m_dmax(dmax) {}

std::string ParticleDensityColorCallback::GetTile() const {
    return "Density (kg/m3)";
}

ChVector2d ParticleDensityColorCallback::GetDataRange() const {
    return ChVector2d(m_dmin, m_dmax);
}

ChColor ParticleDensityColorCallback::GetColor(unsigned int n) const {
    double d = prop[n].x;
    return m_vsys->GetColormap().Get(d, m_dmin, m_dmax);
}

ParticlePressureColorCallback::ParticlePressureColorCallback(double pmin, double pmax, bool bimodal)
    : m_bimodal(bimodal), m_pmin(pmin), m_pmax(pmax) {
    assert(!m_bimodal || m_pmin < 0);
}

std::string ParticlePressureColorCallback::GetTile() const {
    return "Pressure (N/m2)";
}

ChVector2d ParticlePressureColorCallback::GetDataRange() const {
    return ChVector2d(m_pmin, m_pmax);
}

ChColor ParticlePressureColorCallback::GetColor(unsigned int n) const {
    double p = prop[n].y;

    if (m_bimodal) {
        if (p < 0) {
            float factor = (float)(p / m_pmin);  // color scaling factor (1...0)
            return m_vsys->GetColormap().Get(0.5 * (1 - factor));
        } else {
            float factor = (float)(p / m_pmax);  // color scaling factor (0...1)
            return m_vsys->GetColormap().Get(0.5 * (1 + factor));
        }
    } else {
        return m_vsys->GetColormap().Get(p, m_pmin, m_pmax);
    }
}

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
