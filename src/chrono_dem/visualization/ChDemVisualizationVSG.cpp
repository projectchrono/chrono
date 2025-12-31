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

#include "chrono_dem/visualization/ChDemVisualizationVSG.h"

#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/shapes/ShapeBuilder.h"

namespace chrono {
namespace dem {

// -----------------------------------------------------------------------------

// Custom stats overlay
class DEMStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    DEMStatsVSG(ChDemVisualizationVSG* vsysFSI) : m_visDEM(vsysFSI) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        vsg3d::ChVisualSystemVSG& vsys = m_visDEM->GetVisualSystemVSG();

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("DEM DEM");

        if (ImGui::BeginTable("DEM", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Num. particles:");
            ImGui::TableNextColumn();
            ImGui::Text("%8lu", static_cast<unsigned long>(m_visDEM->m_sysDEM->GetNumParticles()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Step size:");
            ImGui::TableNextColumn();
            ImGui::Text("%8.1e", m_visDEM->m_sysDEM->GetFixedStepSize());

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("RTF (DEM):");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f", m_visDEM->m_sysDEM->GetRTF());

            ImGui::EndTable();
        }

        if (ImGui::BeginTable("Particles", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool particles_visible = m_visDEM->m_show_particles;
            if (ImGui::Checkbox("Particles", &particles_visible)) {
                m_visDEM->m_show_particles = !m_visDEM->m_show_particles;
                vsys.SetParticleCloudVisibility(m_visDEM->m_show_particles,0);
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

  private:
    ChDemVisualizationVSG* m_visDEM;
};

// ---------------------------------------------------------------------------

ChDemVisualizationVSG::ChDemVisualizationVSG(ChSystemDem* sysDEM)
    : m_sysDEM(sysDEM),
      m_show_particles(true),
      m_default_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_colormap_type(ChColormap::Type::JET),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC("DEM_internal_system");
}

ChDemVisualizationVSG::~ChDemVisualizationVSG() {
    delete m_sysMBS;
}

ChColormap::Type ChDemVisualizationVSG::GetColormapType() const {
    return m_colormap_type;
}

const ChColormap& ChDemVisualizationVSG::GetColormap() const {
    return *m_colormap;
}

void ChDemVisualizationVSG::SetColorCallback(std::shared_ptr<ParticleColorCallback> functor, ChColormap::Type type) {
    m_color_fun = functor;
    m_color_fun->m_vsys = this;

    m_colormap_type = type;
    if (m_colormap) {
        m_colormap->Load(type);
    }
}

void ChDemVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sysMBS);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChDemVisualizationVSG::OnInitialize() {
    // Create particle cloud; initialize its visibility flag
    {
        m_particle_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_particle_cloud->SetName("particles");
        m_particle_cloud->SetTag(0);
        m_particle_cloud->SetFixed(false);
        for (int i = 0; i < m_sysDEM->GetNumParticles(); i++) {
            m_particle_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysDEM->GetParticleRadius());
        sphere->SetColor(ChColor(0.10f, 0.40f, 0.65f));
        m_particle_cloud->AddVisualShape(sphere);
        m_particle_cloud->RegisterColorCallback(m_color_fun);
        m_particle_cloud->RegisterVisibilityCallback(m_vis_fun);
        m_sysMBS->Add(m_particle_cloud);

        m_vsys->SetParticleCloudVisibility(m_show_particles, 0);
    }

    // Create colormap
    m_colormap = chrono_types::make_unique<ChColormap>(m_colormap_type);

    // Create custom GUI for the DEM plugin
    auto dem_stats = chrono_types::make_shared<DEMStatsVSG>(this);
    m_vsys->AddGuiComponent(dem_stats);

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

void ChDemVisualizationVSG::OnBindAssets() {
}

void ChDemVisualizationVSG::OnRender() {
    if (!m_show_particles)
        return;

    auto num_particles = m_sysDEM->GetNumParticles();

    m_pos.clear();
    m_vel.clear();
    m_pos.resize(num_particles);
    m_vel.resize(num_particles);

    for (unsigned int i = 0; i < num_particles; i++) {
        // Copy SPH particle positions from device to host
        m_pos[i] = m_sysDEM->GetParticlePosition(i);
        if (m_color_fun) {
            m_vel[i] = m_sysDEM->GetParticleVelocity(i);
        }

        // Set particle position in the particle cloud
        m_particle_cloud->Particle(i).SetPos(m_pos[i]);
    }

    // Set members for the callback functors (if defined)
    if (m_color_fun) {
        m_color_fun->pos = m_pos.data();
        m_color_fun->vel = m_vel.data();
    }
    if (m_vis_fun) {
        m_vis_fun->pos = m_pos.data();
    }
}

// ---------------------------------------------------------------------------

ParticleHeightColorCallback::ParticleHeightColorCallback(double hmin, double hmax, const ChVector3d& up)
    : m_hmin(hmin), m_hmax(hmax), m_up(up) {}

std::string ParticleHeightColorCallback::GetTile() const {
    return "Height (m)";
}

ChVector2d ParticleHeightColorCallback::GetDataRange() const {
    return ChVector2d(m_hmin, m_hmax);
}

ChColor ParticleHeightColorCallback::GetColor(unsigned int n) const {
    double h = Vdot(pos[n], m_up);
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
            v = vel[n].Length();
            break;
        case Component::X:
            v = std::abs(vel[n].x());
            break;
        case Component::Y:
            v = std::abs(vel[n].y());
            break;
        case Component::Z:
            v = std::abs(vel[n].z());
            break;
    }

    return m_vsys->GetColormap().Get(v, m_vmin, m_vmax);
}

}  // namespace dem
}  // namespace chrono
