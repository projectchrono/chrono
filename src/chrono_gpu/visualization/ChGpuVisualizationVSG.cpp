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

#include "chrono_gpu/visualization/ChGpuVisualizationVSG.h"

#include "chrono_vsg/utils/ChConversionsVSG.h"
#include "chrono_vsg/shapes/ShapeBuilder.h"

namespace chrono {
namespace gpu {

// -----------------------------------------------------------------------------

// Custom stats overlay
class GPUStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    GPUStatsVSG(ChGpuVisualizationVSG* vsysFSI) : m_visGPU(vsysFSI) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        vsg3d::ChVisualSystemVSG& vsys = m_visGPU->GetVisualSystemVSG();

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("GPU DEM");

        if (ImGui::BeginTable("DEM", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Num. particles:");
            ImGui::TableNextColumn();
            ImGui::Text("%8lu", static_cast<unsigned long>(m_visGPU->m_sysGPU->GetNumParticles()));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Step size:");
            ImGui::TableNextColumn();
            ImGui::Text("%8.1e", m_visGPU->m_sysGPU->GetFixedStepSize());

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("RTF (DEM):");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f", m_visGPU->m_sysGPU->GetRTF());

            ImGui::EndTable();
        }

        if (ImGui::BeginTable("Particles", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool particles_visible = m_visGPU->m_show_particles;
            if (ImGui::Checkbox("Particles", &particles_visible)) {
                m_visGPU->m_show_particles = !m_visGPU->m_show_particles;
                vsys.SetParticleCloudVisibility(m_visGPU->m_show_particles,0);
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

  private:
    ChGpuVisualizationVSG* m_visGPU;
};

// ---------------------------------------------------------------------------

ChGpuVisualizationVSG::ChGpuVisualizationVSG(ChSystemGpu* sysGPU)
    : m_sysGPU(sysGPU),
      m_show_particles(true),
      m_default_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_colormap_type(ChColormap::Type::JET),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC("GPU_internal_system");
}

ChGpuVisualizationVSG::~ChGpuVisualizationVSG() {
    delete m_sysMBS;
}

ChColormap::Type ChGpuVisualizationVSG::GetColormapType() const {
    return m_colormap_type;
}

const ChColormap& ChGpuVisualizationVSG::GetColormap() const {
    return *m_colormap;
}

void ChGpuVisualizationVSG::SetColorCallback(std::shared_ptr<ParticleColorCallback> functor, ChColormap::Type type) {
    m_color_fun = functor;
    m_color_fun->m_vsys = this;

    m_colormap_type = type;
    if (m_colormap) {
        m_colormap->Load(type);
    }
}

void ChGpuVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sysMBS);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChGpuVisualizationVSG::OnInitialize() {
    // Create particle cloud; initialize its visibility flag
    {
        m_particle_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_particle_cloud->SetName("particles");
        m_particle_cloud->SetTag(0);
        m_particle_cloud->SetFixed(false);
        for (int i = 0; i < m_sysGPU->GetNumParticles(); i++) {
            m_particle_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysGPU->GetParticleRadius());
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
    auto gpu_stats = chrono_types::make_shared<GPUStatsVSG>(this);
    m_vsys->AddGuiComponent(gpu_stats);

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

void ChGpuVisualizationVSG::OnBindAssets() {
}

void ChGpuVisualizationVSG::OnRender() {
    if (!m_show_particles)
        return;

    auto num_particles = m_sysGPU->GetNumParticles();

    m_pos.clear();
    m_vel.clear();
    m_pos.resize(num_particles);
    m_vel.resize(num_particles);

    for (unsigned int i = 0; i < num_particles; i++) {
        // Copy SPH particle positions from device to host
        m_pos[i] = m_sysGPU->GetParticlePosition(i);
        if (m_color_fun) {
            m_vel[i] = m_sysGPU->GetParticleVelocity(i);
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

}  // namespace gpu
}  // namespace chrono
