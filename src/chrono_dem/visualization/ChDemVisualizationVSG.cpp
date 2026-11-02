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

#include <algorithm>
#include <cmath>
#include <iostream>

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_dem/visualization/ChDemVisualizationVSG.h"

#include "chrono_vsg/utils/ChDataUtilsVSG.h"
#include "chrono_vsg/utils/ChShapeBuilderVSG.h"

namespace chrono {
namespace dem {

// -----------------------------------------------------------------------------

// Visibility of particles: region filter (if applied to particles) and user callback
class DEMParticleVisibility : public ChParticleCloud::VisibilityCallback {
  public:
    DEMParticleVisibility(ChDemVisualizationVSG* visDEM) : m_visDEM(visDEM) {}

    virtual bool get(unsigned int n, const ChParticleCloud& cloud) const override {
        if (m_visDEM->m_region_particles && n < m_visDEM->m_pos.size() && !m_visDEM->InRegion(ChVector3d(m_visDEM->m_pos[n])))
            return false;
        return !m_visDEM->m_vis_fun || m_visDEM->m_vis_fun->get(n);
    }

  private:
    ChDemVisualizationVSG* m_visDEM;
};

// -----------------------------------------------------------------------------

// Custom stats overlay
class DEMStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    DEMStatsVSG(ChDemVisualizationVSG* visDEM) : m_visDEM(visDEM) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        vsg3d::ChVisualSystemVSG& vsys = m_visDEM->GetVisualSystemVSG();

        // Start out at the right side of the window, below the logo (only the initial position is set)
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - GUI_PADDING, viewport->WorkPos.y + 100.0f), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("Chrono::DEM");

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

            if (m_visDEM->m_network_visible) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Contacts:");
                ImGui::TableNextColumn();
                ImGui::Text("%8u", m_visDEM->m_num_contacts);

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Network contacts:");
                ImGui::TableNextColumn();
                ImGui::Text("%8u", m_visDEM->m_num_contacts_drawn);

                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Dropped segments:");
                ImGui::TableNextColumn();
                ImGui::Text("%8u", m_visDEM->GetNumNetworkSegmentsDropped());
            }

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

            ImGui::TableNextColumn();
            ImGui::Checkbox("Network", &m_visDEM->m_network_visible);

            ImGui::EndTable();
        }

        if (m_visDEM->m_network_visible) {
            int mode = static_cast<int>(m_visDEM->m_network_mode);
            ImGui::RadioButton("Lines", &mode, static_cast<int>(vsg3d::ChForceNetworkVSG::Mode::LINES));
            ImGui::SameLine();
            ImGui::RadioButton("Cylinders", &mode, static_cast<int>(vsg3d::ChForceNetworkVSG::Mode::CYLINDERS));
            m_visDEM->m_network_mode = static_cast<vsg3d::ChForceNetworkVSG::Mode>(mode);

            // Quantity used to color the network (the combo box has no label and spans the panel width)
            const char* quantities[] = {"Normal force", "Coulomb ratio", "Tangential displacement"};
            int coloring = static_cast<int>(m_visDEM->m_coloring);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::Combo("##coloring", &coloring, quantities, 3))
                m_visDEM->m_coloring = static_cast<ChDemVisualizationVSG::NetworkColoring>(coloring);

            if (m_visDEM->m_use_colormap)
                Colorbar(vsys.GetColormapTexture(m_visDEM->m_network_colormap_type), m_visDEM->m_current_range, false, 300.0f, cb.deviceID);
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
      m_image_dir("."),
      m_capacity(0),
      m_network_visible(false),
      m_network_mode(vsg3d::ChForceNetworkVSG::Mode::CYLINDERS),
      m_coloring(NetworkColoring::NORMAL_FORCE),
      m_use_colormap(true),
      m_network_colormap_type(ChColormap::Type::JET),
      m_color_min(ChColor(0.0f, 0.0f, 1.0f)),
      m_color_max(ChColor(1.0f, 0.0f, 0.0f)),
      m_radius(0),
      m_scale_radius(false),
      m_rmin(0),
      m_rmax(0),
      m_fmin(0),
      m_fmax(0),
      m_threshold(0),
      m_relative_threshold(false),
      m_current_range(0, 1),
      m_num_contacts(0),
      m_num_contacts_drawn(0),
      m_contacts_warning(false),
      m_use_slab(false),
      m_slab_point(VNULL),
      m_slab_normal(VECT_Z),
      m_slab_half_thickness(0),
      m_use_box(false),
      m_region_particles(false) {
    m_sysMBS = new ChSystemSMC("DEM_internal_system");

    // Color ranges (an empty range is set automatically from the data at each frame)
    m_ranges[static_cast<int>(NetworkColoring::NORMAL_FORCE)] = ChVector2d(0, 0);
    m_ranges[static_cast<int>(NetworkColoring::COULOMB_RATIO)] = ChVector2d(0, 1);
    m_ranges[static_cast<int>(NetworkColoring::TANGENTIAL_DISPLACEMENT)] = ChVector2d(0, 0);
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

void ChDemVisualizationVSG::SetNetworkColoring(NetworkColoring quantity, double vmin, double vmax) {
    m_coloring = quantity;
    m_ranges[static_cast<int>(quantity)] = ChVector2d(vmin, vmax);
}

void ChDemVisualizationVSG::SetNetworkColormap(ChColormap::Type type) {
    m_use_colormap = true;
    m_network_colormap_type = type;
}

void ChDemVisualizationVSG::SetNetworkColorRamp(const ChColor& color_min, const ChColor& color_max) {
    m_use_colormap = false;
    m_color_min = color_min;
    m_color_max = color_max;
}

void ChDemVisualizationVSG::SetNetworkRadius(double radius) {
    m_radius = radius;
    m_scale_radius = false;
}

void ChDemVisualizationVSG::SetNetworkRadius(double rmin, double rmax, double fmin, double fmax) {
    m_rmin = rmin;
    m_rmax = rmax;
    m_fmin = fmin;
    m_fmax = fmax;
    m_scale_radius = true;
}

void ChDemVisualizationVSG::SetNetworkForceThreshold(double threshold, bool relative) {
    m_threshold = threshold;
    m_relative_threshold = relative;
}

unsigned int ChDemVisualizationVSG::GetNumNetworkSegments() const {
    return (m_network && m_network_visible) ? m_network->GetNumSegments() : 0;
}

unsigned int ChDemVisualizationVSG::GetNumNetworkSegmentsDropped() const {
    return (m_network && m_network_visible) ? m_network->GetNumDropped() : 0;
}

void ChDemVisualizationVSG::SetRegionSlab(const ChVector3d& point, const ChVector3d& normal, double half_thickness) {
    m_use_slab = true;
    m_slab_point = point;
    m_slab_normal = normal.GetNormalized();
    m_slab_half_thickness = half_thickness;
}

void ChDemVisualizationVSG::SetRegionBox(const ChAABB& box) {
    m_use_box = true;
    m_box = box;
}

void ChDemVisualizationVSG::ClearRegion() {
    m_use_slab = false;
    m_use_box = false;
}

bool ChDemVisualizationVSG::InRegion(const ChVector3d& p) const {
    if (m_use_slab && std::abs(Vdot(p - m_slab_point, m_slab_normal)) > m_slab_half_thickness)
        return false;
    if (m_use_box && (p.x() < m_box.min.x() || p.y() < m_box.min.y() || p.z() < m_box.min.z() ||  //
                      p.x() > m_box.max.x() || p.y() > m_box.max.y() || p.z() > m_box.max.z()))
        return false;
    return true;
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
        m_particle_cloud->RegisterVisibilityCallback(chrono_types::make_shared<DEMParticleVisibility>(this));
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
    if (m_vsys->ShadowsEnabled()) {
        std::cerr << "WARNING:  Shadow rendering is enabled for the associated VSG visualization system.\n";
        std::cerr << "          This negatively affects rendering performance, especially for large particle systems."
                  << std::endl;
    }
}

void ChDemVisualizationVSG::OnBindAssets() {
    // Force network (created only before the visual system is initialized)
    if (m_vsys->IsInitialized())
        return;

    unsigned int capacity = m_capacity;
    if (capacity == 0)
        capacity = static_cast<unsigned int>(std::clamp<size_t>(6 * m_sysDEM->GetNumParticles(), 1024, 2000000));
    m_network = chrono_types::make_unique<vsg3d::ChForceNetworkVSG>(capacity);
    m_network->SetMode(m_network_mode);
    m_network->SetVisibility(m_network_visible);
    m_network->Initialize(*m_vsys);
}

void ChDemVisualizationVSG::OnRender() {
    // For display in the VSG GUI, which reports the time and RTF of the internal Chrono system
    m_sysMBS->SetChTime(m_sysDEM->GetSimTime());
    m_sysMBS->SetRTF(m_sysDEM->GetRTF());

    if (!m_show_particles && !m_network_visible) {
        UpdateNetwork();
        return;
    }

    auto num_particles = m_sysDEM->GetNumParticles();

    m_pos.clear();
    m_vel.clear();
    m_pos.resize(num_particles);
    m_vel.resize(num_particles);

    for (unsigned int i = 0; i < num_particles; i++) {
        // Copy particle positions from device to host
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

    UpdateNetwork();
}

void ChDemVisualizationVSG::UpdateNetwork() {
    m_num_contacts = 0;
    m_num_contacts_drawn = 0;
    if (!m_network)
        return;

    m_network->SetMode(m_network_mode);
    m_network->SetVisibility(m_network_visible);
    if (!m_network_visible)
        return;

    if (!m_sysDEM->GetContacts(m_contacts)) {
        if (!m_contacts_warning) {
            std::cerr << "WARNING: ChDemVisualizationVSG: contacts are not available for the force network.\n"
                      << "         A frictional simulation with contact recording enabled is required (ChSystemDem::SetRecordingContactInfo)." << std::endl;
            m_contacts_warning = true;
        }
    }
    m_num_contacts = static_cast<unsigned int>(m_contacts.size());

    // Normal force magnitudes and threshold, possibly relative to the mean normal force over all contacts
    double threshold = m_threshold;
    if (m_relative_threshold) {
        double sum = 0;
        unsigned int count = 0;
        for (const auto& contact : m_contacts) {
            double fn = contact.normal_force.Length();
            if (fn > 0) {
                sum += fn;
                count++;
            }
        }
        threshold = (count > 0) ? m_threshold * sum / count : 0;
    }

    double particle_radius = m_sysDEM->GetParticleRadius();
    float radius = static_cast<float>((m_radius > 0) ? m_radius : 0.2 * particle_radius);
    double mu_s2s = m_sysDEM->GetStaticFrictionCoeff_SPH2SPH();
    double mu_s2w = m_sysDEM->GetStaticFrictionCoeff_SPH2WALL();

    m_points1.clear();
    m_points2.clear();
    m_values.clear();
    m_radii.clear();
    double value_max = 0;

    for (const auto& contact : m_contacts) {
        double fn = contact.normal_force.Length();
        if (fn <= 0 || fn <= threshold)
            continue;

        // A particle-particle contact is a segment between the particle centers; a contact with a boundary condition
        // is a segment from the particle center to the contact point (opposite to the normal force on the particle)
        const ChVector3f& p1 = m_pos[contact.particle];
        ChVector3f p2 = contact.boundary ? p1 - contact.normal_force * static_cast<float>(particle_radius / fn) : m_pos[contact.partner];
        ChVector3f cp = contact.boundary ? p2 : (p1 + p2) * 0.5f;
        if (!InRegion(ChVector3d(cp)))
            continue;

        double value = 0;
        switch (m_coloring) {
            case NetworkColoring::NORMAL_FORCE:
                value = fn;
                break;
            case NetworkColoring::COULOMB_RATIO: {
                double mu = contact.boundary ? mu_s2w : mu_s2s;
                value = (mu > 0) ? contact.tangential_force.Length() / (mu * fn) : 0;
                break;
            }
            case NetworkColoring::TANGENTIAL_DISPLACEMENT:
                value = contact.tangential_displacement.Length();
                break;
        }
        value_max = std::max(value_max, value);

        float r = radius;
        if (m_scale_radius) {
            double t = (m_fmax > m_fmin) ? std::clamp((fn - m_fmin) / (m_fmax - m_fmin), 0.0, 1.0) : 1.0;
            r = static_cast<float>(m_rmin + t * (m_rmax - m_rmin));
        }

        m_points1.insert(m_points1.end(), {p1.x(), p1.y(), p1.z()});
        m_points2.insert(m_points2.end(), {p2.x(), p2.y(), p2.z()});
        m_values.push_back(static_cast<float>(value));
        m_radii.push_back(r);
        m_num_contacts_drawn++;
    }

    // Range of values mapped to colors (from 0 to the largest value, if no range was set)
    ChVector2d range = m_ranges[static_cast<int>(m_coloring)];
    if (range.y() <= range.x())
        range = ChVector2d(0, value_max > 0 ? value_max : 1);
    m_current_range = range;

    if (m_use_colormap)
        m_network->SetColormap(m_network_colormap_type);
    else
        m_network->SetColorRamp(m_color_min, m_color_max);
    m_network->SetValueRange(range.x(), range.y());
    m_network->SetRadius(radius);

    m_network->SetSegments(static_cast<unsigned int>(m_values.size()), m_points1.data(), m_points2.data(), m_values.data(), nullptr, m_scale_radius ? m_radii.data() : nullptr);
}

// ---------------------------------------------------------------------------

ParticleHeightColorCallback::ParticleHeightColorCallback(double hmin, double hmax, const ChVector3d& up)
    : m_hmin(hmin), m_hmax(hmax), m_up(up) {}

std::string ParticleHeightColorCallback::GetTile() const {
    return "Height";
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
    return "Velocity";
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
