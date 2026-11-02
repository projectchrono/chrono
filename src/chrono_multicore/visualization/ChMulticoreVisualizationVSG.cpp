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
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "chrono_multicore/visualization/ChMulticoreVisualizationVSG.h"

namespace chrono {

// -----------------------------------------------------------------------------

// Visibility of granular bodies: region filter (if applied to bodies) and user callback
class MulticoreBodyVisibility : public vsg3d::ChInstancedShapesVSG::VisibilityCallback {
  public:
    MulticoreBodyVisibility(ChMulticoreVisualizationVSG* plugin) : m_plugin(plugin) {}

    virtual bool IsVisible(const ChBody& body) const override {
        if (m_plugin->m_region_bodies && !m_plugin->InRegion(body.GetPos()))
            return false;
        return !m_plugin->m_body_visibility || m_plugin->m_body_visibility->IsVisible(body);
    }

  private:
    ChMulticoreVisualizationVSG* m_plugin;
};

// -----------------------------------------------------------------------------

// Statistics and controls panel
class MulticoreStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    MulticoreStatsVSG(ChMulticoreVisualizationVSG* plugin) : m_plugin(plugin) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        auto sys = m_plugin->m_sys;
        auto dm = sys->data_manager;
        auto sys_smc = dynamic_cast<ChSystemMulticoreSMC*>(sys);
        auto& vsys = m_plugin->GetVisualSystemVSG();

        // Start out at the right side of the window, below the logo (only the initial position is set)
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - GUI_PADDING, viewport->WorkPos.y + 100.0f), ImGuiCond_FirstUseEver, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("Chrono::Multicore");

        if (ImGui::BeginTable("Counts", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit, ImVec2(0.0f, 0.0f))) {
            Row("Bodies:", static_cast<unsigned long long>(sys->GetBodies().size()));
            Row("Instanced bodies:", m_plugin->GetNumInstancedBodies());
            Row("Contacts:", dm->cd_data->num_rigid_contacts);
            if (m_plugin->m_network_visible) {
                Row("Network contacts:", m_plugin->m_num_contacts_drawn);
                Row("Network segments:", m_plugin->GetNumNetworkSegments());
                Row("Dropped segments:", m_plugin->GetNumNetworkSegmentsDropped());
            }
            ImGui::EndTable();
        }

        if (ImGui::BeginTable("Timers", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit, ImVec2(0.0f, 0.0f))) {
            Row("Step (ms):", 1e3 * sys->GetTimerStep());
            Row("  collision:", 1e3 * sys->GetTimerCollision());
            Row("    broadphase:", 1e3 * sys->GetTimerCollisionBroad());
            Row("    narrowphase:", 1e3 * sys->GetTimerCollisionNarrow());
            Row("  update:", 1e3 * sys->GetTimerUpdate());
            Row("  advance:", 1e3 * sys->GetTimerAdvance());
            if (sys_smc) {
                Row("    contact forces:", 1e3 * sys_smc->GetTimerProcessContact());
            } else {
                Row("    solver setup:", 1e3 * sys->GetTimerLSsetup());
                Row("    solver:", 1e3 * sys->GetTimerLSsolve());
                Row("    Jacobians:", 1e3 * sys->GetTimerJacobian());
            }
            ImGui::EndTable();
        }

        if (ImGui::BeginTable("Solver", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit, ImVec2(0.0f, 0.0f))) {
            Row("Threads:", sys->GetNumThreadsChrono());
            if (sys_smc) {
                Row("History overflows:", sys_smc->GetNumContactHistoryOverflows());
                Row("  total:", sys_smc->GetTotalContactHistoryOverflows());
            } else {
                Row("Iterations:", static_cast<unsigned long long>(dm->measures.solver.total_iteration));
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted("Residual:");
                ImGui::TableNextColumn();
                ImGui::Text("%8.2e", static_cast<double>(dm->measures.solver.residual));
            }
            ImGui::EndTable();
        }

        bool bodies = m_plugin->m_shapes.IsVisible();
        if (ImGui::Checkbox("Bodies", &bodies))
            m_plugin->m_shapes.SetVisibility(bodies);
        ImGui::SameLine();
        ImGui::Checkbox("Network", &m_plugin->m_network_visible);

        if (m_plugin->m_network_visible) {
            int mode = static_cast<int>(m_plugin->m_network_mode);
            ImGui::RadioButton("Lines", &mode, static_cast<int>(vsg3d::ChForceNetworkVSG::Mode::LINES));
            ImGui::SameLine();
            ImGui::RadioButton("Cylinders", &mode, static_cast<int>(vsg3d::ChForceNetworkVSG::Mode::CYLINDERS));
            m_plugin->m_network_mode = static_cast<vsg3d::ChForceNetworkVSG::Mode>(mode);

            const char* quantities[] = {"Normal force", "Coulomb ratio", "Tangential displacement"};
            int coloring = static_cast<int>(m_plugin->m_coloring);
            if (ImGui::Combo("Color", &coloring, quantities, 3))
                m_plugin->m_coloring = static_cast<ChMulticoreVisualizationVSG::NetworkColoring>(coloring);

            if (m_plugin->m_use_colormap) {
                ImGui::TextUnformatted(ChMulticoreVisualizationVSG::GetColoringName(m_plugin->m_coloring).c_str());
                Colorbar(vsys.GetColormapTexture(m_plugin->m_colormap_type), m_plugin->m_current_range, false, 300.0f, cb.deviceID);
            }
        }

        ImGui::End();
    }

  private:
    static void Row(const char* label, unsigned long long value) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::Text("%8llu", value);
    }

    static void Row(const char* label, unsigned int value) { Row(label, static_cast<unsigned long long>(value)); }

    static void Row(const char* label, double value) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::Text("%8.3f", value);
    }

    ChMulticoreVisualizationVSG* m_plugin;
};

// -----------------------------------------------------------------------------

ChMulticoreVisualizationVSG::ChMulticoreVisualizationVSG(ChSystemMulticore* sys)
    : m_sys(sys),
      m_use_instancing(true),
      m_size_factor(10),
      m_source(vsg3d::ChInstancedShapesVSG::ShapeSource::COLLISION),
      m_median_size(0),
      m_size_limit(0),
      m_capacity(0),
      m_network_visible(false),
      m_network_mode(vsg3d::ChForceNetworkVSG::Mode::CYLINDERS),
      m_coloring(NetworkColoring::NORMAL_FORCE),
      m_use_colormap(true),
      m_colormap_type(ChColormap::Type::JET),
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
      m_max_half_length(0),
      m_anchor_type(NetworkAnchor::BODY_CENTER),
      m_current_range(0, 1),
      m_num_contacts_drawn(0),
      m_use_slab(false),
      m_slab_point(VNULL),
      m_slab_normal(VECT_Z),
      m_slab_half_thickness(0),
      m_use_box(false),
      m_region_bodies(false),
      m_write_images(false),
      m_image_dir("."),
      m_image_format("bmp"),
      m_image_interval(1),
      m_num_images(0),
      m_num_frames(0) {
    // Color ranges (an empty range is set automatically from the data at each frame)
    m_ranges[static_cast<int>(NetworkColoring::NORMAL_FORCE)] = ChVector2d(0, 0);
    m_ranges[static_cast<int>(NetworkColoring::COULOMB_RATIO)] = ChVector2d(0, 1);
    m_ranges[static_cast<int>(NetworkColoring::TANGENTIAL_DISPLACEMENT)] = ChVector2d(0, 0);
}

ChMulticoreVisualizationVSG::~ChMulticoreVisualizationVSG() {}

void ChMulticoreVisualizationVSG::SetBodySizeFactor(double factor) {
    m_size_factor = factor;
    m_shapes.SetSizeFactor(factor);
}

void ChMulticoreVisualizationVSG::SetBodyShapeSource(vsg3d::ChInstancedShapesVSG::ShapeSource source) {
    m_source = source;
    m_shapes.SetShapeSource(source);
}

void ChMulticoreVisualizationVSG::SetNetworkColoring(NetworkColoring quantity, double vmin, double vmax) {
    m_coloring = quantity;
    m_ranges[static_cast<int>(quantity)] = ChVector2d(vmin, vmax);
}

void ChMulticoreVisualizationVSG::SetNetworkColormap(ChColormap::Type type) {
    m_use_colormap = true;
    m_colormap_type = type;
}

void ChMulticoreVisualizationVSG::SetNetworkColorRamp(const ChColor& color_min, const ChColor& color_max) {
    m_use_colormap = false;
    m_color_min = color_min;
    m_color_max = color_max;
}

void ChMulticoreVisualizationVSG::SetNetworkRadius(double radius) {
    m_radius = radius;
    m_scale_radius = false;
}

void ChMulticoreVisualizationVSG::SetNetworkRadius(double rmin, double rmax, double fmin, double fmax) {
    m_rmin = rmin;
    m_rmax = rmax;
    m_fmin = fmin;
    m_fmax = fmax;
    m_scale_radius = true;
}

void ChMulticoreVisualizationVSG::SetNetworkForceThreshold(double threshold, bool relative) {
    m_threshold = threshold;
    m_relative_threshold = relative;
}

void ChMulticoreVisualizationVSG::SetRegionSlab(const ChVector3d& point, const ChVector3d& normal, double half_thickness) {
    m_use_slab = true;
    m_slab_point = point;
    m_slab_normal = normal.GetNormalized();
    m_slab_half_thickness = half_thickness;
}

void ChMulticoreVisualizationVSG::SetRegionBox(const ChAABB& box) {
    m_use_box = true;
    m_box = box;
}

void ChMulticoreVisualizationVSG::ClearRegion() {
    m_use_slab = false;
    m_use_box = false;
}

void ChMulticoreVisualizationVSG::SetImageOutputInterval(unsigned int interval) {
    m_image_interval = std::max(interval, 1u);
}

unsigned int ChMulticoreVisualizationVSG::GetNumNetworkSegments() const {
    return (m_network && m_network_visible) ? m_network->GetNumSegments() : 0;
}

unsigned int ChMulticoreVisualizationVSG::GetNumNetworkSegmentsDropped() const {
    return (m_network && m_network_visible) ? m_network->GetNumDropped() : 0;
}

std::string ChMulticoreVisualizationVSG::GetColoringName(NetworkColoring quantity) {
    switch (quantity) {
        case NetworkColoring::NORMAL_FORCE:
            return "Normal force (N)";
        case NetworkColoring::COULOMB_RATIO:
            return "Coulomb ratio (-)";
        case NetworkColoring::TANGENTIAL_DISPLACEMENT:
            return "Tangential displacement (m)";
    }
    return "";
}

bool ChMulticoreVisualizationVSG::InRegion(const ChVector3d& p) const {
    if (m_use_slab && std::abs(Vdot(p - m_slab_point, m_slab_normal)) > m_slab_half_thickness)
        return false;
    if (m_use_box && (p.x() < m_box.min.x() || p.y() < m_box.min.y() || p.z() < m_box.min.z() ||  //
                      p.x() > m_box.max.x() || p.y() > m_box.max.y() || p.z() > m_box.max.z()))
        return false;
    return true;
}

bool ChMulticoreVisualizationVSG::IsAnchor(unsigned int body) {
    // Flags of bodies added since the last call (anchor sizes always come from the collision model)
    const auto& bodies = m_sys->GetBodies();
    while (m_is_anchor.size() <= body && m_is_anchor.size() < bodies.size()) {
        double size = vsg3d::ChInstancedShapesVSG::GetBodySize(*bodies[m_is_anchor.size()], vsg3d::ChInstancedShapesVSG::ShapeSource::COLLISION);
        m_is_anchor.push_back(size > 0 && size <= m_size_limit);
    }
    return body < m_is_anchor.size() && m_is_anchor[body];
}

// -----------------------------------------------------------------------------

void ChMulticoreVisualizationVSG::OnAttach() {}

void ChMulticoreVisualizationVSG::OnInitialize() {
    // Attach the Multicore system, if not already attached
    auto& systems = m_vsys->GetSystems();
    if (std::find(systems.begin(), systems.end(), m_sys) == systems.end())
        m_vsys->AttachSystem(m_sys);

    m_vsys->AddGuiComponent(chrono_types::make_shared<MulticoreStatsVSG>(this));
}

void ChMulticoreVisualizationVSG::OnBindAssets() {
    if (m_vsys->IsInitialized()) {
        std::cerr << "ChMulticoreVisualizationVSG: the plugin must be attached before the visual system is initialized." << std::endl;
        return;
    }

    const auto& bodies = m_sys->GetBodies();

    // Median size of the bodies active at initialization, and size limit of granular bodies
    std::vector<double> sizes;
    for (const auto& body : bodies) {
        double size = vsg3d::ChInstancedShapesVSG::GetBodySize(*body, vsg3d::ChInstancedShapesVSG::ShapeSource::COLLISION);
        if (size > 0 && body->IsActive())
            sizes.push_back(size);
    }
    if (!sizes.empty()) {
        auto mid = sizes.begin() + sizes.size() / 2;
        std::nth_element(sizes.begin(), mid, sizes.end());
        m_median_size = *mid;
    }
    m_size_limit = m_size_factor * m_median_size;
    m_is_anchor.clear();
    if (!bodies.empty())
        IsAnchor(static_cast<unsigned int>(bodies.size() - 1));

    // Granular bodies
    if (m_use_instancing) {
        m_shapes.SetVisibilityCallback(chrono_types::make_shared<MulticoreBodyVisibility>(this));
        m_shapes.Initialize(*m_vsys, bodies);
    }

    // Force network
    unsigned int capacity = m_capacity;
    if (capacity == 0)
        capacity = static_cast<unsigned int>(std::clamp<size_t>(12 * bodies.size(), 1024, 2000000));
    m_network = chrono_types::make_unique<vsg3d::ChForceNetworkVSG>(capacity);
    m_network->SetMode(m_network_mode);
    m_network->SetVisibility(m_network_visible);
    m_network->Initialize(*m_vsys);
}

void ChMulticoreVisualizationVSG::OnRender() {
    // Request the image of the previous frame, which the visual system writes during this call to Render()
    if (m_write_images && m_num_frames > 0 && (m_num_frames - 1) % m_image_interval == 0) {
        if (m_num_images == 0) {
            std::error_code ec;
            std::filesystem::create_directories(m_image_dir, ec);
        }
        std::ostringstream filename;
        filename << m_image_dir << "/img_" << std::setw(5) << std::setfill('0') << m_num_images << "." << m_image_format;
        m_vsys->WriteImageToFile(filename.str());
        m_num_images++;
    }
    m_num_frames++;

    if (m_use_instancing)
        m_shapes.Update();

    UpdateNetwork();
}

// -----------------------------------------------------------------------------

void ChMulticoreVisualizationVSG::UpdateNetwork() {
    m_num_contacts_drawn = 0;
    if (!m_network)
        return;

    m_network->SetMode(m_network_mode);
    m_network->SetVisibility(m_network_visible);
    if (!m_network_visible)
        return;

    auto dm = m_sys->data_manager;
    const auto& cd = *dm->cd_data;
    const auto& host = dm->host_data;
    const auto num_bodies = static_cast<unsigned int>(m_sys->GetBodies().size());
    const unsigned int nc = cd.num_rigid_contacts;

    // Flags of bodies added after initialization
    if (num_bodies > 0)
        IsAnchor(num_bodies - 1);

    // Normal and tangential contact forces
    m_fn.assign(nc, 0.0);
    m_ft.assign(nc, 0.0);
    bool nsc = m_sys->GetContactMethod() == ChContactMethod::NSC;
    if (nsc) {
        const auto& gamma = host.gamma;
        double h = dm->settings.step_size;
        auto mode = dm->settings.solver.local_solver_mode;
        bool tangential = (mode == SolverMode::SLIDING || mode == SolverMode::SPINNING) && gamma.size() >= 3 * nc;
        if (gamma.size() >= nc && h > 0) {
            for (unsigned int i = 0; i < nc; i++) {
                m_fn[i] = gamma[i] / h;
                if (tangential)
                    m_ft[i] = std::hypot(gamma[nc + 2 * i], gamma[nc + 2 * i + 1]) / h;
            }
        }
    } else if (host.ct_force.size() >= 2 * nc) {
        // Force on the second body, along the contact normal (which points from the first body to the second)
        for (unsigned int i = 0; i < nc; i++) {
            const real3& force = host.ct_force[2 * i + 1];
            const real3& normal = cd.norm_rigid_rigid[i];
            real fn = Dot(force, normal);
            m_fn[i] = fn;
            m_ft[i] = Length(force - fn * normal);
        }
    }

    // Force threshold, possibly relative to the mean normal force over contacts with a positive normal force
    double threshold = m_threshold;
    if (m_relative_threshold) {
        double sum = 0;
        unsigned int count = 0;
        for (unsigned int i = 0; i < nc; i++) {
            if (m_fn[i] > 0) {
                sum += m_fn[i];
                count++;
            }
        }
        threshold = (count > 0) ? m_threshold * sum / count : 0;
    }

    double max_length = (m_max_half_length > 0) ? m_max_half_length : m_median_size;
    float radius = static_cast<float>((m_radius > 0) ? m_radius : 0.1 * m_median_size);

    // Contact history (SMC, MultiStep), stored on the body with the larger index for each pair of shapes
    bool history = !nsc && dm->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep &&
                   host.shear_neigh.size() >= max_shear * static_cast<size_t>(num_bodies) && cd.contact_shapeIDs.size() >= nc;
    auto stored_displacement = [&](unsigned int i) {
        int b1 = cd.bids_rigid_rigid[i].x;
        int b2 = cd.bids_rigid_rigid[i].y;
        int s1 = int(cd.contact_shapeIDs[i] >> 32);
        int s2 = int(cd.contact_shapeIDs[i] & 0xffffffff);
        int body1 = std::max(b1, b2);
        int body2 = std::min(b1, b2);
        int shape1 = std::max(s1, s2);
        int shape2 = std::min(s1, s2);
        for (int k = 0; k < max_shear; k++) {
            size_t index = max_shear * static_cast<size_t>(body1) + k;
            const vec3& neighbor = host.shear_neigh[index];
            if (neighbor.x == body2 && neighbor.y == shape1 && neighbor.z == shape2)
                return static_cast<double>(Length(host.shear_disp[index]));
        }
        return 0.0;
    };

    bool shape_centers = m_anchor_type == NetworkAnchor::SHAPE_CENTER && cd.contact_shapeIDs.size() >= nc && cd.shape_data.obj_data_A_global.size() >= cd.num_rigid_shapes;
    bool friction = host.fric_rigid_rigid.size() >= nc;

    m_points1.clear();
    m_points2.clear();
    m_values.clear();
    m_radii.clear();
    double value_max = 0;

    auto add_segment = [&](const real3& a, const real3& b, double value, float r) {
        m_points1.insert(m_points1.end(), {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)});
        m_points2.insert(m_points2.end(), {static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(b.z)});
        m_values.push_back(static_cast<float>(value));
        m_radii.push_back(r);
    };

    for (unsigned int i = 0; i < nc; i++) {
        double fn = m_fn[i];
        if (fn <= 0 || fn <= threshold)
            continue;

        // Contact point, midway between the points on the two shapes
        real3 cp = (cd.cpta_rigid_rigid[i] + cd.cptb_rigid_rigid[i]) * real(0.5);
        if (!InRegion(ChVector3d(cp.x, cp.y, cp.z)))
            continue;

        // Start points of the two half-segments
        int b1 = cd.bids_rigid_rigid[i].x;
        int b2 = cd.bids_rigid_rigid[i].y;
        real3 c1;
        real3 c2;
        if (shape_centers) {
            int s1 = int(cd.contact_shapeIDs[i] >> 32);
            int s2 = int(cd.contact_shapeIDs[i] & 0xffffffff);
            if (static_cast<int>(cd.shape_data.id_rigid[s1]) != b1)
                std::swap(s1, s2);
            c1 = cd.shape_data.obj_data_A_global[s1];
            c2 = cd.shape_data.obj_data_A_global[s2];
        } else {
            c1 = host.pos_rigid[b1];
            c2 = host.pos_rigid[b2];
        }
        bool draw1 = IsAnchor(b1) && Length(cp - c1) <= max_length;
        bool draw2 = IsAnchor(b2) && Length(c2 - cp) <= max_length;
        if (!draw1 && !draw2)
            continue;

        double value = 0;
        switch (m_coloring) {
            case NetworkColoring::NORMAL_FORCE:
                value = fn;
                break;
            case NetworkColoring::COULOMB_RATIO: {
                double mu = friction ? host.fric_rigid_rigid[i].x : 0;
                value = (mu > 0) ? m_ft[i] / (mu * fn) : 0;
                break;
            }
            case NetworkColoring::TANGENTIAL_DISPLACEMENT:
                value = history ? stored_displacement(i) : 0;
                break;
        }
        value_max = std::max(value_max, value);

        float r = radius;
        if (m_scale_radius) {
            double t = (m_fmax > m_fmin) ? std::clamp((fn - m_fmin) / (m_fmax - m_fmin), 0.0, 1.0) : 1.0;
            r = static_cast<float>(m_rmin + t * (m_rmax - m_rmin));
        }

        if (draw1)
            add_segment(c1, cp, value, r);
        if (draw2)
            add_segment(cp, c2, value, r);
        m_num_contacts_drawn++;
    }

    // Range of values mapped to colors (from 0 to the largest value, if no range was set)
    ChVector2d range = m_ranges[static_cast<int>(m_coloring)];
    if (range.y() <= range.x())
        range = ChVector2d(0, value_max > 0 ? value_max : 1);
    m_current_range = range;

    if (m_use_colormap)
        m_network->SetColormap(m_colormap_type);
    else
        m_network->SetColorRamp(m_color_min, m_color_max);
    m_network->SetValueRange(range.x(), range.y());
    m_network->SetRadius(radius);

    m_network->SetSegments(static_cast<unsigned int>(m_values.size()), m_points1.data(), m_points2.data(), m_values.data(), nullptr, m_scale_radius ? m_radii.data() : nullptr);
}

}  // namespace chrono
