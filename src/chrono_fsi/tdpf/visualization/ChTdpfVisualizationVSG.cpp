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

#include "chrono_fsi/tdpf/visualization/ChTdpfVisualizationVSG.h"

#include "chrono_vsg/utils/ChDataUtilsVSG.h"
#include "chrono_vsg/utils/ChShapeBuilderVSG.h"
#include "chrono_vsg/impl/VSGvisitors.h"

namespace chrono {
namespace fsi {
namespace tdpf {

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSITDPFStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    FSITDPFStatsVSG(ChTdpfVisualizationVSG* vsysFSI) : m_vsysFSI(vsysFSI) {}

    virtual void render(vsg::CommandBuffer& cb) override {
        ////vsg3d::ChVisualSystemVSG& vsys = m_vsysFSI->GetVisualSystemVSG();

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("TDPF");

        if (ImGui::BeginTable("TDPF_STATS", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("FSI bodies:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysTDPF->m_num_rigid_bodies));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("FSI 1D meshes:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysTDPF->m_num_1D_meshes));

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("FSI 2D meshes:");
            ImGui::TableNextColumn();
            ImGui::Text("%lu", static_cast<unsigned long>(m_vsysFSI->m_sysTDPF->m_num_2D_meshes));

            ImGui::TableNextRow();

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
                ImGui::Text("%8.1e", m_vsysFSI->m_sysTDPF->GetStepSize());
            }

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("RTF (fluid):");
            ImGui::TableNextColumn();
            ImGui::Text("%8.3f", m_vsysFSI->m_sysTDPF->GetRtf());

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

        if (ImGui::BeginTable("TDPF_VIS", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            static bool waves_visible = m_vsysFSI->m_waves_visible;
            if (ImGui::Checkbox("Render water surface", &waves_visible)) {
                m_vsysFSI->m_waves_visible = !m_vsysFSI->m_waves_visible;
                m_vsysFSI->SetWaveMeshVisibility(m_vsysFSI->m_waves_visible);
            }

            ImGui::EndTable();
        }

        ImGui::End();
    }

  private:
    ChTdpfVisualizationVSG* m_vsysFSI;
};

// ---------------------------------------------------------------------------

ChTdpfVisualizationVSG::ChTdpfVisualizationVSG(ChFsiSystemTDPF* sysFSI)
    : ChTdpfVisualizationVSG(&sysFSI->GetFluidSystemTDPF()) {
    m_sysFSI = sysFSI;
}

ChTdpfVisualizationVSG::ChTdpfVisualizationVSG(ChFsiFluidSystemTDPF* sysTDPF)
    : m_sysFSI(nullptr),
      m_sysTDPF(sysTDPF),
      m_waves_visible(false),
      m_update_freq(20),
      m_last_update(0),
      m_write_images(false),
      m_image_dir(".") {
    m_sysMBS = new ChSystemSMC("FSI_internal_system");
    m_wave_scene = vsg::Switch::create();
}

ChTdpfVisualizationVSG::~ChTdpfVisualizationVSG() {
    auto& systems = m_vsys->GetSystems();
    auto index = std::find(systems.begin(), systems.end(), m_sysMBS);
    if (index != systems.end())
        systems.erase(index);

    delete m_sysMBS;
}

void ChTdpfVisualizationVSG::SetWaveMeshParams(const ChVector2d& center, const ChVector2d& size, double resolution) {
    m_wave_mesh.center = center;
    m_wave_mesh.size = size;
    m_wave_mesh.resolution = resolution;
}

void ChTdpfVisualizationVSG::SetWaveMeshWireframe(bool wireframe) {
    m_wave_mesh.wireframe = wireframe;
}

void ChTdpfVisualizationVSG::SetWaveMeshColorMode(ColorMode mode, const ChVector2d& range) {
    m_wave_mesh.colormode = mode;
    m_wave_mesh.range = range;
}

std::string ChTdpfVisualizationVSG::GetWaveMeshColorModeAsString(ColorMode mode) {
    switch (mode) {
        case ColorMode::HEIGHT:
            return "Height";
        case ColorMode::VELOCITY_MAG:
            return "Velocity magnitude";
        case ColorMode::VELOCITY_X:
            return "Velocityy component X";
        case ColorMode::VELOCITY_Y:
            return "Velocityy component Y";
        case ColorMode::VELOCITY_Z:
            return "Velocityy component Z";
    }
    return "None";
}

void ChTdpfVisualizationVSG::SetWaveMeshColormap(ChColormap::Type type, float opacity) {
    m_wave_mesh.colormap_type = type;
    m_wave_mesh.opacity = opacity;
    if (m_wave_mesh.colormap) {
        m_wave_mesh.colormap->Load(type);
    }
}

void ChTdpfVisualizationVSG::SetWaveMeshUpdateFrequency(double freq) {
    m_update_freq = freq;
}

ChColormap::Type ChTdpfVisualizationVSG::GetColormapType() const {
    return m_wave_mesh.colormap_type;
}

const ChColormap& ChTdpfVisualizationVSG::GetColormap() const {
    return *m_wave_mesh.colormap;
}

void ChTdpfVisualizationVSG::OnAttach() {
    m_vsys->AttachSystem(m_sysMBS);

    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
}

void ChTdpfVisualizationVSG::OnInitialize() {
    // Create colormap
    m_wave_mesh.colormap = chrono_types::make_unique<ChColormap>(m_wave_mesh.colormap_type);

    // Create custom GUI for the FSI plugin
    auto fsi_states = chrono_types::make_shared<FSITDPFStatsVSG>(this);
    m_vsys->AddGuiComponent(fsi_states);

    // Add colorbar GUI
    if (m_waves_visible) {
        m_wave_colorbar_id =
            m_vsys->AddGuiColorbar("waves", m_wave_mesh.range, m_wave_mesh.colormap_type, false, 400.0f);
    }

    m_vsys->SetImageOutput(m_write_images);
    m_vsys->SetImageOutputDirectory(m_image_dir);
}

void ChTdpfVisualizationVSG::CreateWaveMesh() {
    m_wave_mesh.trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    std::vector<ChVector3d>& vertices = m_wave_mesh.trimesh->GetCoordsVertices();
    std::vector<ChVector3d>& normals = m_wave_mesh.trimesh->GetCoordsNormals();
    std::vector<ChVector3i>& idx_vertices = m_wave_mesh.trimesh->GetIndicesVertexes();
    std::vector<ChVector3i>& idx_normals = m_wave_mesh.trimesh->GetIndicesNormals();
    std::vector<ChVector2d>& uv_coords = m_wave_mesh.trimesh->GetCoordsUV();
    std::vector<ChColor>& colors = m_wave_mesh.trimesh->GetCoordsColors();

    // Resize mesh arrays
    int n = m_wave_mesh.resolution;
    int n_verts = n * n;
    int n_faces = 2 * (n - 1) * (n - 1);
    double uv_scale = 1.0 / n;

    vertices.resize(n_verts);
    normals.resize(n_verts);
    uv_coords.resize(n_verts);
    colors.resize(n_verts);
    idx_vertices.resize(n_faces);
    idx_normals.resize(n_faces);

    // Create mesh vertices (in x-y plane, centered at origin)
    ChVector2d delta = m_wave_mesh.size / (n - 1);
    int iv = 0;
    for (int iy = 0; iy < n; iy++) {
        double y = iy * delta.y() - 0.5 * m_wave_mesh.size.y();
        for (int ix = 0; ix < n; ix++) {
            double x = ix * delta.x() - 0.5 * m_wave_mesh.size.x();
            vertices[iv] = ChVector3d(m_wave_mesh.center.x() + x, m_wave_mesh.center.y() + y, 0);
            colors[iv] = ChColor(1, 1, 1);
            uv_coords[iv] = ChVector2d(ix * uv_scale, iy * uv_scale);
            ++iv;
        }
    }

    // Specify triangular faces (two at a time)
    // Specify the face vertices counter-clockwise
    // Set the normal indices same as the vertex indices
    int it = 0;
    for (int iy = 0; iy < n - 1; iy++) {
        for (int ix = 0; ix < n - 1; ix++) {
            int v0 = ix + n * iy;
            idx_vertices[it] = ChVector3i(v0, v0 + 1, v0 + n + 1);
            idx_normals[it] = ChVector3i(v0, v0 + 1, v0 + n + 1);
            ++it;
            idx_vertices[it] = ChVector3i(v0, v0 + n + 1, v0 + n);
            idx_normals[it] = ChVector3i(v0, v0 + n + 1, v0 + n);
            ++it;
        }
    }
}

void ChTdpfVisualizationVSG::OnBindAssets() {
    // Create the VSG group for the wave mesh
    m_vsys->GetVSGScene()->addChild(m_wave_scene);

    // Create the trimesh
    CreateWaveMesh();

    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::dmat4CH(ChFramed(), 1);
    auto child = m_vsys->GetVSGShapeBuilder()->CreateTrimeshColShape(m_wave_mesh.trimesh, transform, ChColor(1, 1, 1),
                                                                     m_wave_mesh.opacity, m_wave_mesh.wireframe);
    vsg::Mask mask = m_waves_visible;
    m_wave_scene->addChild(mask, child);

    // Load deformable mesh data (for CPU->GPU transfer)
    m_wave_mesh.mesh_soup = true;

    m_wave_mesh.vertices = vsg::visit<FindVec3BufferData<0>>(child).getBufferData();
    assert(m_wave_mesh.vertices->size() == 3 * m_wave_mesh.trimesh->GetNumTriangles());
    m_wave_mesh.vertices->properties.dataVariance = vsg::DYNAMIC_DATA;
    m_wave_mesh.dynamic_vertices = true;

    if (!m_wave_mesh.wireframe) {
        m_wave_mesh.normals = vsg::visit<FindVec3BufferData<1>>(child).getBufferData();
        assert(m_wave_mesh.normals->size() == m_wave_mesh.vertices->size());
        m_wave_mesh.normals->properties.dataVariance = vsg::DYNAMIC_DATA;
        m_wave_mesh.dynamic_normals = true;
    } else {
        m_wave_mesh.dynamic_normals = false;
    }

    m_wave_mesh.colors = vsg::visit<FindVec4BufferData<3>>(child).getBufferData();
    assert(m_wave_mesh.colors->size() == m_wave_mesh.vertices->size());
    m_wave_mesh.colors->properties.dataVariance = vsg::DYNAMIC_DATA;
    m_wave_mesh.dynamic_colors = true;
}

void ChTdpfVisualizationVSG::SetWaveMeshVisibility(bool val) {
    m_waves_visible = val;

    if (!m_vsys || !m_vsys->IsInitialized())
        return;

    m_vsys->GetGuiComponent(m_wave_colorbar_id)->SetVisibility(val);

    for (auto& child : m_wave_scene->children) {
        child.mask = val;
    }
}

void ChTdpfVisualizationVSG::OnRender() {
    // For display in VSG GUI
    if (m_sysFSI) {
        m_sysMBS->SetChTime(m_sysFSI->GetSimTime());
        m_sysMBS->SetRTF(m_sysFSI->GetRtf());
    } else {
        m_sysMBS->SetChTime(m_sysTDPF->GetSimTime());
        m_sysMBS->SetRTF(m_sysTDPF->GetRtf());
    }

    if (!m_waves_visible)
        return;

    double time = m_sysTDPF->GetSimTime();
    if ((time - m_last_update) < 1 / m_update_freq)
        return;

    m_last_update = time;

    // Update trimesh vertex heights and colors based on wave elevation
    for (size_t iv = 0; iv < m_wave_mesh.trimesh->GetNumVertices(); iv++) {
        auto& v = m_wave_mesh.trimesh->GetCoordsVertices()[iv];
        
        auto height = m_sysTDPF->GetWaveElevation(v.eigen());
        v.z() = height;

        if (m_wave_mesh.colormode != ColorMode::NONE) {
            ChVector3d vel = VNULL;
            if (m_wave_mesh.colormode != ColorMode::HEIGHT)
                vel = ChVector3d(m_sysTDPF->GetWaveVelocity(v.eigen(), height));
            
            ChColor color;
            switch (m_wave_mesh.colormode) {
                case ColorMode::HEIGHT:
                    color = m_wave_mesh.colormap->Get(height, m_wave_mesh.range[0], m_wave_mesh.range[1]);
                    break;
                case ColorMode::VELOCITY_MAG:
                    color = m_wave_mesh.colormap->Get(vel.Length(), m_wave_mesh.range[0], m_wave_mesh.range[1]);
                    break;
                case ColorMode::VELOCITY_X:
                    color = m_wave_mesh.colormap->Get(vel.x(), m_wave_mesh.range[0], m_wave_mesh.range[1]);
                    break;
                case ColorMode::VELOCITY_Y:
                    color = m_wave_mesh.colormap->Get(vel.y(), m_wave_mesh.range[0], m_wave_mesh.range[1]);
                    break;
                case ColorMode::VELOCITY_Z:
                    color = m_wave_mesh.colormap->Get(vel.z(), m_wave_mesh.range[0], m_wave_mesh.range[1]);
                    break;
            }
            m_wave_mesh.trimesh->GetCoordsColors()[iv] = color;
        }
    }

    // Dynamic data transfer CPU->GPU for wave mesh
    if (m_wave_mesh.dynamic_vertices) {
        const auto& new_vertices =
            m_wave_mesh.mesh_soup ? m_wave_mesh.trimesh->getFaceVertices() : m_wave_mesh.trimesh->GetCoordsVertices();
        assert(m_wave_mesh.vertices->size() == new_vertices.size());

        const size_t count = new_vertices.size();
        if (count > 0) {
            // ChVector3d stores 3 doubles contiguously, cast to raw double* and float* with less overhead
            const double* src_ptr = new_vertices[0].data();
            float* dst_ptr = reinterpret_cast<float*>(m_wave_mesh.vertices->data());

            // convert 3*count doubles to floats with tight loop
            const size_t total_components = count * 3;
            for (size_t i = 0; i < total_components; ++i) {
                dst_ptr[i] = static_cast<float>(src_ptr[i]);
            }

            m_wave_mesh.vertices->dirty();
        }
    }

    if (m_wave_mesh.dynamic_normals) {
        const auto& new_normals =
            m_wave_mesh.mesh_soup ? m_wave_mesh.trimesh->getFaceNormals() : m_wave_mesh.trimesh->getAverageNormals();
        assert(m_wave_mesh.normals->size() == new_normals.size());

        const size_t count = new_normals.size();
        if (count > 0) {
            const double* src_ptr = new_normals[0].data();
            float* dst_ptr = reinterpret_cast<float*>(m_wave_mesh.normals->data());

            const size_t total_components = count * 3;
            for (size_t i = 0; i < total_components; ++i) {
                dst_ptr[i] = static_cast<float>(src_ptr[i]);
            }

            m_wave_mesh.normals->dirty();
        }
    }

    if (m_wave_mesh.dynamic_colors) {
        const auto& new_colors =
            m_wave_mesh.mesh_soup ? m_wave_mesh.trimesh->getFaceColors() : m_wave_mesh.trimesh->GetCoordsColors();
        assert(m_wave_mesh.colors->size() == new_colors.size());

        const size_t count = new_colors.size();
        if (count > 0) {
            // ChColor is 12 bytes (3 floats), but need to give to the gpu with vec4 (16 bytes) for alignment
            // copy element-wise with manual unroll
            const ChColor* src_ptr = new_colors.data();
            float* dst_ptr = reinterpret_cast<float*>(m_wave_mesh.colors->data());

            // Manual unroll (RGBA = 4 components)
            for (size_t k = 0; k < count; ++k) {
                const size_t idx = k * 4;
                dst_ptr[idx + 0] = src_ptr[k].R;
                dst_ptr[idx + 1] = src_ptr[k].G;
                dst_ptr[idx + 2] = src_ptr[k].B;
                dst_ptr[idx + 3] = 0.4f;  // Alpha channel (ChColor has no transparency)
            }
            m_wave_mesh.colors->dirty();
        }
    }
}

}  // namespace tdpf
}  // namespace fsi
}  // namespace chrono
