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

#include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {

using namespace sph;

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSIStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    FSIStatsVSG(ChFsiVisualizationVSG* vsysFSI) : m_vsysFSI(vsysFSI) {}

    virtual void render() override {
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
            ImGui::TextUnformatted(m_vsysFSI->m_sysSPH->GetSphMethodTypeString().c_str());

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

        ImGui::End();
    }

  private:
    ChFsiVisualizationVSG* m_vsysFSI;
};

// -----------------------------------------------------------------------------

ChFsiVisualizationVSG::ChFsiVisualizationVSG(ChFsiSystemSPH* sysFSI) : ChFsiVisualization(sysFSI) {
    m_vsys = new vsg3d::ChVisualSystemVSG();
    m_vsys->AttachSystem(m_sysMBS);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetWireFrameMode(true);
    m_vsys->AddCamera(ChVector3d(0, -3, 0), ChVector3d(0, 0, 0));
    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
    m_vsys->SetUseSkyBox(false);
    m_vsys->SetClearColor(ChColor(18.0f / 255, 26.0f / 255, 32.0f / 255));
}

ChFsiVisualizationVSG::ChFsiVisualizationVSG(ChFluidSystemSPH* sysSPH) : ChFsiVisualization(sysSPH) {
    m_vsys = new vsg3d::ChVisualSystemVSG();
    m_vsys->AttachSystem(m_sysMBS);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetWireFrameMode(true);
    m_vsys->AddCamera(ChVector3d(0, -3, 0), ChVector3d(0, 0, 0));
    m_vsys->SetCameraVertical(CameraVerticalDir::Z);
    m_vsys->SetUseSkyBox(false);
    m_vsys->SetClearColor(ChColor(18.0f / 255, 26.0f / 255, 32.0f / 255));
}

ChFsiVisualizationVSG::~ChFsiVisualizationVSG() {
    delete m_vsys;
}

void ChFsiVisualizationVSG::SetSize(int width, int height) {
    m_vsys->SetWindowSize(width, height);
}

void ChFsiVisualizationVSG::SetTitle(const std::string& title) {
    m_vsys->SetWindowTitle(title);
}

void ChFsiVisualizationVSG::AddCamera(const ChVector3d& pos, const ChVector3d& target) {
    m_vsys->AddCamera(pos, target);
}

void ChFsiVisualizationVSG::UpdateCamera(const ChVector3d& pos, const ChVector3d& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChFsiVisualizationVSG::SetCameraVertical(CameraVerticalDir up) {
    m_vsys->SetCameraVertical(up);
}

void ChFsiVisualizationVSG::SetCameraMoveScale(float scale) {
    //// TOOD
}

void ChFsiVisualizationVSG::SetLightIntensity(double intensity) {
    m_vsys->SetLightIntensity(intensity);
}

void ChFsiVisualizationVSG::SetLightDirection(double azimuth, double elevation) {
    m_vsys->SetLightDirection(azimuth, elevation);
}

void ChFsiVisualizationVSG::SetParticleRenderMode(RenderMode mode) {
    //// TODO
}

void ChFsiVisualizationVSG::SetRenderMode(RenderMode mode) {
    m_vsys->SetWireFrameMode(mode == RenderMode::WIREFRAME);
}

void ChFsiVisualizationVSG::EnableInfoOverlay(bool val) {
    ////m_vsys->EnableStats(val);
}

void ChFsiVisualizationVSG::SetUseSkyBox(bool val) {
    m_vsys->SetUseSkyBox(val);
}

void ChFsiVisualizationVSG::SetClearColor(const ChColor& color) {
    m_vsys->SetClearColor(color);
}

void ChFsiVisualizationVSG::Initialize() {
    if (m_sph_markers) {
        m_sph_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_sph_cloud->SetName("sph_particles");
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
    }

    if (m_bndry_bce_markers) {
        m_bndry_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_bndry_bce_cloud->SetName("bce_boundary");
        m_bndry_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumBoundaryMarkers(); i++) {
            m_bndry_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_bndry_bce_color);
        m_bndry_bce_cloud->AddVisualShape(sphere);
        m_bndry_bce_cloud->RegisterVisibilityCallback(m_vis_bndry_fun);
        m_sysMBS->Add(m_bndry_bce_cloud);
    }

    if (m_rigid_bce_markers) {
        m_rigid_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_rigid_bce_cloud->SetName("bce_rigid");
        m_rigid_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumRigidBodyMarkers(); i++) {
            m_rigid_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_rigid_bce_color);
        m_rigid_bce_cloud->AddVisualShape(sphere);
        m_sysMBS->Add(m_rigid_bce_cloud);
    }

    if (m_flex_bce_markers) {
        m_flex_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_flex_bce_cloud->SetName("bce_flex");
        m_flex_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_sysSPH->GetNumFlexBodyMarkers(); i++) {
            m_flex_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 4);
        sphere->SetColor(m_flex_bce_color);
        m_flex_bce_cloud->AddVisualShape(sphere);
        m_sysMBS->Add(m_flex_bce_cloud);
    }

    auto fsi_states = chrono_types::make_shared<FSIStatsVSG>(this);
    m_vsys->AddGuiComponent(fsi_states);

    if (m_user_system)
        m_vsys->AttachSystem(m_user_system);

    m_vsys->SetImageOutput(m_write_images);
    m_vsys->SetImageOutputDirectory(m_image_dir);

    m_vsys->Initialize();
}

bool ChFsiVisualizationVSG::Render() {
    if (!m_vsys->Run())
        return false;

    // Copy SPH particle positions from device to host
    thrust::host_vector<Real3> posH = m_sysSPH->GetPositions();
    thrust::host_vector<Real3> velH = m_sysSPH->GetVelocities();
    ////thrust::host_vector<Real3> accH = m_sysSPH->GetAccelerations();
    ////thrust::host_vector<Real3> frcH = m_sysSPH->GetForces();
    thrust::host_vector<Real3> propH = m_sysSPH->GetProperties();

    // Set members for the callback functors (if defined)
    if (m_color_fun) {
        m_color_fun->pos = posH.data();
        m_color_fun->vel = velH.data();
        m_color_fun->prop = propH.data();
    }
    if (m_vis_sph_fun) {
        m_vis_sph_fun->pos = posH.data();
    }
    if (m_vis_bndry_fun) {
        const auto n = m_sysSPH->GetNumFluidMarkers();
        m_vis_bndry_fun->pos = &posH.data()[n];
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
            m_sph_cloud->Particle(i).SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            ////m_sph_cloud->Particle(i).SetPosDt(ChVector3d(velH[p + i].x, velH[p + i].y, velH[p + i].z));
        }
    }
    p += m_sysSPH->GetNumFluidMarkers();

    if (m_bndry_bce_markers) {
        for (unsigned int i = 0; i < m_sysSPH->GetNumBoundaryMarkers(); i++) {
            m_bndry_bce_cloud->Particle(i).SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
        }
    }
    p += m_sysSPH->GetNumBoundaryMarkers();

    if (m_rigid_bce_markers) {
        for (unsigned int i = 0; i < m_sysSPH->GetNumRigidBodyMarkers(); i++) {
            m_rigid_bce_cloud->Particle(i).SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
        }
    }
    p += m_sysSPH->GetNumRigidBodyMarkers();

    if (m_flex_bce_markers) {
        for (unsigned int i = 0; i < m_sysSPH->GetNumFlexBodyMarkers(); i++) {
            m_flex_bce_cloud->Particle(i).SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
        }
    }

    m_vsys->Render();

    return true;
}

}  // namespace fsi
}  // namespace chrono
