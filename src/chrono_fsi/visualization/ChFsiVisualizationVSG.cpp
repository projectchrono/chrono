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

#include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

namespace chrono {
namespace fsi {

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSIStatsVSG : public vsg3d::ChGuiComponentVSG {
  public:
    FSIStatsVSG(ChFsiVisualizationVSG* vsysFSI) : m_vsysFSI(vsysFSI) {}

    virtual void render() override {
        char label[64];
        int nstr = sizeof(label) - 1;

        ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
        ImGui::Begin("SPH");

        if (ImGui::BeginTable("SPH", 2, ImGuiTableFlags_BordersOuter | ImGuiTableFlags_SizingFixedFit,
                              ImVec2(0.0f, 0.0f))) {
            ImGui::TableNextColumn();
            ImGui::Text("SPH particles:");
            ImGui::TableNextColumn();
            snprintf(label, nstr, "%lu", static_cast<unsigned long>(m_vsysFSI->m_systemFSI->GetNumFluidMarkers()));
            ImGui::Text(label);

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Boundary BCE:");
            ImGui::TableNextColumn();
            snprintf(label, nstr, "%lu", static_cast<unsigned long>(m_vsysFSI->m_systemFSI->GetNumBoundaryMarkers()));
            ImGui::Text(label);

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Rigid body BCE:");
            ImGui::TableNextColumn();
            snprintf(label, nstr, "%lu", static_cast<unsigned long>(m_vsysFSI->m_systemFSI->GetNumRigidBodyMarkers()));
            ImGui::Text(label);

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Flex body BCE:");
            ImGui::TableNextColumn();
            snprintf(label, nstr, "%lu", static_cast<unsigned long>(m_vsysFSI->m_systemFSI->GetNumFlexBodyMarkers()));
            ImGui::Text(label);

            ImGui::EndTable();
        }

        ImGui::End();
    }

  private:
    ChFsiVisualizationVSG* m_vsysFSI;
};

// -----------------------------------------------------------------------------

ChFsiVisualizationVSG::ChFsiVisualizationVSG(ChSystemFsi* sysFSI, bool verbose) : ChFsiVisualization(sysFSI) {
    m_vsys = new vsg3d::ChVisualSystemVSG();
    m_vsys->SetVerbose(verbose);
    m_vsys->AttachSystem(m_system);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetWireFrameMode(true);
    m_vsys->AddCamera(ChVector<>(0, -3, 0), ChVector<>(0, 0, 0));
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

void ChFsiVisualizationVSG::AddCamera(const ChVector<>& pos, const ChVector<>& target) {
    m_vsys->AddCamera(pos, target);
}

void ChFsiVisualizationVSG::UpdateCamera(const ChVector<>& pos, const ChVector<>& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChFsiVisualizationVSG::SetCameraVertical(CameraVerticalDir up) {
    m_vsys->SetCameraVertical(up);
}

void ChFsiVisualizationVSG::SetCameraMoveScale(float scale) {
    //// TOOD
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
        m_sph_cloud->SetFixed(false);
        for (int i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
            m_sph_cloud->AddParticle(CSYSNULL);
        }
        auto sph = chrono_types::make_shared<ChVisualShapeSphere>(m_systemFSI->GetInitialSpacing() / 2);
        sph->SetColor(ChColor(0.10f, 0.40f, 0.65f));
        m_sph_cloud->AddVisualShape(sph);
        m_sph_cloud->RegisterColorCallback(m_color_fun);
        m_system->Add(m_sph_cloud);
    }

    if (m_bndry_bce_markers) {
        m_bndry_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_bndry_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_systemFSI->GetNumBoundaryMarkers(); i++) {
            m_bndry_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sph = chrono_types::make_shared<ChVisualShapeSphere>(m_systemFSI->GetInitialSpacing() / 4);
        sph->SetColor(m_bndry_bce_color);
        m_bndry_bce_cloud->AddVisualShape(sph);
        m_system->Add(m_bndry_bce_cloud);
    }

    if (m_rigid_bce_markers) {
        m_rigid_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_rigid_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_systemFSI->GetNumRigidBodyMarkers(); i++) {
            m_rigid_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sph = chrono_types::make_shared<ChVisualShapeSphere>(m_systemFSI->GetInitialSpacing() / 4);
        sph->SetColor(m_rigid_bce_color);
        m_rigid_bce_cloud->AddVisualShape(sph);
        m_system->Add(m_rigid_bce_cloud);
    }

    if (m_flex_bce_markers) {
        m_flex_bce_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_flex_bce_cloud->SetFixed(false);
        for (int i = 0; i < m_systemFSI->GetNumFlexBodyMarkers(); i++) {
            m_flex_bce_cloud->AddParticle(CSYSNULL);
        }
        auto sph = chrono_types::make_shared<ChVisualShapeSphere>(m_systemFSI->GetInitialSpacing() / 4);
        sph->SetColor(m_flex_bce_color);
        m_flex_bce_cloud->AddVisualShape(sph);
        m_system->Add(m_flex_bce_cloud);
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
    // For display in VSG GUI
    m_system->SetChTime(m_systemFSI->GetSimTime());
    m_system->SetRTF(m_systemFSI->GetRTF());

    if (m_vsys->Run()) {
        // Copy SPH particle positions from device to host
        thrust::host_vector<Real4> posH = m_systemFSI->m_sysFSI->sphMarkersD2->posRadD;

        // List of proxy bodies
        ////const auto& blist = m_system->Get_bodylist();

        size_t p = 0;

        if (m_sph_markers) {
            for (unsigned int i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
                m_sph_cloud->GetParticle(i).SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_systemFSI->GetNumFluidMarkers();

        if (m_bndry_bce_markers) {
            for (unsigned int i = 0; i < m_systemFSI->GetNumBoundaryMarkers(); i++) {
                m_bndry_bce_cloud->GetParticle(i).SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_systemFSI->GetNumBoundaryMarkers();

        if (m_rigid_bce_markers) {
            for (unsigned int i = 0; i < m_systemFSI->GetNumRigidBodyMarkers(); i++) {
                m_rigid_bce_cloud->GetParticle(i).SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_systemFSI->GetNumRigidBodyMarkers();

        if (m_flex_bce_markers) {
            for (unsigned int i = 0; i < m_systemFSI->GetNumFlexBodyMarkers(); i++) {
                m_flex_bce_cloud->GetParticle(i).SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }

        m_vsys->Render();
        return true;
    }

    return false;  // rendering stopped
}

}  // namespace fsi
}  // namespace chrono
