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

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

namespace chrono {
namespace fsi {

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSIStatsGL : public opengl::ChOpenGLStats {
  public:
    FSIStatsGL() : ChOpenGLStats() {}
    virtual void GenerateStats(ChSystem& sys) override {
        sprintf(buffer, "SPH particles:  %lu", num_sph);
        text.Render(buffer, screen.LEFT, screen.TOP - 1 * screen.SPACING, screen.SX, screen.SY);

        sprintf(buffer, "Boundary BCE:   %lu", num_bndry_bce);
        text.Render(buffer, screen.LEFT, screen.TOP - 2 * screen.SPACING, screen.SX, screen.SY);

        sprintf(buffer, "Rigid body BCE: %lu", num_rigid_bce);
        text.Render(buffer, screen.LEFT, screen.TOP - 3 * screen.SPACING, screen.SX, screen.SY);

        sprintf(buffer, "Flex body BCE:  %lu", num_flex_bce);
        text.Render(buffer, screen.LEFT, screen.TOP - 4 * screen.SPACING, screen.SX, screen.SY);

        sprintf(buffer, "TIME: %04f", sys.GetChTime());
        text.Render(buffer, screen.LEFT, screen.TOP - 6 * screen.SPACING, screen.SX, screen.SY);

        sprintf(buffer, "FPS:  %04d", int(fps));
        text.Render(buffer, screen.LEFT, screen.TOP - 7 * screen.SPACING, screen.SX, screen.SY);
    }
    unsigned long num_sph;
    unsigned long num_bndry_bce;
    unsigned long num_rigid_bce;
    unsigned long num_flex_bce;
    char buffer[50];
};

// -----------------------------------------------------------------------------

ChFsiVisualizationGL::ChFsiVisualizationGL(ChSystemFsi* sysFSI, bool verbose) : ChFsiVisualization(sysFSI), m_bce_start_index(0) {
    m_vsys = new opengl::ChVisualSystemOpenGL();
    m_vsys->SetVerbose(verbose);
    m_vsys->AttachSystem(m_system);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetCameraProperties(0.1f);
    m_vsys->SetRenderMode(opengl::WIREFRAME);
    m_vsys->SetParticleRenderMode(opengl::SOLID, (float)sysFSI->GetInitialSpacing() / 2);
    m_vsys->AddCamera(ChVector<>(0, -3, 0), ChVector<>(0, 0, 0));
    m_vsys->SetCameraVertical(ChVector<>(0, 0, 1));
} 

ChFsiVisualizationGL::~ChFsiVisualizationGL() {
    delete m_vsys;
}

void ChFsiVisualizationGL::SetSize(int width, int height) {
    m_vsys->SetWindowSize(width, height);
}

void ChFsiVisualizationGL::SetTitle(const std::string& title) {
    m_vsys->SetWindowTitle("");
}

void ChFsiVisualizationGL::AddCamera(const ChVector<>& pos, const ChVector<>& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChFsiVisualizationGL::UpdateCamera(const ChVector<>& pos, const ChVector<>& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChFsiVisualizationGL::SetCameraVertical(CameraVerticalDir up) {
    if (up == CameraVerticalDir::Z)
        m_vsys->SetCameraVertical(ChVector<>(0, 0, 1));
    m_vsys->SetCameraVertical(ChVector<>(0, 1, 0));
}

void ChFsiVisualizationGL::SetCameraMoveScale(float scale) {
    m_vsys->SetCameraProperties(scale);
}

void ChFsiVisualizationGL::SetParticleRenderMode(RenderMode mode) {
    opengl::RenderMode gl_mode = (mode == RenderMode::SOLID) ? opengl::SOLID : opengl::POINTS;
    m_vsys->SetParticleRenderMode(gl_mode, (float)m_systemFSI->GetInitialSpacing() / 2);
}

void ChFsiVisualizationGL::SetRenderMode(RenderMode mode) {
    switch (mode) {
        case RenderMode::SOLID:
            m_vsys->SetRenderMode(opengl::SOLID);
            break;
        case RenderMode::WIREFRAME:
            m_vsys->SetRenderMode(opengl::WIREFRAME);
            break;
        case RenderMode::POINTS:
            m_vsys->SetRenderMode(opengl::POINTS);
            break;
    }
}

void ChFsiVisualizationGL::EnableInfoOverlay(bool val) {
    m_vsys->EnableStats(val);
}

void ChFsiVisualizationGL::Initialize() {
    // Cache current number of bodies (if any) in m_system
    m_bce_start_index = static_cast<unsigned int>(m_system->Get_bodylist().size());

    if (m_sph_markers) {
        m_sph_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_sph_cloud->SetFixed(true);
        for (int i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
            m_sph_cloud->AddParticle(CSYSNULL);
        }
        auto sph = chrono_types::make_shared<ChSphereShape>(m_systemFSI->GetInitialSpacing() / 2);
        m_sph_cloud->AddVisualShape(sph);
        m_system->Add(m_sph_cloud);
    }

    if (m_bndry_bce_markers) {
        for (int i = 0; i < m_systemFSI->GetNumBoundaryMarkers(); i++) {
            auto body = std::shared_ptr<ChBody>(m_system->NewBody());
            body->SetPos(ChVector<>(0, 0, 0));
            body->SetBodyFixed(true);
            auto sph = chrono_types::make_shared<ChBoxShape>(ChVector<>(m_systemFSI->GetInitialSpacing() / 2));
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }
    }

    if (m_rigid_bce_markers) {
        for (int i = 0; i < m_systemFSI->GetNumRigidBodyMarkers(); i++) {
            auto body = std::shared_ptr<ChBody>(m_system->NewBody());
            body->SetPos(ChVector<>(0, 0, 0));
            body->SetBodyFixed(true);
            auto sph = chrono_types::make_shared<ChBoxShape>(ChVector<>(m_systemFSI->GetInitialSpacing() / 2));
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }
    }

    if (m_flex_bce_markers) {
        for (int i = 0; i < m_systemFSI->GetNumFlexBodyMarkers(); i++) {
            auto body = std::shared_ptr<ChBody>(m_system->NewBody());
            body->SetPos(ChVector<>(0, 0, 0));
            body->SetBodyFixed(true);
            auto sph = chrono_types::make_shared<ChBoxShape>(ChVector<>(m_systemFSI->GetInitialSpacing() / 2));
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }
    }

    if (m_user_system)
        m_vsys->AttachSystem(m_user_system);
    m_vsys->Initialize();

    auto fsi_stats = chrono_types::make_shared<FSIStatsGL>();
    fsi_stats->num_sph = static_cast<unsigned long>(m_systemFSI->GetNumFluidMarkers());
    fsi_stats->num_bndry_bce = static_cast<unsigned long>(m_systemFSI->GetNumBoundaryMarkers());
    fsi_stats->num_rigid_bce = static_cast<unsigned long>(m_systemFSI->GetNumRigidBodyMarkers());
    fsi_stats->num_flex_bce = static_cast<unsigned long>(m_systemFSI->GetNumFlexBodyMarkers());
    m_vsys->AttachStatsRenderer(fsi_stats);
}

bool ChFsiVisualizationGL::Render() {
    // For display in OpenGL GUI
    m_system->SetChTime(m_systemFSI->GetSimTime());
    m_system->SetRTF(m_systemFSI->GetRTF());

    if (m_vsys->Run()) {
        // Copy SPH particle positions from device to host
        thrust::host_vector<Real4> posH = m_systemFSI->m_sysFSI->sphMarkersD2->posRadD;

        // List of proxy bodies
        const auto& blist = m_system->Get_bodylist();

        size_t p = 0;
        size_t b = 0;

        if (m_sph_markers) {
            for (unsigned int i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
                m_sph_cloud->GetParticle(i).SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_systemFSI->GetNumFluidMarkers();

        if (m_bndry_bce_markers) {
            for (size_t i = 0; i < m_systemFSI->GetNumBoundaryMarkers(); i++) {
                blist[m_bce_start_index + b++]->SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_systemFSI->GetNumBoundaryMarkers();

        if (m_rigid_bce_markers) {
            for (size_t i = 0; i < m_systemFSI->GetNumRigidBodyMarkers(); i++) {
                blist[m_bce_start_index + b++]->SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_systemFSI->GetNumRigidBodyMarkers();

        if (m_flex_bce_markers) {
            for (size_t i = 0; i < m_systemFSI->GetNumFlexBodyMarkers(); i++) {
                blist[m_bce_start_index + b++]->SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }

        m_vsys->Render();
        return true;
    }

    return false;  // rendering stopped
}

}  // namespace fsi
}  // namespace chrono
