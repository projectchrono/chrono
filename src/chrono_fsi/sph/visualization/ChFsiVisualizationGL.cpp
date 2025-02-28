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

#include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {

using namespace sph;

// -----------------------------------------------------------------------------

// Custom stats overlay
class FSIStatsGL : public opengl::ChOpenGLStats {
  public:
    FSIStatsGL() : ChOpenGLStats() {}

    virtual void GenerateStats(ChSystem& sys) override {
        text.Render("SPH particles:  " + std::to_string(num_sph), screen.LEFT, screen.TOP - 1 * screen.SPACING,
                    screen.SX, screen.SY);

        text.Render("Boundary BCE:   " + std::to_string(num_bndry_bce), screen.LEFT, screen.TOP - 2 * screen.SPACING,
                    screen.SX, screen.SY);

        text.Render("Rigid body BCE: " + std::to_string(num_rigid_bce), screen.LEFT, screen.TOP - 3 * screen.SPACING,
                    screen.SX, screen.SY);

        text.Render("Flex body BCE:  " + std::to_string(num_flex_bce), screen.LEFT, screen.TOP - 4 * screen.SPACING,
                    screen.SX, screen.SY);

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

ChFsiVisualizationGL::ChFsiVisualizationGL(ChFsiSystemSPH* sysFSI)
    : ChFsiVisualization(sysFSI), m_bce_start_index(0) {
    m_vsys = new opengl::ChVisualSystemOpenGL();
    m_vsys->AttachSystem(m_sysMBS);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetCameraProperties(0.1f);
    m_vsys->SetRenderMode(opengl::WIREFRAME);
    m_vsys->SetParticleRenderMode(opengl::SOLID, (float)m_sysSPH->GetInitialSpacing() / 2);
    m_vsys->AddCamera(ChVector3d(0, -3, 0), ChVector3d(0, 0, 0));
    m_vsys->SetCameraVertical(ChVector3d(0, 0, 1));
}

ChFsiVisualizationGL::ChFsiVisualizationGL(ChFluidSystemSPH* sysSPH) : ChFsiVisualization(sysSPH), m_bce_start_index(0) {
    m_vsys = new opengl::ChVisualSystemOpenGL();
    m_vsys->AttachSystem(m_sysMBS);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetCameraProperties(0.1f);
    m_vsys->SetRenderMode(opengl::WIREFRAME);
    m_vsys->SetParticleRenderMode(opengl::SOLID, (float)m_sysSPH->GetInitialSpacing() / 2);
    m_vsys->AddCamera(ChVector3d(0, -3, 0), ChVector3d(0, 0, 0));
    m_vsys->SetCameraVertical(ChVector3d(0, 0, 1));
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

void ChFsiVisualizationGL::AddCamera(const ChVector3d& pos, const ChVector3d& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChFsiVisualizationGL::UpdateCamera(const ChVector3d& pos, const ChVector3d& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChFsiVisualizationGL::SetCameraVertical(CameraVerticalDir up) {
    if (up == CameraVerticalDir::Z)
        m_vsys->SetCameraVertical(ChVector3d(0, 0, 1));
    m_vsys->SetCameraVertical(ChVector3d(0, 1, 0));
}

void ChFsiVisualizationGL::SetCameraMoveScale(float scale) {
    m_vsys->SetCameraProperties(scale);
}

void ChFsiVisualizationGL::SetParticleRenderMode(RenderMode mode) {
    opengl::RenderMode gl_mode = (mode == RenderMode::SOLID) ? opengl::SOLID : opengl::POINTS;
    m_vsys->SetParticleRenderMode(gl_mode, (float)m_sysSPH->GetInitialSpacing() / 2);
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
    // Cache current number of bodies (if any) in m_sysMBS
    m_bce_start_index = static_cast<unsigned int>(m_sysMBS->GetBodies().size());

    if (m_sph_markers) {
        m_sph_cloud = chrono_types::make_shared<ChParticleCloud>();
        m_sph_cloud->SetFixed(true);
        for (int i = 0; i < m_sysSPH->GetNumFluidMarkers(); i++) {
            m_sph_cloud->AddParticle(CSYSNULL);
        }
        auto sph = chrono_types::make_shared<ChVisualShapeSphere>(m_sysSPH->GetInitialSpacing() / 2);
        m_sph_cloud->AddVisualShape(sph);
        m_sysMBS->Add(m_sph_cloud);
    }

    if (m_bndry_bce_markers) {
        for (int i = 0; i < m_sysSPH->GetNumBoundaryMarkers(); i++) {
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos(ChVector3d(0, 0, 0));
            body->SetFixed(true);
            auto sph = chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(m_sysSPH->GetInitialSpacing() / 2));
            body->AddVisualShape(sph);
            m_sysMBS->AddBody(body);
        }
    }

    if (m_rigid_bce_markers) {
        for (int i = 0; i < m_sysSPH->GetNumRigidBodyMarkers(); i++) {
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos(ChVector3d(0, 0, 0));
            body->SetFixed(true);
            auto sph = chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(m_sysSPH->GetInitialSpacing() / 2));
            body->AddVisualShape(sph);
            m_sysMBS->AddBody(body);
        }
    }

    if (m_flex_bce_markers) {
        for (int i = 0; i < m_sysSPH->GetNumFlexBodyMarkers(); i++) {
            auto body = chrono_types::make_shared<ChBody>();
            body->SetPos(ChVector3d(0, 0, 0));
            body->SetFixed(true);
            auto sph = chrono_types::make_shared<ChVisualShapeBox>(ChVector3d(m_sysSPH->GetInitialSpacing() / 2));
            body->AddVisualShape(sph);
            m_sysMBS->AddBody(body);
        }
    }

    if (m_user_system)
        m_vsys->AttachSystem(m_user_system);
    m_vsys->Initialize();

    auto fsi_stats = chrono_types::make_shared<FSIStatsGL>();
    fsi_stats->num_sph = static_cast<unsigned long>(m_sysSPH->GetNumFluidMarkers());
    fsi_stats->num_bndry_bce = static_cast<unsigned long>(m_sysSPH->GetNumBoundaryMarkers());
    fsi_stats->num_rigid_bce = static_cast<unsigned long>(m_sysSPH->GetNumRigidBodyMarkers());
    fsi_stats->num_flex_bce = static_cast<unsigned long>(m_sysSPH->GetNumFlexBodyMarkers());
    m_vsys->AttachStatsRenderer(fsi_stats);
}

bool ChFsiVisualizationGL::Render() {
    // For display in OpenGL GUI
    if (m_sysFSI) {
        m_sysMBS->SetChTime(m_sysFSI->GetSimTime());
        m_sysMBS->SetRTF(m_sysFSI->GetRtf());
    } else {
        m_sysMBS->SetChTime(m_sysSPH->GetSimTime());
        m_sysMBS->SetRTF(m_sysSPH->GetRtf());    
    }

    if (m_vsys->Run()) {
        // Copy SPH particle positions from device to host
        std::vector<Real3> posH = m_sysSPH->GetPositions();

        // List of proxy bodies
        const auto& blist = m_sysMBS->GetBodies();

        size_t p = 0;
        size_t b = 0;

        if (m_sph_markers) {
            for (unsigned int i = 0; i < m_sysSPH->GetNumFluidMarkers(); i++) {
                m_sph_cloud->Particle(i).SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_sysSPH->GetNumFluidMarkers();

        if (m_bndry_bce_markers) {
            for (size_t i = 0; i < m_sysSPH->GetNumBoundaryMarkers(); i++) {
                blist[m_bce_start_index + b++]->SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_sysSPH->GetNumBoundaryMarkers();

        if (m_rigid_bce_markers) {
            for (size_t i = 0; i < m_sysSPH->GetNumRigidBodyMarkers(); i++) {
                blist[m_bce_start_index + b++]->SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }
        p += m_sysSPH->GetNumRigidBodyMarkers();

        if (m_flex_bce_markers) {
            for (size_t i = 0; i < m_sysSPH->GetNumFlexBodyMarkers(); i++) {
                blist[m_bce_start_index + b++]->SetPos(ChVector3d(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }

        m_vsys->Render();
        return true;
    }

    return false;  // rendering stopped
}

}  // namespace fsi
}  // namespace chrono
