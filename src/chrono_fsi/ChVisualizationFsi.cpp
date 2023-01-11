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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_fsi/ChVisualizationFsi.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

namespace chrono {
namespace fsi {

ChVisualizationFsi::ChVisualizationFsi(ChSystemFsi* sysFSI)
    : m_systemFSI(sysFSI),
      m_owns_vis(true),
      m_user_system(nullptr),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_bce_start_index(0) {
    m_radius = sysFSI->GetInitialSpacing() / 2;
#ifdef CHRONO_OPENGL
    m_system = new ChSystemSMC();

    m_vsys = new opengl::ChVisualSystemOpenGL();
    m_vsys->AttachSystem(m_system);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetCameraProperties(0.1f);
    m_vsys->SetRenderMode(opengl::WIREFRAME);
    m_vsys->SetParticleRenderMode(sysFSI->GetInitialSpacing() / 2, opengl::POINTS);
    m_vsys->AddCamera(ChVector<>(0, -3, 0), ChVector<>(0, 0, 0));
    m_vsys->SetCameraVertical(ChVector<>(0, 0, 1));
    m_vsys->EnableStats(false);
#else
    m_system = nullptr;
    std::cout << "\nWARNING! Chrono::OpenGL not available.  Visualization disabled!\n" << std::endl;
#endif
}

#ifdef CHRONO_OPENGL
ChVisualizationFsi::ChVisualizationFsi(ChSystemFsi* sysFSI, opengl::ChVisualSystemOpenGL* vis)
    : m_systemFSI(sysFSI),
      m_owns_vis(false),
      m_user_system(nullptr),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_bce_start_index(0) {
    m_radius = sysFSI->GetInitialSpacing() / 2;
    m_system = new ChSystemSMC();

    m_vsys = vis;
    m_vsys->AttachSystem(m_system);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetCameraProperties(0.1f);
    m_vsys->SetRenderMode(opengl::WIREFRAME);
    m_vsys->SetParticleRenderMode(sysFSI->GetInitialSpacing() / 2, opengl::POINTS);
    m_vsys->AddCamera(ChVector<>(0, -3, 0), ChVector<>(0, 0, 0));
    m_vsys->SetCameraVertical(ChVector<>(0, 0, 1));
    m_vsys->EnableStats(false);
}
#endif

ChVisualizationFsi::~ChVisualizationFsi() {
    delete m_system;
#ifdef CHRONO_OPENGL
    if (m_owns_vis)
        delete m_vsys;
#endif
}

void ChVisualizationFsi::SetSize(int width, int height) {
#ifdef CHRONO_OPENGL
    m_vsys->SetWindowSize(width, height);
#endif
}

void ChVisualizationFsi::SetTitle(const std::string& title) {
#ifdef CHRONO_OPENGL
    m_vsys->SetWindowTitle("");
#endif
}

void ChVisualizationFsi::UpdateCamera(const ChVector<>& pos, const ChVector<>& target) {
#ifdef CHRONO_OPENGL
    m_vsys->UpdateCamera(pos, target);
#endif
}

void ChVisualizationFsi::SetCameraUpVector(const ChVector<>& up) {
#ifdef CHRONO_OPENGL
    m_vsys->SetCameraVertical(up);
#endif
}

void ChVisualizationFsi::SetCameraMoveScale(float scale) {
#ifdef CHRONO_OPENGL
    m_vsys->SetCameraProperties(scale);
#endif
}

void ChVisualizationFsi::SetParticleRenderMode(double radius, RenderMode mode) {
#ifdef CHRONO_OPENGL
    opengl::RenderMode gl_mode = (mode == RenderMode::SOLID) ? opengl::SOLID : opengl::POINTS;
    m_vsys->SetParticleRenderMode(radius, gl_mode);
#endif
}

void ChVisualizationFsi::SetRenderMode(RenderMode mode) {
#ifdef CHRONO_OPENGL
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
#endif
}

void ChVisualizationFsi::EnableInfoOverlay(bool val) {
#ifdef CHRONO_OPENGL
    m_vsys->EnableStats(val);
#endif
}

void ChVisualizationFsi::AddProxyBody(std::shared_ptr<ChBody> body) {
    body->SetBodyFixed(true);
#ifdef CHRONO_OPENGL
    m_system->AddBody(body);
#endif
}

void ChVisualizationFsi::Initialize() {
#ifdef CHRONO_OPENGL
    // Cache current number of bodies (if any) in m_system
    m_bce_start_index = static_cast<unsigned int>(m_system->Get_bodylist().size());

    if (m_sph_markers) {
        m_particles = chrono_types::make_shared<ChParticleCloud>();
        m_particles->SetFixed(true);
        for (int i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
            m_particles->AddParticle(ChCoordsys<>(ChVector<>(i * 0.01, 0, 0)));
        }
        auto sph = chrono_types::make_shared<ChSphereShape>();
        sph->GetSphereGeometry().rad = m_radius;
        m_particles->AddVisualShape(sph);
        m_system->Add(m_particles);
    }

    if (m_rigid_bce_markers) {
        for (int i = 0; i < m_systemFSI->GetNumRigidBodyMarkers(); i++) {
            auto body = std::shared_ptr<ChBody>(m_system->NewBody());
            body->SetPos(ChVector<>(0, 0, 0));
            body->SetBodyFixed(true);
            auto sph = chrono_types::make_shared<ChBoxShape>();
            sph->GetBoxGeometry().Size = ChVector<>(m_radius);
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }
    }

    if (m_flex_bce_markers) {
        for (int i = 0; i < m_systemFSI->GetNumFlexBodyMarkers(); i++) {
            auto body = std::shared_ptr<ChBody>(m_system->NewBody());
            body->SetPos(ChVector<>(0, 0, 0));
            body->SetBodyFixed(true);
            auto sph = chrono_types::make_shared<ChBoxShape>();
            sph->GetBoxGeometry().Size = ChVector<>(m_radius);
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }   
    }

    if (m_bndry_bce_markers) {
        for (int i = 0; i < m_systemFSI->GetNumBoundaryMarkers(); i++) {
            auto body = std::shared_ptr<ChBody>(m_system->NewBody());
            body->SetPos(ChVector<>(0, 0, 0));
            body->SetBodyFixed(true);
            auto sph = chrono_types::make_shared<ChBoxShape>();
            sph->GetBoxGeometry().Size = ChVector<>(m_radius);
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }
    }

    if (m_owns_vis) {
        if (m_user_system)
            m_vsys->AttachSystem(m_user_system);
        m_vsys->Initialize();
    }
#endif
}

bool ChVisualizationFsi::Render() {
#ifdef CHRONO_OPENGL
    // Only for display in OpenGL window
    m_system->SetChTime(m_systemFSI->GetSimTime());

    if (m_vsys->Run()) {
        // Copy SPH particle positions from device to host
        thrust::host_vector<Real4> posH = m_systemFSI->m_sysFSI->sphMarkersD2->posRadD;

        // List of proxy bodies
        const auto& blist = m_system->Get_bodylist();

        size_t p = 0;
        size_t b = 0;

        if (m_sph_markers) {
            for (unsigned int i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
                m_particles->GetParticle(i).SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
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
#else
    return true;
#endif
}

}  // namespace fsi
}  // namespace chrono
