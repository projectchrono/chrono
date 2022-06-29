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

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_fsi/ChVisualizationFsi.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

#ifdef CHRONO_OPENGL
    #include "chrono_opengl/ChOpenGLWindow.h"
#endif

namespace chrono {
namespace fsi {

ChVisualizationFsi::ChVisualizationFsi(ChSystemFsi* sysFSI)
    : m_systemFSI(sysFSI),
      m_user_system(nullptr),
      m_title(""),
      m_cam_pos(0, -3, 0),
      m_cam_target(0, 0, 0),
      m_cam_up(0, 0, 1),
      m_cam_scale(0.1f),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_bce_start_index(0) {
    m_radius = sysFSI->GetInitialSpacing() / 2;
#ifdef CHRONO_OPENGL
    m_system = new ChSystemSMC();
#else
    std::cout << "\nWARNING! Chrono::OpenGL not available.  Visualization disabled!\n" << std::endl;
#endif
}

ChVisualizationFsi::~ChVisualizationFsi() {
    delete m_system;
}

void ChVisualizationFsi::SetCameraPosition(const ChVector<>& pos, const ChVector<>& target) {
    m_cam_pos = pos;
    m_cam_target = target;
}

void ChVisualizationFsi::SetCameraUpVector(const ChVector<>& up) {
    m_cam_up = up;
}

void ChVisualizationFsi::SetCameraMoveScale(float scale) {
    m_cam_scale = scale;
}

void ChVisualizationFsi::SetVisualizationRadius(double radius) {
    m_radius = radius;
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

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.AttachSystem(m_system);
    if (m_user_system)
        gl_window.AttachSystem(m_user_system);
    gl_window.Initialize(1280, 720, m_title.c_str());
    gl_window.SetCamera(m_cam_pos, m_cam_target, m_cam_up, m_cam_scale);
    gl_window.SetRenderMode(opengl::WIREFRAME);
    gl_window.SetParticleRenderMode(m_radius, opengl::POINTS);
#endif
}

bool ChVisualizationFsi::Render() {
#ifdef CHRONO_OPENGL
    // Only for display in OpenGL window
    m_system->SetChTime(m_systemFSI->GetSimTime());

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    if (gl_window.Active()) {
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

        gl_window.Render();
        return true;
    }
    return false;  // rendering stopped
#else
    return false;
#endif
}

}  // namespace fsi
}  // namespace chrono
