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

#include "chrono_fsi/utils/ChFsiVisualization.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

namespace chrono {
namespace fsi {

ChFsiVisualization::ChFsiVisualization(ChSystemFsi* sysFSI)
    : m_systemFSI(sysFSI),
      m_title(""),
      m_cam_pos(0, -3, 0),
      m_cam_target(0, 0, 0),
      m_cam_up(0, 0, 1),
      m_cam_scale(0.1f),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_bndry_bce_markers(false),
      m_part_start_index(0) {
    m_radius = sysFSI->GetInitialSpacing() / 2;
#ifdef CHRONO_OPENGL
    m_system = new ChSystemSMC();
#else
    std::cout << "\nWARNING! Chrono::OpenGL not available.  Visualization disabled!\n" << std::endl;
#endif
}

ChFsiVisualization::~ChFsiVisualization() {
    delete m_system;
}

void ChFsiVisualization::SetCameraPosition(const ChVector<>& pos, const ChVector<>& target) {
    m_cam_pos = pos;
    m_cam_target = target;
}

void ChFsiVisualization::SetCameraUpVector(const ChVector<>& up) {
    m_cam_up = up;
}

void ChFsiVisualization::SetCameraMoveScale(float scale) {
    m_cam_scale = scale;
}

void ChFsiVisualization::SetVisualizationRadius(double radius) {
    m_radius = radius;
}

void ChFsiVisualization::AddProxyBody(std::shared_ptr<ChBody> body) {
    body->SetBodyFixed(true);
#ifdef CHRONO_OPENGL
    m_system->AddBody(body);
#endif
}

void ChFsiVisualization::Initialize() {
#ifdef CHRONO_OPENGL
    // Cache current number of bodies (if any) in m_system
    m_part_start_index = static_cast<unsigned int>(m_system->Get_bodylist().size());

    if (m_sph_markers) {
        for (int i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
            auto body = std::shared_ptr<ChBody>(m_system->NewBody());
            body->SetPos(ChVector<>(0, 0, 0));
            body->SetBodyFixed(true);
            auto sph = chrono_types::make_shared<ChSphereShape>();
            sph->GetSphereGeometry().rad = m_radius;
            body->AddVisualShape(sph);
            m_system->AddBody(body);
        }
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
    gl_window.Initialize(1280, 720, m_title.c_str(), m_system);
    gl_window.SetCamera(m_cam_pos, m_cam_target, m_cam_up, m_cam_scale);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif
}

bool ChFsiVisualization::Render() {
#ifdef CHRONO_OPENGL
    // Only for display in OpenGL window
    m_system->SetChTime(m_systemFSI->GetSimTime());

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    if (gl_window.Active()) {
        // Copy SPH particle positions from device to host
        thrust::host_vector<Real4> posH = m_systemFSI->sysFSI.sphMarkersD2->posRadD;

        // List of proxy bodies
        const auto& blist = m_system->Get_bodylist();

        size_t b = 0;
        size_t p = 0;
        if (m_sph_markers) {
            for (size_t i = 0; i < m_systemFSI->GetNumFluidMarkers(); i++) {
                blist[m_part_start_index + b++]->SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }

        p += m_systemFSI->GetNumFluidMarkers();
        if (m_bndry_bce_markers) {
            for (size_t i = 0; i < m_systemFSI->GetNumBoundaryMarkers(); i++) {
                blist[m_part_start_index + b++]->SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }
        }

        p += m_systemFSI->GetNumBoundaryMarkers();
        if (m_rigid_bce_markers) {
            for (size_t i = 0; i < m_systemFSI->GetNumRigidBodyMarkers(); i++) {
                blist[m_part_start_index + b++]->SetPos(ChVector<>(posH[p + i].x, posH[p + i].y, posH[p + i].z));
            }        
        }

        gl_window.Render();
        return false;
    }
    return true;  // rendering stopped
#else
    return false;
#endif
}

}  // namespace fsi
}  // namespace chrono
