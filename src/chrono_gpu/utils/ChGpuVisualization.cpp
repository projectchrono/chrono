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

#include "chrono_gpu/utils/ChGpuVisualization.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

namespace chrono {
namespace gpu {

ChGpuVisualization::ChGpuVisualization(ChSystemGpu* sysGPU, ChSystem* sys)
    : m_systemGPU(sysGPU),
      m_system(nullptr),
      m_owns_sys(false),
      m_title(""),
      m_cam_pos(0, -3, 0),
      m_cam_target(0, 0, 0),
      m_cam_up(0, 0, 1),
      m_cam_scale(0.1f),
      m_part_start_index(0) {
#ifdef CHRONO_OPENGL
    if (!sys) {
        m_system = new ChSystemSMC();
        m_owns_sys = true;
    } else {
        m_system = sys;
        m_owns_sys = false;
    }
#else
    std::cout << "\nWARNING! Chrono::OpenGL not available.  Visualization disabled!\n" << std::endl;
#endif
}

ChGpuVisualization::~ChGpuVisualization() {
    if (m_owns_sys)
        delete m_system;
}

void ChGpuVisualization::SetCameraPosition(const ChVector<>& pos, const ChVector<>& target) {
    m_cam_pos = pos;
    m_cam_target = target;
}

void ChGpuVisualization::SetCameraUpVector(const ChVector<>& up) {
    m_cam_up = up;
}

void ChGpuVisualization::SetCameraMoveScale(float scale) {
    m_cam_scale = scale;
}

void ChGpuVisualization::AddProxyBody(std::shared_ptr<ChBody> body) {
    body->SetBodyFixed(true);
#ifdef CHRONO_OPENGL
    m_system->AddBody(body);
#endif
}

void ChGpuVisualization::Initialize() {
#ifdef CHRONO_OPENGL
    // Cache current number of bodies (if any) in m_system
    m_part_start_index = static_cast<unsigned int>(m_system->Get_bodylist().size());

    // Note: we cannot yet use ChSystemGpu::GetParticlePosition!
    for (int i = 0; i < m_systemGPU->GetNumParticles(); i++) {
        auto body = std::shared_ptr<ChBody>(m_system->NewBody());
        body->SetPos(ChVector<>(0, 0, 0));
        body->SetBodyFixed(true);
        auto sph = chrono_types::make_shared<ChSphereShape>();
        sph->GetSphereGeometry().rad = m_systemGPU->GetParticleRadius();
        body->AddVisualShape(sph);
        m_system->AddBody(body);
    }

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, m_title.c_str(), m_system);
    gl_window.SetCamera(m_cam_pos, m_cam_target, m_cam_up, m_cam_scale);
    gl_window.SetRenderMode(opengl::WIREFRAME);
#endif
}

bool ChGpuVisualization::Render() {
#ifdef CHRONO_OPENGL
    if (m_owns_sys) {
        // Only for display in OpenGL window
        m_system->SetChTime(m_systemGPU->GetSimTime());
    }

    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    if (gl_window.Active()) {
        const auto& blist = m_system->Get_bodylist();
        for (unsigned int i = 0; i < m_systemGPU->GetNumParticles(); i++) {
            auto pos = m_systemGPU->GetParticlePosition(i);
            blist[m_part_start_index + i]->SetPos(pos);
        }
        gl_window.Render();
        return false;
    }
    return true;  // rendering stopped
#else
    return false;
#endif
}

}  // namespace gpu
}  // namespace chrono
