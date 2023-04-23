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
    #include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

namespace chrono {
namespace gpu {

ChGpuVisualization::ChGpuVisualization(ChSystemGpu* sysGPU) : m_systemGPU(sysGPU), m_user_system(nullptr) {
#ifdef CHRONO_OPENGL
    m_system = new ChSystemSMC();

    m_vsys = new opengl::ChVisualSystemOpenGL();
    m_vsys->AttachSystem(m_system);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetCameraProperties(0.1f);
    m_vsys->SetRenderMode(opengl::WIREFRAME);
    m_vsys->AddCamera(ChVector<>(0, -3, 0), ChVector<>(0, 0, 0));
    m_vsys->SetCameraVertical(ChVector<>(0, 0, 1));
    m_vsys->EnableStats(false);
#else
    m_system = nullptr;
    std::cout << "\nWARNING! Chrono::OpenGL not available.  Visualization disabled!\n" << std::endl;
#endif
}

ChGpuVisualization::~ChGpuVisualization() {
#ifdef CHRONO_OPENGL
    delete m_vsys;
#endif
    delete m_system;
}

void ChGpuVisualization::SetSize(int width, int height) {
#ifdef CHRONO_OPENGL
    m_vsys->SetWindowSize(width, height);
#endif
}

void ChGpuVisualization::SetTitle(const std::string& title) {
#ifdef CHRONO_OPENGL
    m_vsys->SetWindowTitle("");
#endif
}

void ChGpuVisualization::AddCamera(const ChVector<>& pos, const ChVector<>& target) {
#ifdef CHRONO_OPENGL
    m_vsys->UpdateCamera(pos, target);
#endif
}

void ChGpuVisualization::UpdateCamera(const ChVector<>& pos, const ChVector<>& target) {
#ifdef CHRONO_OPENGL
    m_vsys->UpdateCamera(pos, target);
#endif
}

void ChGpuVisualization::SetCameraVertical(CameraVerticalDir up) {
#ifdef CHRONO_OPENGL
    if (up == CameraVerticalDir::Z)
        m_vsys->SetCameraVertical(ChVector<>(0, 0, 1));
    m_vsys->SetCameraVertical(ChVector<>(0, 1, 0));
#endif
}

void ChGpuVisualization::SetCameraMoveScale(float scale) {
#ifdef CHRONO_OPENGL
    m_vsys->SetCameraProperties(scale);
#endif
}

void ChGpuVisualization::AddProxyBody(std::shared_ptr<ChBody> body) {
    body->SetBodyFixed(true);
#ifdef CHRONO_OPENGL
    m_system->AddBody(body);
#endif
}

void ChGpuVisualization::Initialize() {
#ifdef CHRONO_OPENGL
    m_particles = chrono_types::make_shared<ChParticleCloud>();
    m_particles->SetFixed(true);
    for (int i = 0; i < m_systemGPU->GetNumParticles(); i++) {
        m_particles->AddParticle(CSYSNULL);
    }
    auto sph = chrono_types::make_shared<ChSphereShape>(m_systemGPU->GetParticleRadius());
    m_particles->AddVisualShape(sph);
    m_system->Add(m_particles);

    if (m_user_system)
        m_vsys->AttachSystem(m_user_system);

    m_vsys->Initialize();
#endif
}

bool ChGpuVisualization::Render() {
#ifdef CHRONO_OPENGL
    // Only for display in OpenGL window
    m_system->SetChTime(m_systemGPU->GetSimTime());

    if (m_vsys->Run()) {
        for (unsigned int i = 0; i < m_systemGPU->GetNumParticles(); i++) {
            m_particles->GetParticle(i).SetPos(m_systemGPU->GetParticlePosition(i));
        }
        m_vsys->Render();
        return true;
    }
    return false;  // rendering stopped
#else
    return false;
#endif
}

}  // namespace gpu
}  // namespace chrono
