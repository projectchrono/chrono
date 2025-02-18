// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
#include "chrono/assets/ChVisualShapeSphere.h"

#include "chrono_gpu/visualization/ChGpuVisualizationGL.h"

namespace chrono {
namespace gpu {

ChGpuVisualizationGL::ChGpuVisualizationGL(ChSystemGpu* sysGPU) : ChGpuVisualization(sysGPU) {
    m_vsys = new opengl::ChVisualSystemOpenGL();
    m_vsys->AttachSystem(m_system);
    m_vsys->SetWindowTitle("");
    m_vsys->SetWindowSize(1280, 720);
    m_vsys->SetCameraProperties(0.1f);
    m_vsys->SetRenderMode(opengl::WIREFRAME);
    m_vsys->AddCamera(ChVector3d(0, -3, 0), ChVector3d(0, 0, 0));
    m_vsys->SetCameraVertical(ChVector3d(0, 0, 1));
    m_vsys->EnableStats(false);
}

ChGpuVisualizationGL::~ChGpuVisualizationGL() {
    delete m_vsys;
}

void ChGpuVisualizationGL::SetSize(int width, int height) {
    m_vsys->SetWindowSize(width, height);
}

void ChGpuVisualizationGL::SetTitle(const std::string& title) {
    m_vsys->SetWindowTitle("");
}

void ChGpuVisualizationGL::AddCamera(const ChVector3d& pos, const ChVector3d& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChGpuVisualizationGL::UpdateCamera(const ChVector3d& pos, const ChVector3d& target) {
    m_vsys->UpdateCamera(pos, target);
}

void ChGpuVisualizationGL::SetCameraVertical(CameraVerticalDir up) {
    if (up == CameraVerticalDir::Z)
        m_vsys->SetCameraVertical(ChVector3d(0, 0, 1));
    m_vsys->SetCameraVertical(ChVector3d(0, 1, 0));
}

void ChGpuVisualizationGL::SetCameraMoveScale(float scale) {
    m_vsys->SetCameraProperties(scale);
}

void ChGpuVisualizationGL::Initialize() {
    m_particles = chrono_types::make_shared<ChParticleCloud>();
    m_particles->SetFixed(true);
    for (unsigned int i = 0; i < m_systemGPU->GetNumParticles(); i++) {
        m_particles->AddParticle(CSYSNULL);
    }
    auto sph = chrono_types::make_shared<ChVisualShapeSphere>(m_systemGPU->GetParticleRadius());
    m_particles->AddVisualShape(sph);
    m_system->Add(m_particles);

    if (m_user_system)
        m_vsys->AttachSystem(m_user_system);

    m_vsys->Initialize();
}

bool ChGpuVisualizationGL::Render() {
    // Only for display in OpenGL window
    m_system->SetChTime(m_systemGPU->GetSimTime());

    if (m_vsys->Run()) {
        for (unsigned int i = 0; i < m_systemGPU->GetNumParticles(); i++) {
            m_particles->Particle(i).SetPos(m_systemGPU->GetParticlePosition(i));
        }
        m_vsys->Render();
        return true;
    }
    return false;  // rendering stopped
}

}  // namespace gpu
}  // namespace chrono
