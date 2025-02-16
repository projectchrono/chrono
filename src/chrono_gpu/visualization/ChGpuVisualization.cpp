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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_gpu/visualization/ChGpuVisualization.h"

namespace chrono {
namespace gpu {

ChGpuVisualization::ChGpuVisualization(ChSystemGpu* sysGPU) : m_systemGPU(sysGPU), m_user_system(nullptr) {
    m_system = new ChSystemSMC();
}

ChGpuVisualization::~ChGpuVisualization() {
    delete m_system;
}

void ChGpuVisualization::SetSize(int width, int height) {}

void ChGpuVisualization::SetTitle(const std::string& title) {}

void ChGpuVisualization::AddCamera(const ChVector3d& pos, const ChVector3d& target) {}

void ChGpuVisualization::UpdateCamera(const ChVector3d& pos, const ChVector3d& target) {}

void ChGpuVisualization::SetCameraVertical(CameraVerticalDir up) {}

void ChGpuVisualization::SetCameraMoveScale(float scale) {}

void ChGpuVisualization::AddProxyBody(std::shared_ptr<ChBody> body) {
    body->SetFixed(true);
    m_system->AddBody(body);
}

void ChGpuVisualization::Initialize() {}

}  // namespace gpu
}  // namespace chrono
