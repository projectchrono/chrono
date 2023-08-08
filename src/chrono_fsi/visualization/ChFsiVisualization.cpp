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
#include "chrono_fsi/visualization/ChFsiVisualization.h"

namespace chrono {
namespace fsi {

ChFsiVisualization::ChFsiVisualization(ChSystemFsi* sysFSI)
    : m_systemFSI(sysFSI),
      m_user_system(nullptr),
      m_sph_markers(true),
      m_rigid_bce_markers(true),
      m_flex_bce_markers(true),
      m_bndry_bce_markers(false),
      m_sph_color(ChColor(0.10f, 0.40f, 0.65f)),
      m_bndry_bce_color(ChColor(0.65f, 0.30f, 0.03f)),
      m_rigid_bce_color(ChColor(0.10f, 0.60f, 0.30f)),
      m_flex_bce_color(ChColor(0.40f, 0.10f, 0.65f)),
      m_write_images(false),
      m_image_dir(".") {
    m_system = new ChSystemSMC();
}

ChFsiVisualization::~ChFsiVisualization() {
    delete m_system;
}

void ChFsiVisualization::SetSize(int width, int height) {}

void ChFsiVisualization::SetTitle(const std::string& title) {}

void ChFsiVisualization::AddCamera(const ChVector<>& pos, const ChVector<>& target) {}

void ChFsiVisualization::UpdateCamera(const ChVector<>& pos, const ChVector<>& target) {}

void ChFsiVisualization::SetCameraVertical(CameraVerticalDir up) {}

void ChFsiVisualization::SetCameraMoveScale(float scale) {}

void ChFsiVisualization::SetParticleRenderMode(RenderMode mode) {}

void ChFsiVisualization::SetRenderMode(RenderMode mode) {}

void ChFsiVisualization::EnableInfoOverlay(bool val) {}

void ChFsiVisualization::AddProxyBody(std::shared_ptr<ChBody> body) {
    body->SetBodyFixed(true);
    m_system->AddBody(body);
}

void ChFsiVisualization::Initialize() {}

}  // namespace fsi
}  // namespace chrono
