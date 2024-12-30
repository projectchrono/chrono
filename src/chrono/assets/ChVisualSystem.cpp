// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#include "chrono/assets/ChVisualSystem.h"

namespace chrono {

ChVisualSystem ::ChVisualSystem() : m_initialized(false), m_verbose(false), m_write_images(false), m_image_dir(".") {}

ChVisualSystem ::~ChVisualSystem() {
    for (auto s : m_systems)
        s->visual_system = nullptr;
}

void ChVisualSystem::AttachSystem(ChSystem* sys) {
    m_systems.push_back(sys);
    sys->visual_system = this;
}

void ChVisualSystem::UpdateCamera(int id, const ChVector3d& pos, ChVector3d target) {
    SetCameraPosition(id, pos);
    SetCameraTarget(id, target);
}

void ChVisualSystem::UpdateCamera(const ChVector3d& pos, ChVector3d target) {
    SetCameraPosition(pos);
    SetCameraTarget(target);
}

double ChVisualSystem::GetSimulationRTF() const {
    if (m_systems.empty())
        return 0;
    return m_systems[0]->GetRTF();
}

double ChVisualSystem::GetSimulationTime() const {
    if (m_systems.empty())
        return 0;
    return m_systems[0]->GetChTime();
}

}  // namespace chrono
