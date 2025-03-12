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

// -----------------------------------------------------------------------------

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

unsigned int ChVisualSystem::GetNumBodies() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumBodiesActive();

    return count;
}

unsigned int ChVisualSystem::GetNumLinks() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumLinksActive();

    return count;
}

unsigned int ChVisualSystem::GetNumMeshes() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumMeshes();

    return count;
}

unsigned int ChVisualSystem::GetNumShafts() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumShafts();

    return count;
}

unsigned int ChVisualSystem::GetNumStates() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumCoordsVelLevel();

    return count;
}

unsigned int ChVisualSystem::GetNumConstraints() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumConstraints();

    return count;
}

unsigned int ChVisualSystem::GetNumContacts() const {
    unsigned int count = 0;
    for (const auto& sys : m_systems)
        count += sys->GetNumContacts();

    return count;
}

}  // namespace chrono
