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

#include <algorithm>

#include "chrono/assets/ChVisualSystem.h"

namespace chrono {

ChVisualSystem ::ChVisualSystem()
    : m_initialized(false),
      m_background_color(ChColor(0.10f, 0.20f, 0.30f)),
      m_verbose(false),
      m_rtf(0),
      m_write_images(false),
      m_image_dir(".") {}

ChVisualSystem ::~ChVisualSystem() {
    for (auto s : m_systems)
        s->visual_system = nullptr;
}

void ChVisualSystem::AttachSystem(ChSystem* sys) {
    // Attach provided system only if not already done
    if (std::find(m_systems.begin(), m_systems.end(), sys) == m_systems.end()) {
        m_systems.push_back(sys);
        sys->visual_system = this;
    }
}

void ChVisualSystem::SetBackgroundColor(const ChColor& color) {
    m_background_color = color;
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

void ChVisualSystem::Render() {
    static double t_last = 0;
    double t_curr = GetSimulationTime();
    m_timer.stop();
    if (t_curr > t_last)
        m_rtf = m_timer() / (t_curr - t_last);
    t_last = t_curr;
    m_timer.reset();
    m_timer.start();
}

// -----------------------------------------------------------------------------

double ChVisualSystem::GetSimulationRTF(unsigned int i) const {
    if (i >= m_systems.size())
        return 0;

    return m_systems[i]->GetRTF();
}

std::vector<double> ChVisualSystem::GetSimulationRTFs() const {
    std::vector<double> rtf(m_systems.size());
    for (size_t i = 0; i < m_systems.size(); i++)
        rtf[i] = m_systems[i]->GetRTF();
    return rtf;
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
