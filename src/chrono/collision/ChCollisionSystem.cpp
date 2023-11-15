// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

ChCollisionSystem::ChCollisionSystem() : m_system(nullptr), m_initialized(false) {}

ChCollisionSystem::~ChCollisionSystem() {
    if (m_system)
        m_system->collision_system = nullptr;
}

void ChCollisionSystem::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionSystem>();
}

void ChCollisionSystem::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionSystem>();
}

}  // namespace chrono
