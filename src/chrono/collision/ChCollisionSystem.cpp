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
#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/physics/ChConveyor.h"
#include "chrono/fea/ChMesh.h"

namespace chrono {

ChCollisionSystem::ChCollisionSystem() : m_system(nullptr), m_initialized(false) {}

ChCollisionSystem::~ChCollisionSystem() {}

void ChCollisionSystem::Initialize() {
    if (m_initialized)
        return;

    BindAll();

    m_initialized = true;
}

void ChCollisionSystem::BindAll() {
    if (!m_system)
        return;

    // Start a (recursive) traversal of all physics items in the system's assembly
    m_system->GetAssembly().AddCollisionModelsToSystem(this);
}

void ChCollisionSystem::BindItem(std::shared_ptr<ChPhysicsItem> item) {
    item->AddCollisionModelsToSystem(this);
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
