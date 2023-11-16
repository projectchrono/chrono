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
#include "chrono/fea/ChMesh.h"

namespace chrono {

ChCollisionSystem::ChCollisionSystem() : m_system(nullptr), m_initialized(false) {}

ChCollisionSystem::~ChCollisionSystem() {
    if (m_system)
        m_system->collision_system = nullptr;
}

void ChCollisionSystem::Initialize() {
    if (m_initialized)
        return;

    BindAll();

    m_initialized = true;
}

void ChCollisionSystem::BindAll() {
    if (!m_system)
        return;

    BindAssembly(&m_system->GetAssembly());
}

void ChCollisionSystem::BindItem(std::shared_ptr<ChPhysicsItem> item) {
    if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
        Add(body->GetCollisionModel());
    }

    if (auto mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
        for (const auto& surf : mesh->GetContactSurfaces()) {
            surf->AddCollisionModelsToSystem(this);
        }
    }

    if (const auto& cloud = std::dynamic_pointer_cast<ChParticleCloud>(item))
        BindParticleCloud(cloud.get());

    if (const auto& a = std::dynamic_pointer_cast<ChAssembly>(item)) {
        BindAssembly(a.get());
    }
}

void ChCollisionSystem::BindAssembly(const ChAssembly* assembly) {
    for (const auto& body : assembly->Get_bodylist()) {
        if (body->GetCollide())
            Add(body->GetCollisionModel());
    }

    for (const auto& mesh : assembly->Get_meshlist()) {
        for (const auto& surf : mesh->GetContactSurfaces()) {
            surf->AddCollisionModelsToSystem(this);
        }
    }

    for (const auto& item : assembly->Get_otherphysicslist()) {
        if (const auto& cloud = std::dynamic_pointer_cast<ChParticleCloud>(item))
            BindParticleCloud(cloud.get());

        if (const auto& a = std::dynamic_pointer_cast<ChAssembly>(item))
            BindAssembly(a.get());
    }
}

void ChCollisionSystem::BindParticleCloud(const ChParticleCloud* cloud) {
    if (!cloud->GetCollide())
        return;

    for (const auto& p : cloud->GetParticles()) {
        Add(p->GetCollisionModel());
    }
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
