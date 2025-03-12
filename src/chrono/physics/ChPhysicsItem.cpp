// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPhysicsItem)

ChPhysicsItem::ChPhysicsItem(const ChPhysicsItem& other) : ChObj(other) {
    // Do not copy the system; this is initialized at insertion time
    system = NULL;
    offset_x = other.offset_x;
    offset_w = other.offset_w;
    offset_L = other.offset_L;
}

ChPhysicsItem::~ChPhysicsItem() {
    SetSystem(NULL);  // note that this might remove collision model from system
}

void ChPhysicsItem::SetSystem(ChSystem* m_system) {
    system = m_system;
}

ChAABB ChPhysicsItem::GetTotalAABB() const {
    return ChAABB();
}

ChVector3d ChPhysicsItem::GetCenter() const {
    auto bbox = GetTotalAABB();
    return (bbox.min + bbox.max) * 0.5;
}

void ChPhysicsItem::Update(double time, bool update_assets) {
    ChObj::Update(time, update_assets);
}

void ChPhysicsItem::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChPhysicsItem>();

    // serialize parent class
    ChObj::ArchiveOut(archive_out);

    // serialize all member data:
    // archive_out << CHNVP(system); ***TODO***
    // archive_out << CHNVP(offset_x);
    // archive_out << CHNVP(offset_w);
    // archive_out << CHNVP(offset_L);
}

/// Method to allow de serialization of transient data from archives.
void ChPhysicsItem::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChPhysicsItem>();

    // deserialize parent class
    ChObj::ArchiveIn(archive_in);

    // stream in all member data:
    // archive_in >> CHNVP(system); ***TODO***
    // archive_in >> CHNVP(offset_x);
    // archive_in >> CHNVP(offset_w);
    // archive_in >> CHNVP(offset_L);
}

}  // end namespace chrono
