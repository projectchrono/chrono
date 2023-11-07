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
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShape)

ChCollisionShape::ChCollisionShape(Type type) : m_type(type), m_material(nullptr) {}

ChCollisionShape::ChCollisionShape(Type type, std::shared_ptr<ChMaterialSurface> material)
    : m_type(type), m_material(material) {}

void ChCollisionShape::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShape>();
    // serialize all member data:
    marchive << CHNVP(m_material);
}

void ChCollisionShape::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShape>();
    // stream in all member data:
    marchive >> CHNVP(m_material);
}

}  // namespace chrono
