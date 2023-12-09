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
// Authors: Radu Serban
// =============================================================================

#include "chrono/collision/ChCollisionShapeMeshTriangle.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionShapeMeshTriangle)
CH_UPCASTING(ChCollisionShapeMeshTriangle, ChCollisionShape)

ChCollisionShapeMeshTriangle::ChCollisionShapeMeshTriangle() : ChCollisionShape(Type::MESHTRIANGLE) {}

ChCollisionShapeMeshTriangle::ChCollisionShapeMeshTriangle(
    std::shared_ptr<ChMaterialSurface> material,  // contact material
    ChVector<>* V1,                               // vertex1 coords
    ChVector<>* V2,                               // vertex2 coords
    ChVector<>* V3,                               // vertex3 coords
    ChVector<>* eP1,                              // neighboring vertex at edge1 if any
    ChVector<>* eP2,                              // neighboring vertex at edge2 if any
    ChVector<>* eP3,                              // neighboring vertex at edge3 if any
    bool ownsV1,                                  // vertex1 owned by this triangle (otherwise, owned by neighbor)
    bool ownsV2,                                  // vertex2 owned by this triangle (otherwise, owned by neighbor)
    bool ownsV3,                                  // vertex3 owned by this triangle (otherwise, owned by neighbor)
    bool ownsE1,                                  // edge1 owned by this triangle (otherwise, owned by neighbor)
    bool ownsE2,                                  // edge2 owned by this triangle (otherwise, owned by neighbor)
    bool ownsE3,                                  // edge3 owned by this triangle (otherwise, owned by neighbor)
    double sphere_radius                          // radius of swept sphere
    )
    : ChCollisionShape(Type::MESHTRIANGLE, material) {
    this->V1 = V1;
    this->V2 = V2;
    this->V3 = V3;
    this->eP1 = eP1;
    this->eP2 = eP2;
    this->eP3 = eP3;
    this->ownsV1 = ownsV1;
    this->ownsV2 = ownsV2;
    this->ownsV3 = ownsV3;
    this->ownsE1 = ownsE1;
    this->ownsE2 = ownsE2;
    this->ownsE3 = ownsE3;
    this->sradius = sphere_radius;
}

void ChCollisionShapeMeshTriangle::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShapeMeshTriangle>();
    // serialize parent class
    ChCollisionShape::ArchiveOut(marchive);
    // serialize all member data:
    //// TODO
}

void ChCollisionShapeMeshTriangle::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShapeMeshTriangle>();
    // deserialize parent class
    ChCollisionShape::ArchiveIn(marchive);
    // stream in all member data:
    //// TODO
}

}  // end namespace chrono
