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


class ChCollisionShape_Type_enum_mapper : public ChCollisionShape {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::SPHERE);
    CH_ENUM_VAL(Type::ELLIPSOID);
    CH_ENUM_VAL(Type::BOX);
    CH_ENUM_VAL(Type::CYLINDER);
    CH_ENUM_VAL(Type::CYLSHELL);
    CH_ENUM_VAL(Type::CONVEXHULL);
    CH_ENUM_VAL(Type::TRIANGLEMESH);
    CH_ENUM_VAL(Type::BARREL);
    CH_ENUM_VAL(Type::POINT);
    CH_ENUM_VAL(Type::TRIANGLE);
    CH_ENUM_VAL(Type::CAPSULE);
    CH_ENUM_VAL(Type::CONE);
    CH_ENUM_VAL(Type::ROUNDEDBOX);
    CH_ENUM_VAL(Type::ROUNDEDCYL);
    CH_ENUM_VAL(Type::TETRAHEDRON);
    CH_ENUM_VAL(Type::PATH2D);
    CH_ENUM_VAL(Type::SEGMENT2D);
    CH_ENUM_VAL(Type::ARC2D);
    CH_ENUM_VAL(Type::UNKNOWN_SHAPE);
    CH_ENUM_MAPPER_END(Type);
};

ChCollisionShape::ChCollisionShape(Type type) : m_type(type), m_material(nullptr) {}

ChCollisionShape::ChCollisionShape(Type type, std::shared_ptr<ChMaterialSurface> material)
    : m_type(type), m_material(material) {}

void ChCollisionShape::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionShape>();
    // serialize all member data:
    marchive << CHNVP(m_material);

    ChCollisionShape_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    marchive << CHNVP(typemapper(type), "ChCollisionShape__Type");

}

void ChCollisionShape::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionShape>();
    // stream in all member data:
    marchive >> CHNVP(m_material);

    ChCollisionShape_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    marchive >> CHNVP(typemapper(type), "ChCollisionShape__Type");

}

}  // namespace chrono
