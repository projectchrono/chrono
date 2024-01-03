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

#include <cstdio>

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChGeometry)  // NO! Abstract class!

class ChGeometry_Type_enum_mapper : public ChGeometry {
  public:
    CH_ENUM_MAPPER_BEGIN(Type);
    CH_ENUM_VAL(Type::NONE);
    CH_ENUM_VAL(Type::SPHERE);
    CH_ENUM_VAL(Type::ELLIPSOID);
    CH_ENUM_VAL(Type::BOX);
    CH_ENUM_VAL(Type::CYLINDER);
    CH_ENUM_VAL(Type::TRIANGLE);
    CH_ENUM_VAL(Type::CAPSULE);
    CH_ENUM_VAL(Type::CONE);
    CH_ENUM_VAL(Type::LINE);
    CH_ENUM_VAL(Type::LINE_ARC);
    CH_ENUM_VAL(Type::LINE_BEZIER);
    CH_ENUM_VAL(Type::LINE_CAM);
    CH_ENUM_VAL(Type::LINE_PATH);
    CH_ENUM_VAL(Type::LINE_POLY);
    CH_ENUM_VAL(Type::LINE_SEGMENT);
    CH_ENUM_VAL(Type::ROUNDED_BOX);
    CH_ENUM_VAL(Type::ROUNDED_CYLINDER);
    CH_ENUM_VAL(Type::TRIANGLEMESH);
    CH_ENUM_VAL(Type::TRIANGLEMESH_CONNECTED);
    CH_ENUM_VAL(Type::TRIANGLEMESH_SOUP);
    CH_ENUM_MAPPER_END(Type);
};

ChAABB ChGeometry::GetBoundingBox() const {
    return ChAABB();
}

void ChGeometry::InflateBoundingBox(ChAABB& bbox) const {
    auto this_bbox = GetBoundingBox();
    bbox.min = Vmin(bbox.min, this_bbox.min);
    bbox.max = Vmin(bbox.max, this_bbox.max);
}

double ChGeometry::GetBoundingSphereRadius() const {
    auto bbox = GetBoundingBox();
    return bbox.Size().Length() / 2;
}

void ChGeometry::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChGeometry>();
    ChGeometry_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetClassType();
    marchive << CHNVP(typemapper(type), "ChGeometry__Type");
}

void ChGeometry::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChGeometry>();
    ChGeometry_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetClassType();
    marchive >> CHNVP(typemapper(type), "ChGeometry__Type");
}

// -----------------------------------------------------------------------------

ChAABB::ChAABB()
    : min(ChVector<>(+std::numeric_limits<double>::max())), max(ChVector<>(-std::numeric_limits<double>::max())) {}

ChAABB::ChAABB(const ChVector<>& aabb_min, const ChVector<>& aabb_max) : min(aabb_min), max(aabb_max) {}

ChVector<> ChAABB::Center() const {
    return 0.5 * (max + min);
}

ChVector<> ChAABB::Size() const {
    return max - min;
}

bool ChAABB::IsInverted() const {
    return min > max;
}

}  // end namespace geometry
}  // end namespace chrono
