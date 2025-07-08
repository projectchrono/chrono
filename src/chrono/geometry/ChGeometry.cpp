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

void ChGeometry::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChGeometry>();
    ChGeometry_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    archive_out << CHNVP(typemapper(type), "ChGeometry__Type");
}

void ChGeometry::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChGeometry>();
    ChGeometry_Type_enum_mapper::Type_mapper typemapper;
    Type type = GetType();
    archive_in >> CHNVP(typemapper(type), "ChGeometry__Type");
}

// -----------------------------------------------------------------------------

ChIntAABB::ChIntAABB()
    : min(ChVector3i(+std::numeric_limits<int>::max())), max(ChVector3i(-std::numeric_limits<int>::max())) {}

ChIntAABB::ChIntAABB(const ChVector3i& aabb_min, const ChVector3i& aabb_max) : min(aabb_min), max(aabb_max) {}

ChVector3i ChIntAABB::Size() const {
    return max - min;
}

bool ChIntAABB::IsInverted() const {
    return min > max;
}

ChIntAABB& ChIntAABB::operator+=(const ChIntAABB& aabb) {
    min = Vmin(min, aabb.min);
    max = Vmax(max, aabb.max);
    return *this;
}

ChIntAABB ChIntAABB::operator+(const ChIntAABB& aabb) {
    ChIntAABB result = *this;
    result += aabb;
    return result;
}

ChIntAABB& ChIntAABB::operator+=(const ChVector3i p) {
    min = Vmin(min, p);
    max = Vmax(max, p);
    return *this;
}

// -----------------------------------------------------------------------------

ChAABB::ChAABB()
    : min(ChVector3d(+std::numeric_limits<double>::max())), max(ChVector3d(-std::numeric_limits<double>::max())) {}

ChAABB::ChAABB(const ChVector3d& aabb_min, const ChVector3d& aabb_max) : min(aabb_min), max(aabb_max) {}

ChAABB::ChAABB(const ChIntAABB& aabb, double spacing) : min(spacing * aabb.min), max(spacing * aabb.max) {}

ChVector3d ChAABB::Center() const {
    return 0.5 * (max + min);
}

ChVector3d ChAABB::Size() const {
    return max - min;
}

bool ChAABB::IsInverted() const {
    return min > max;
}

ChAABB& ChAABB::operator+=(const ChAABB& aabb) {
    min = Vmin(min, aabb.min);
    max = Vmax(max, aabb.max);
    return *this;
}

ChAABB ChAABB::operator+(const ChAABB& aabb) {
    ChAABB result = *this;
    result += aabb;
    return result;
}

ChAABB& ChAABB::operator+=(const ChVector3d p) {
    min = Vmin(min, p);
    max = Vmax(max, p);
    return *this;
}

ChAABB ChAABB::Transform(const ChFrame<>& frame) const {
    // Do not perform any operations on this box if it is inverted
    if (IsInverted())
        return ChAABB();

    // Calculate the AABB transformed by the given frame
    // Use algorithm by Jim Arvo (Graphics Gems, 1990)
    ChAABB aabb(frame.GetPos(), frame.GetPos());
    const auto& R = frame.GetRotMat();
    for (int irow = 0; irow < 3; irow++) {
        for (int icol = 0; icol < 3; icol++) {
            double a = R(irow, icol) * min[icol];
            double b = R(irow, icol) * max[icol];
            aabb.min[irow] += a < b ? a : b;
            aabb.max[irow] += a < b ? b : a;
        }
    }
    return aabb;
}

}  // end namespace chrono
