// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include <cstdio>

#include "chrono/geometry/ChAABB.h"

namespace chrono {

ChIntAABB::ChIntAABB() : min(ChVector3i(+std::numeric_limits<int>::max())), max(ChVector3i(-std::numeric_limits<int>::max())) {}

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

void ChIntAABB::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChIntAABB>();
    archive_out << CHNVP(min);
    archive_out << CHNVP(max);
}

void ChIntAABB::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChIntAABB>();
    archive_in >> CHNVP(min);
    archive_in >> CHNVP(max);
}

// -----------------------------------------------------------------------------

ChAABB::ChAABB() : min(ChVector3d(+std::numeric_limits<double>::max())), max(ChVector3d(-std::numeric_limits<double>::max())) {}

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

ChAABB ChAABB::Inflate(const ChVector3d& v) const {
    ChAABB aabb(min, max);
    aabb.min -= v;
    aabb.max += v;
    return aabb;
}

ChAABB ChAABB::Transform(const ChFramed& frame) const {
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

void ChAABB::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChAABB>();
    archive_out << CHNVP(min);
    archive_out << CHNVP(max);
}

void ChAABB::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChAABB>();
    archive_in >> CHNVP(min);
    archive_in >> CHNVP(max);
}

}  // end namespace chrono
