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

#include "chrono/geometry/ChEllipsoid.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChEllipsoid)

ChEllipsoid::ChEllipsoid(const ChVector<>& axes) : rad(0.5 * axes) {}
ChEllipsoid::ChEllipsoid(double axis_x, double axis_y, double axis_z) : rad(0.5 * ChVector<>(axis_x, axis_y, axis_z)) {}
ChEllipsoid::ChEllipsoid(const ChEllipsoid& source) {
    rad = source.rad;
}

ChGeometry::AABB ChEllipsoid::GetBoundingBox(const ChMatrix33<>& rot) const {
    return AABB(-rad, +rad);
}

double ChEllipsoid::GetBoundingSphereRadius() const {
    return ChMax(rad.x(), ChMax(rad.y(), rad.z()));
}

void ChEllipsoid::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChEllipsoid>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(rad);
}

void ChEllipsoid::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChEllipsoid>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(rad);
}

}  // end namespace geometry
}  // end namespace chrono
