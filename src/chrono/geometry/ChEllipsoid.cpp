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

// -----------------------------------------------------------------------------

double ChEllipsoid::GetVolume(const ChVector<>& axes) {
    return (1 / 6.0) * CH_C_PI * axes.x() * axes.y() * axes.z();
}

double ChEllipsoid::GetVolume() const {
    return GetVolume(2.0 * rad);
}

ChMatrix33<> ChEllipsoid::GetGyration(const ChVector<>& axes) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 20.0) * (axes.y() * axes.y() + axes.z() * axes.z());
    J(1, 1) = (1.0 / 20.0) * (axes.z() * axes.z() + axes.x() * axes.x());
    J(2, 2) = (1.0 / 20.0) * (axes.x() * axes.x() + axes.y() * axes.y());

    return J;
}

ChMatrix33<> ChEllipsoid::GetGyration() const {
    return GetGyration(rad);
}

ChAABB ChEllipsoid::GetBoundingBox(const ChVector<>& axes) {
    auto rad = 0.5 * axes;
    return ChAABB(-rad, +rad);
}

ChAABB ChEllipsoid::GetBoundingBox() const {
    return GetBoundingBox(2.0 * rad);
}

double ChEllipsoid::GetBoundingSphereRadius(const ChVector<>& axes) {
    return 0.5 * ChMax(axes.x(), ChMax(axes.y(), axes.z()));
}

double ChEllipsoid::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(2.0 * rad);
}

// -----------------------------------------------------------------------------

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
