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

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChEllipsoid)

ChEllipsoid::ChEllipsoid(const ChVector3d& axes) : rad(0.5 * axes) {}
ChEllipsoid::ChEllipsoid(double axis_x, double axis_y, double axis_z) : rad(0.5 * ChVector3d(axis_x, axis_y, axis_z)) {}
ChEllipsoid::ChEllipsoid(const ChEllipsoid& source) {
    rad = source.rad;
}

// -----------------------------------------------------------------------------

double ChEllipsoid::CalcVolume(const ChVector3d& axes) {
    return (1 / 6.0) * CH_PI * axes.x() * axes.y() * axes.z();
}

double ChEllipsoid::GetVolume() const {
    return CalcVolume(2.0 * rad);
}

ChMatrix33<> ChEllipsoid::CalcGyration(const ChVector3d& axes) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 20.0) * (axes.y() * axes.y() + axes.z() * axes.z());
    J(1, 1) = (1.0 / 20.0) * (axes.z() * axes.z() + axes.x() * axes.x());
    J(2, 2) = (1.0 / 20.0) * (axes.x() * axes.x() + axes.y() * axes.y());

    return J;
}

ChMatrix33<> ChEllipsoid::GetGyration() const {
    return CalcGyration(rad);
}

ChAABB ChEllipsoid::CalcBoundingBox(const ChVector3d& axes) {
    auto rad = 0.5 * axes;
    return ChAABB(-rad, +rad);
}

ChAABB ChEllipsoid::GetBoundingBox() const {
    return CalcBoundingBox(2.0 * rad);
}

double ChEllipsoid::CalcBoundingSphereRadius(const ChVector3d& axes) {
    return 0.5 * std::max(axes.x(), std::max(axes.y(), axes.z()));
}

double ChEllipsoid::GetBoundingSphereRadius() const {
    return CalcBoundingSphereRadius(2.0 * rad);
}

// -----------------------------------------------------------------------------

void ChEllipsoid::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChEllipsoid>();
    // serialize parent class
    ChGeometry::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(rad);
}

void ChEllipsoid::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChEllipsoid>();
    // deserialize parent class
    ChGeometry::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(rad);
}

}  // end namespace chrono
