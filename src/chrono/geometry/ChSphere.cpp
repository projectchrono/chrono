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

#include "chrono/geometry/ChSphere.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSphere)

ChSphere::ChSphere(const ChSphere& source) {
    rad = source.rad;
}

// -----------------------------------------------------------------------------

double ChSphere::CalcVolume(double radius) {
    return CH_4_3 * CH_PI * radius * radius * radius;
}

double ChSphere::GetVolume() const {
    return CalcVolume(rad);
}

ChMatrix33<> ChSphere::CalcGyration(double radius) {
    double Jxx = (2.0 / 5.0) * radius * radius;

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = Jxx;
    J(1, 1) = Jxx;
    J(2, 2) = Jxx;

    return J;
}

ChMatrix33<> ChSphere::GetGyration() const {
    return CalcGyration(rad);
}

ChAABB ChSphere::CalcBoundingBox(double radius) {
    return ChAABB(ChVector3d(-radius), ChVector3d(+radius));
}

ChAABB ChSphere::GetBoundingBox() const {
    return CalcBoundingBox(rad);
}

double ChSphere::CalcBoundingSphereRadius(double radius) {
    return radius;
}

double ChSphere::GetBoundingSphereRadius() const {
    return CalcBoundingSphereRadius(rad);
}

// -----------------------------------------------------------------------------

void ChSphere::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSphere>();
    // serialize parent class
    ChGeometry::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(rad);
}

void ChSphere::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSphere>();
    // deserialize parent class
    ChGeometry::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(rad);
}

}  // end namespace chrono
