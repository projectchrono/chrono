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
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSphere)

ChSphere::ChSphere(const ChSphere& source) {
    rad = source.rad;
}

// -----------------------------------------------------------------------------

double ChSphere::GetVolume(double radius) {
    return (4.0 / 3.0) * CH_C_PI * radius * radius * radius;
}

double ChSphere::GetVolume() const {
    return GetVolume(rad);
}

ChMatrix33<> ChSphere::GetGyration(double radius) {
    double Jxx = (2.0 / 5.0) * radius * radius;

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = Jxx;
    J(1, 1) = Jxx;
    J(2, 2) = Jxx;

    return J;
}

ChMatrix33<> ChSphere::GetGyration() const {
    return GetGyration(rad);
}

ChAABB ChSphere::GetBoundingBox(double radius) {
    return ChAABB(ChVector<>(-radius), ChVector<>(+radius));
}

ChAABB ChSphere::GetBoundingBox() const {
    return GetBoundingBox(rad);
}

double ChSphere::GetBoundingSphereRadius(double radius) {
    return radius;
}

double ChSphere::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(rad);
}

// -----------------------------------------------------------------------------

void ChSphere::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSphere>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(rad);
}

void ChSphere::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChSphere>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(rad);
}

}  // end namespace geometry
}  // end namespace chrono
