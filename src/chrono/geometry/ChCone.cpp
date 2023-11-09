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

#include "chrono/geometry/ChCone.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCone)

ChCone::ChCone(const ChCone& source) {
    r = source.r;
    h = source.h;
}

// -----------------------------------------------------------------------------

double ChCone::GetVolume(double radius, double height) {
    return CH_C_PI * radius * radius * height / 3.0;
}

double ChCone::GetVolume() const {
    return GetVolume(r, h);
}

ChMatrix33<> ChCone::GetGyration(double radius, double height) {
    double Ixx = (3.0 / 80.0) * (height * height) + (3.0 / 20.0) * (radius * radius);

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = Ixx;
    J(1, 1) = (3.0 / 10.0) * (radius * radius);
    J(2, 2) = Ixx;

    return J;
}

ChMatrix33<> ChCone::GetGyration() const {
    return GetGyration(r, h);
}

ChAABB ChCone::GetBoundingBox(double radius, double height) {
    return ChAABB(ChVector<>(-radius, -radius, 0), ChVector<>(+radius, +radius, height));
}

ChAABB ChCone::GetBoundingBox() const {
    return GetBoundingBox(r, h);
}

double ChCone::GetBoundingSphereRadius(double radius, double height) {
    return std::sqrt(height * height / 4 + radius * radius);
}

double ChCone::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(r, h);
}

// -----------------------------------------------------------------------------

void ChCone::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCone>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(r);
    marchive << CHNVP(h);
}

void ChCone::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCone>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(r);
    marchive >> CHNVP(h);
}

}  // end namespace geometry
}  // end namespace chrono
