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
#include <cmath>

#include "chrono/geometry/ChCylinder.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCylinder)

ChCylinder::ChCylinder(const ChCylinder& source) {
    r = source.r;
    h = source.h;
}

// -----------------------------------------------------------------------------

double ChCylinder::GetVolume(double radius, double height) {
    return CH_C_PI * radius * radius * height;
}

double ChCylinder::GetVolume() const {
    return GetVolume(r, h);
}

ChMatrix33<> ChCylinder::GetGyration(double radius, double height) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 12.0) * (3 * radius * radius + height * height);
    J(1, 1) = (1.0 / 2.0) * (radius * radius);
    J(2, 2) = (1.0 / 12.0) * (3 * radius * radius + height * height);

    return J;
}

ChMatrix33<> ChCylinder::GetGyration() const {
    return GetGyration(r, h);
}

ChAABB ChCylinder::GetBoundingBox(double radius, double height) {
    return ChAABB(ChVector<>(-radius, -radius, -height / 2),  //
                  ChVector<>(+radius, +radius, +height / 2));
}

ChAABB ChCylinder::GetBoundingBox() const {
    return GetBoundingBox(r, h);
}

double ChCylinder::GetBoundingSphereRadius(double radius, double height) {
    return std::sqrt(height * height / 4 + radius * radius);
}

double ChCylinder::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(r, h);
}

// -----------------------------------------------------------------------------

void ChCylinder::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCylinder>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(r);
    marchive << CHNVP(h);
}

void ChCylinder::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCylinder>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(r);
    marchive >> CHNVP(h);
}

}  // end namespace geometry
}  // end namespace chrono
