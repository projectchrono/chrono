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

#include "chrono/geometry/ChCapsule.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCapsule)

ChCapsule::ChCapsule(const ChCapsule& source) {
    r = source.r;
    h = source.h;
}

// -----------------------------------------------------------------------------
double ChCapsule::GetVolume(double radius, double height) {
    double tmp = radius * radius * height + (4 / 3.0) * radius * radius * radius;
    return CH_C_PI * tmp;
}

double ChCapsule::GetVolume() const {
    return GetVolume(r, h);
}

ChMatrix33<> ChCapsule::GetGyration(double radius, double height) {
    double massRatio = (3 / 4.0) * height / radius;
    double cmDist = (1 / 2.0) * height + (3 / 8.0) * radius;
    double Ixx = massRatio / (1 + massRatio) * (1.0 / 12.0) * (3 * radius * radius + height * height) +
                 1 / (1 + massRatio) * (0.259 * radius * radius + cmDist * cmDist);
    double Iyy = massRatio / (1 + massRatio) * (1.0 / 2.0) * (radius * radius) +
                 1 / (1 + massRatio) * (2.0 / 5.0) * (radius * radius);

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = Ixx;
    J(1, 1) = Iyy;
    J(2, 2) = Ixx;

    return J;
}

ChMatrix33<> ChCapsule::GetGyration() const {
    return GetGyration(r, h);
}

ChAABB ChCapsule::GetBoundingBox(double radius, double height) {
    return ChAABB(ChVector<>(-radius, -radius, -(radius + height / 2)),
                  ChVector<>(+radius, +radius, +(radius + height / 2)));
}

ChAABB ChCapsule::GetBoundingBox() const {
    return GetBoundingBox(r, h);
}

double ChCapsule::GetBoundingSphereRadius(double radius, double height) {
    return radius + height / 2;
}

double ChCapsule::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(r, h);
}

// -----------------------------------------------------------------------------

void ChCapsule::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCapsule>();
    // serialize parent class
    ChGeometry::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(r);
    marchive << CHNVP(h);
}

void ChCapsule::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCapsule>();
    // deserialize parent class
    ChGeometry::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(r);
    marchive >> CHNVP(h);
}

}  // end namespace geometry
}  // end namespace chrono
