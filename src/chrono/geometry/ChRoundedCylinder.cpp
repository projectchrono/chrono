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

#include "chrono/geometry/ChRoundedCylinder.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedCylinder)

ChRoundedCylinder::ChRoundedCylinder(double radius, double height, double sphere_radius)
    : r(radius), h(height), sr(sphere_radius) {}

ChRoundedCylinder::ChRoundedCylinder(const ChRoundedCylinder& source) {
    r = source.r;
    h = source.h;
    sr = source.sr;
}

// -----------------------------------------------------------------------------

double ChRoundedCylinder::GetVolume(double radius, double height, double srad) {
    double tmp = (radius + srad) * (radius + srad) * height / 2 + srad * (radius * radius + CH_2_3 * srad * srad) +
                 (CH_PI_2 - 1.0) * radius * srad * srad;
    return 2.0 * CH_PI * tmp;
}

double ChRoundedCylinder::GetVolume() const {
    return GetVolume(r, h, sr);
}

ChMatrix33<> ChRoundedCylinder::GetGyration(double radius, double height, double srad) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 12.0) * (3 * radius * radius + height * height);
    J(1, 1) = (1.0 / 2.0) * (radius * radius);
    J(2, 2) = (1.0 / 12.0) * (3 * radius * radius + height * height);

    return J;
}

ChMatrix33<> ChRoundedCylinder::GetGyration() const {
    return GetGyration(r, h, sr);
}

ChAABB ChRoundedCylinder::GetBoundingBox(double radius, double height, double srad) {
    return ChAABB(ChVector3d(-radius, -radius, -height / 2) - srad,  //
                  ChVector3d(+radius, +radius, +height / 2) + srad);
}

ChAABB ChRoundedCylinder::GetBoundingBox() const {
    return GetBoundingBox(r, h, sr);
}

double ChRoundedCylinder::GetBoundingSphereRadius(double radius, double height, double srad) {
    return std::sqrt(height * height / 4 + radius * radius) + srad;
}

double ChRoundedCylinder::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(r, h, sr);
}

// -----------------------------------------------------------------------------

void ChRoundedCylinder::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChRoundedCylinder>();
    // serialize parent class
    ChGeometry::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(r);
    archive_out << CHNVP(h);
    archive_out << CHNVP(sr);
}

void ChRoundedCylinder::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChRoundedCylinder>();
    // deserialize parent class
    ChGeometry::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(r);
    archive_in >> CHNVP(h);
    archive_in >> CHNVP(sr);
}

}  // end namespace chrono
