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

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCone)

ChCone::ChCone(const ChCone& source) {
    r = source.r;
    h = source.h;
}

// -----------------------------------------------------------------------------

double ChCone::CalcVolume(double radius, double height) {
    return CH_PI_3 * radius * radius * height;
}

double ChCone::GetVolume() const {
    return CalcVolume(r, h);
}

ChMatrix33<> ChCone::CalcGyration(double radius, double height) {
    double Ixx = (3.0 / 20.0) * (radius * radius) + (1.0 / 10.0) * (height * height);

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = Ixx;
    J(1, 1) = Ixx;
    J(2, 2) = (3.0 / 10.0) * (radius * radius);

    return J;
}

ChMatrix33<> ChCone::GetGyration() const {
    return CalcGyration(r, h);
}

ChAABB ChCone::CalcBoundingBox(double radius, double height) {
    return ChAABB(ChVector3d(-radius, -radius, -height / 2),  //
                  ChVector3d(+radius, +radius, +height / 2));
}

ChAABB ChCone::GetBoundingBox() const {
    return CalcBoundingBox(r, h);
}

double ChCone::CalcBoundingSphereRadius(double radius, double height) {
    return std::sqrt(height * height / 4 + radius * radius);
}

double ChCone::GetBoundingSphereRadius() const {
    return CalcBoundingSphereRadius(r, h);
}

// -----------------------------------------------------------------------------

void ChCone::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChCone>();
    // serialize parent class
    ChGeometry::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(r);
    archive_out << CHNVP(h);
}

void ChCone::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChCone>();
    // deserialize parent class
    ChGeometry::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(r);
    archive_in >> CHNVP(h);
}

}  // end namespace chrono
