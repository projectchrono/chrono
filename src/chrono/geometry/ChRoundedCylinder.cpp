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

#include <algorithm>
#include <cstdio>

#include "chrono/geometry/ChRoundedCylinder.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedCylinder)

ChRoundedCylinder::ChRoundedCylinder(double radius, double height, double sphere_radius) : r(radius), h(height), sr(sphere_radius) {}

ChRoundedCylinder::ChRoundedCylinder(const ChRoundedCylinder& source) {
    r = source.r;
    h = source.h;
    sr = source.sr;
}

// -----------------------------------------------------------------------------

// The rounded cylinder is the Minkowski sum of an inner cylinder (radius r-sr, height h-2*sr) with a sphere of radius sr,
// such that r and h are the outer dimensions. The sweeping sphere radius is clamped to min(sr, r, h/2).
// The solid is split in a cylinder of radius r and height h-2*sr, two disks of radius r-sr and thickness sr (end caps),
// and two rims, each obtained by revolving a quarter disk of radius sr about the cylinder axis.

static void CalcRoundedCylinderMassProperties(double radius, double height, double srad, double& V, double& Ixx, double& Izz) {
    double s = std::min({srad, radius, height / 2});
    double a = radius - s;      // inner cylinder radius
    double c = height / 2 - s;  // inner cylinder half-height

    // Moments int(u^i w^j) over a quarter disk of radius s (u, w >= 0)
    double m00 = CH_PI_4 * s * s;
    double m10 = s * s * s / 3;
    double m20 = CH_PI * s * s * s * s / 16;
    double m11 = s * s * s * s / 8;
    double m30 = 2 * s * s * s * s * s / 15;
    double m12 = s * s * s * s * s / 15;

    // central cylinder
    double V1 = CH_PI * radius * radius * 2 * c;
    double Izz1 = V1 * radius * radius / 2;
    double Ixx1 = V1 * (radius * radius / 4 + c * c / 3);

    // end cap disks (one)
    double V2 = CH_PI * a * a * s;
    double Izz2 = V2 * a * a / 2;
    double Ixx2 = V2 * (a * a / 4 + s * s / 12 + (c + s / 2) * (c + s / 2));

    // rims (one)
    double V3 = CH_2PI * (a * m00 + m10);
    double Izz3 = CH_2PI * (a * a * a * m00 + 3 * a * a * m10 + 3 * a * m20 + m30);
    double Ixx3 = Izz3 / 2 + CH_2PI * (a * (c * c * m00 + 2 * c * m10 + m20) + c * c * m10 + 2 * c * m11 + m12);

    V = V1 + 2 * V2 + 2 * V3;
    Ixx = Ixx1 + 2 * Ixx2 + 2 * Ixx3;
    Izz = Izz1 + 2 * Izz2 + 2 * Izz3;
}

double ChRoundedCylinder::CalcVolume(double radius, double height, double srad) {
    double V, Ixx, Izz;
    CalcRoundedCylinderMassProperties(radius, height, srad, V, Ixx, Izz);
    return V;
}

double ChRoundedCylinder::GetVolume() const {
    return CalcVolume(r, h, sr);
}

ChMatrix33<> ChRoundedCylinder::CalcGyration(double radius, double height, double srad) {
    double V, Ixx, Izz;
    CalcRoundedCylinderMassProperties(radius, height, srad, V, Ixx, Izz);

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = Ixx / V;
    J(1, 1) = Ixx / V;
    J(2, 2) = Izz / V;

    return J;
}

ChMatrix33<> ChRoundedCylinder::GetGyration() const {
    return CalcGyration(r, h, sr);
}

ChAABB ChRoundedCylinder::CalcBoundingBox(double radius, double height, double srad) {
    return ChAABB(ChVector3d(-radius, -radius, -height / 2),  //
                  ChVector3d(+radius, +radius, +height / 2));
}

ChAABB ChRoundedCylinder::GetBoundingBox() const {
    return CalcBoundingBox(r, h, sr);
}

double ChRoundedCylinder::CalcBoundingSphereRadius(double radius, double height, double srad) {
    double s = std::min({srad, radius, height / 2});
    double a = radius - s;
    double c = height / 2 - s;
    return std::sqrt(a * a + c * c) + s;
}

double ChRoundedCylinder::GetBoundingSphereRadius() const {
    return CalcBoundingSphereRadius(r, h, sr);
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
