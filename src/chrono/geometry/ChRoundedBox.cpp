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

#include "chrono/geometry/ChRoundedBox.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedBox)

ChRoundedBox::ChRoundedBox(const ChVector3d& lengths, double sphere_radius) : hlen(0.5 * lengths), srad(sphere_radius) {}

ChRoundedBox::ChRoundedBox(double length_x, double length_y, double length_z, double sphere_radius) : hlen(0.5 * ChVector3d(length_x, length_y, length_z)), srad(sphere_radius) {}

ChRoundedBox::ChRoundedBox(const ChRoundedBox& source) {
    hlen = source.hlen;
    srad = source.srad;
}

ChVector3d ChRoundedBox::Evaluate(double parU, double parV, double parW) const {
    return ChVector3d(2 * hlen.x() * (parU - 0.5), 2 * hlen.y() * (parV - 0.5), 2 * hlen.z() * (parW - 0.5));
}

// -----------------------------------------------------------------------------

// The rounded box is the Minkowski sum of an inner box (lengths L-2*srad) with a sphere of radius srad, such that L are the
// outer dimensions. The sweeping sphere radius is clamped to half the smallest box length.
// The solid is split in the inner box, 6 face slabs, 12 edge quarter-cylinders, and 8 corner sphere octants.

// Return int(x_i^2 dV) over the rounded box, given the inner half-lengths along axis i and the two other axes j and k.
static double CalcRoundedBoxSecondMoment(double ei, double ej, double ek, double s) {
    // Moments int(u^i w^j) over a quarter disk of radius s (u, w >= 0)
    double m00 = CH_PI_4 * s * s;
    double m10 = s * s * s / 3;
    double m20 = CH_PI * s * s * s * s / 16;

    double res = 8 * ei * ej * ek * ei * ei / 3;                                                                     // inner box
    res += 2 * (4 * s * ej * ek) * ((ei + s / 2) * (ei + s / 2) + s * s / 12);                                       // slabs normal to i
    res += 2 * (4 * s * ei * ek + 4 * s * ei * ej) * ei * ei / 3;                                                    // slabs normal to j and k
    res += 4 * (2 * ei * m00) * ei * ei / 3;                                                                         // edges parallel to i
    res += 4 * (2 * ej + 2 * ek) * (ei * ei * m00 + 2 * ei * m10 + m20);                                             // edges parallel to j and k
    res += 8 * (CH_PI * s * s * s / 6 * ei * ei + ei * CH_PI * s * s * s * s / 8 + CH_PI * s * s * s * s * s / 30);  // corners
    return res;
}

double ChRoundedBox::CalcVolume(const ChVector3d& lengths, double srad) {
    double s = std::min({srad, lengths.x() / 2, lengths.y() / 2, lengths.z() / 2});
    double a = lengths.x() - 2 * s;
    double b = lengths.y() - 2 * s;
    double c = lengths.z() - 2 * s;
    return a * b * c + 2 * s * (a * b + b * c + c * a) + CH_PI * s * s * (a + b + c) + (4.0 * CH_PI_3) * s * s * s;
}

double ChRoundedBox::GetVolume() const {
    return CalcVolume(2.0 * hlen, srad);
}

ChMatrix33<> ChRoundedBox::CalcGyration(const ChVector3d& lengths, double srad) {
    double s = std::min({srad, lengths.x() / 2, lengths.y() / 2, lengths.z() / 2});
    double ex = lengths.x() / 2 - s;
    double ey = lengths.y() / 2 - s;
    double ez = lengths.z() / 2 - s;

    double V = CalcVolume(lengths, srad);
    double Sx = CalcRoundedBoxSecondMoment(ex, ey, ez, s);
    double Sy = CalcRoundedBoxSecondMoment(ey, ez, ex, s);
    double Sz = CalcRoundedBoxSecondMoment(ez, ex, ey, s);

    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (Sy + Sz) / V;
    J(1, 1) = (Sz + Sx) / V;
    J(2, 2) = (Sx + Sy) / V;

    return J;
}

ChMatrix33<> ChRoundedBox::GetGyration() const {
    return CalcGyration(2.0 * hlen, srad);
}

ChAABB ChRoundedBox::CalcBoundingBox(const ChVector3d& lengths, double srad) {
    return ChAABB(-lengths / 2, +lengths / 2);
}

ChAABB ChRoundedBox::GetBoundingBox() const {
    return CalcBoundingBox(2.0 * hlen, srad);
}

double ChRoundedBox::CalcBoundingSphereRadius(const ChVector3d& lengths, double srad) {
    double s = std::min({srad, lengths.x() / 2, lengths.y() / 2, lengths.z() / 2});
    return (lengths / 2 - ChVector3d(s)).Length() + s;
}

double ChRoundedBox::GetBoundingSphereRadius() const {
    return CalcBoundingSphereRadius(2.0 * hlen, srad);
}

// -----------------------------------------------------------------------------

void ChRoundedBox::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChRoundedBox>();
    // serialize parent class
    ChVolume::ArchiveOut(archive_out);
    // serialize all member data:
    ChVector3d lengths = GetLengths();
    archive_out << CHNVP(lengths);
    archive_out << CHNVP(srad);
}

void ChRoundedBox::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChRoundedBox>();
    // deserialize parent class
    ChVolume::ArchiveIn(archive_in);
    // stream in all member data:
    ChVector3d lengths;
    archive_in >> CHNVP(lengths);
    SetLengths(lengths);
    archive_in >> CHNVP(srad);
}

}  // end namespace chrono
