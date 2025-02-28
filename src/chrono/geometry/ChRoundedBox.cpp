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

#include "chrono/geometry/ChRoundedBox.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedBox)

ChRoundedBox::ChRoundedBox(const ChVector3d& lengths, double sphere_radius)
    : hlen(0.5 * lengths), srad(sphere_radius) {}

ChRoundedBox::ChRoundedBox(double length_x, double length_y, double length_z, double sphere_radius)
    : hlen(0.5 * ChVector3d(length_z, length_y, length_z)), srad(sphere_radius) {}

ChRoundedBox::ChRoundedBox(const ChRoundedBox& source) {
    hlen = source.hlen;
}

ChVector3d ChRoundedBox::Evaluate(double parU, double parV, double parW) const {
    return ChVector3d(hlen.x() * (parU - 0.5), hlen.y() * (parV - 0.5), hlen.z() * (parW - 0.5));
}

// -----------------------------------------------------------------------------

double ChRoundedBox::GetVolume(const ChVector3d& lengths, double srad) {
    return lengths.x() * lengths.y() * lengths.z() +
           0.5 * srad * (lengths.x() * lengths.y() + lengths.y() * lengths.z() + lengths.z() * lengths.x()) +
           (4.0 * CH_PI_3) * srad * srad * srad;
}

double ChRoundedBox::GetVolume() const {
    return GetVolume(2.0 * hlen, srad);
}

ChMatrix33<> ChRoundedBox::GetGyration(const ChVector3d& lengths, double srad) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 12.0) * (lengths.y() * lengths.y() + lengths.z() * lengths.z());
    J(1, 1) = (1.0 / 12.0) * (lengths.z() * lengths.z() + lengths.x() * lengths.x());
    J(2, 2) = (1.0 / 12.0) * (lengths.x() * lengths.x() + lengths.y() * lengths.y());

    return J;
}

ChMatrix33<> ChRoundedBox::GetGyration() const {
    return GetGyration(hlen, srad);
}

ChAABB ChRoundedBox::GetBoundingBox(const ChVector3d& lengths, double srad) {
    auto hlen = lengths / 2;

    std::vector<ChVector3d> vertices{
        ChVector3d(+hlen.x(), +hlen.y(), +hlen.z()),  //
        ChVector3d(-hlen.x(), +hlen.y(), +hlen.z()),  //
        ChVector3d(-hlen.x(), -hlen.y(), +hlen.z()),  //
        ChVector3d(+hlen.x(), -hlen.y(), +hlen.z()),  //
        ChVector3d(+hlen.x(), +hlen.y(), -hlen.z()),  //
        ChVector3d(-hlen.x(), +hlen.y(), -hlen.z()),  //
        ChVector3d(-hlen.x(), -hlen.y(), -hlen.z()),  //
        ChVector3d(+hlen.x(), -hlen.y(), -hlen.z())   //
    };

    ChAABB bbox;
    for (const auto& v : vertices) {
        bbox.min.x() = std::min(bbox.min.x(), v.x());
        bbox.min.y() = std::min(bbox.min.y(), v.y());
        bbox.min.z() = std::min(bbox.min.z(), v.z());

        bbox.max.x() = std::max(bbox.max.x(), v.x());
        bbox.max.y() = std::max(bbox.max.y(), v.y());
        bbox.max.z() = std::max(bbox.max.z(), v.z());
    }

    return bbox;
}

ChAABB ChRoundedBox::GetBoundingBox() const {
    return GetBoundingBox(2.0 * hlen, srad);
}

double ChRoundedBox::GetBoundingSphereRadius(const ChVector3d& lengths, double srad) {
    return lengths.Length() / 2 + srad;
}

double ChRoundedBox::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(2.0 * hlen, srad);
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
