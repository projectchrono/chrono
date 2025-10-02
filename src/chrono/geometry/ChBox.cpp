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

#include "chrono/geometry/ChBox.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChBox)

ChBox::ChBox(const ChVector3d& lengths) : hlen(0.5 * lengths) {}
ChBox::ChBox(double length_x, double length_y, double length_z)
    : hlen(0.5 * ChVector3d(length_z, length_y, length_z)) {}
ChBox::ChBox(const ChBox& source) {
    hlen = source.hlen;
}

ChVector3d ChBox::Evaluate(double parU, double parV, double parW) const {
    return ChVector3d(hlen.x() * (parU - 0.5), hlen.y() * (parV - 0.5), hlen.z() * (parW - 0.5));
}

// -----------------------------------------------------------------------------

double ChBox::CalcVolume(const ChVector3d& lengths) {
    return lengths.x() * lengths.y() * lengths.z();
}

double ChBox::GetVolume() const {
    return CalcVolume(2.0 * hlen);
}

ChMatrix33<> ChBox::CalcGyration(const ChVector3d& lengths) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 12.0) * (lengths.y() * lengths.y() + lengths.z() * lengths.z());
    J(1, 1) = (1.0 / 12.0) * (lengths.z() * lengths.z() + lengths.x() * lengths.x());
    J(2, 2) = (1.0 / 12.0) * (lengths.x() * lengths.x() + lengths.y() * lengths.y());

    return J;
}

ChMatrix33<> ChBox::GetGyration() const {
    return CalcGyration(hlen);
}

ChAABB ChBox::CalcBoundingBox(const ChVector3d& lengths) {
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

ChAABB ChBox::GetBoundingBox() const {
    return CalcBoundingBox(2.0 * hlen);
}

double ChBox::CalcBoundingSphereRadius(const ChVector3d& lengths) {
    return lengths.Length() / 2;
}

double ChBox::GetBoundingSphereRadius() const {
    return CalcBoundingSphereRadius(2.0 * hlen);
}

// -----------------------------------------------------------------------------

void ChBox::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChBox>();
    // serialize parent class
    ChVolume::ArchiveOut(archive_out);
    // serialize all member data:
    ChVector3d lengths = GetLengths();
    archive_out << CHNVP(lengths);
}

void ChBox::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChBox>();
    // deserialize parent class
    ChVolume::ArchiveIn(archive_in);
    // stream in all member data:
    ChVector3d lengths;
    archive_in >> CHNVP(lengths);
    SetLengths(lengths);
}

}  // end namespace chrono
