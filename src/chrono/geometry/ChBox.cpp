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
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChBox)

ChBox::ChBox(const ChVector<>& lengths) : hlen(0.5 * lengths) {}
ChBox::ChBox(double length_x, double length_y, double length_z)
    : hlen(0.5 * ChVector<>(length_z, length_y, length_z)) {}
ChBox::ChBox(const ChBox& source) {
    hlen = source.hlen;
}

ChVector<> ChBox::Evaluate(double parU, double parV, double parW) const {
    return ChVector<>(hlen.x() * (parU - 0.5), hlen.y() * (parV - 0.5), hlen.z() * (parW - 0.5));
}

// -----------------------------------------------------------------------------

double ChBox::GetVolume(const ChVector<>& lengths) {
    return lengths.x() * lengths.y() * lengths.z();
}

double ChBox::GetVolume() const {
    return GetVolume(2.0 * hlen);
}

ChMatrix33<> ChBox::GetGyration(const ChVector<>& lengths) {
    ChMatrix33<> J;
    J.setZero();
    J(0, 0) = (1.0 / 12.0) * (lengths.y() * lengths.y() + lengths.z() * lengths.z());
    J(1, 1) = (1.0 / 12.0) * (lengths.z() * lengths.z() + lengths.x() * lengths.x());
    J(2, 2) = (1.0 / 12.0) * (lengths.x() * lengths.x() + lengths.y() * lengths.y());

    return J;
}

ChMatrix33<> ChBox::GetGyration() const {
    return GetGyration(hlen);
}

ChAABB ChBox::GetBoundingBox(const ChVector<>& lengths) {
    auto hlen = lengths / 2;

    std::vector<ChVector<>> vertices{
        ChVector<>(+hlen.x(), +hlen.y(), +hlen.z()),  //
        ChVector<>(-hlen.x(), +hlen.y(), +hlen.z()),  //
        ChVector<>(-hlen.x(), -hlen.y(), +hlen.z()),  //
        ChVector<>(+hlen.x(), -hlen.y(), +hlen.z()),  //
        ChVector<>(+hlen.x(), +hlen.y(), -hlen.z()),  //
        ChVector<>(-hlen.x(), +hlen.y(), -hlen.z()),  //
        ChVector<>(-hlen.x(), -hlen.y(), -hlen.z()),  //
        ChVector<>(+hlen.x(), -hlen.y(), -hlen.z())   //
    };

    ChAABB bbox;
    for (const auto& v : vertices) {
        bbox.min.x() = ChMin(bbox.min.x(), v.x());
        bbox.min.y() = ChMin(bbox.min.y(), v.y());
        bbox.min.z() = ChMin(bbox.min.z(), v.z());

        bbox.max.x() = ChMax(bbox.max.x(), v.x());
        bbox.max.y() = ChMax(bbox.max.y(), v.y());
        bbox.max.z() = ChMax(bbox.max.z(), v.z());
    }

    return bbox;
}

ChAABB ChBox::GetBoundingBox() const {
    return GetBoundingBox(2.0 * hlen);
}

double ChBox::GetBoundingSphereRadius(const ChVector<>& lengths) {
    return lengths.Length() / 2;
}

double ChBox::GetBoundingSphereRadius() const {
    return GetBoundingSphereRadius(2.0 * hlen);
}

// -----------------------------------------------------------------------------

void ChBox::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChBox>();
    // serialize parent class
    ChVolume::ArchiveOut(marchive);
    // serialize all member data:
    ChVector<> lengths = GetLengths();
    marchive << CHNVP(lengths);
}

void ChBox::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChBox>();
    // deserialize parent class
    ChVolume::ArchiveIn(marchive);
    // stream in all member data:
    ChVector<> lengths;
    marchive >> CHNVP(lengths);
    SetLengths(lengths);
}

}  // end namespace geometry
}  // end namespace chrono
