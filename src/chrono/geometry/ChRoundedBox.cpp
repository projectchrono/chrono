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

#include "chrono/core/ChTransform.h"
#include "chrono/geometry/ChRoundedBox.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChRoundedBox)

ChRoundedBox::ChRoundedBox(const ChVector<>& lengths, double sphere_radius)
    : hlen(0.5 * lengths), srad(sphere_radius) {}

ChRoundedBox::ChRoundedBox(double length_x, double length_y, double length_z, double sphere_radius)
    : hlen(0.5 * ChVector<>(length_z, length_y, length_z)), srad(sphere_radius) {}

ChRoundedBox::ChRoundedBox(const ChRoundedBox& source) {
    hlen = source.hlen;
}

void ChRoundedBox::Evaluate(ChVector<>& pos, const double parU, const double parV, const double parW) const {
    pos.x() = hlen.x() * (parU - 0.5);
    pos.y() = hlen.y() * (parV - 0.5);
    pos.z() = hlen.z() * (parW - 0.5);
}

ChGeometry::AABB ChRoundedBox::GetBoundingBox(const ChMatrix33<>& rot) const {
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

    AABB bbox;
    for (const auto& v : vertices) {
        auto p = rot.transpose() * v;

        bbox.min.x() = ChMin(bbox.min.x(), p.x());
        bbox.min.y() = ChMin(bbox.min.y(), p.y());
        bbox.min.z() = ChMin(bbox.min.z(), p.z());

        bbox.max.x() = ChMax(bbox.max.x(), p.x());
        bbox.max.y() = ChMax(bbox.max.y(), p.y());
        bbox.max.z() = ChMax(bbox.max.z(), p.z());
    }

    return bbox;
}

void ChRoundedBox::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChRoundedBox>();
    // serialize parent class
    ChVolume::ArchiveOUT(marchive);
    // serialize all member data:
    ChVector<> lengths = GetLengths();
    marchive << CHNVP(lengths);
    marchive << CHNVP(srad);
}

void ChRoundedBox::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChRoundedBox>();
    // deserialize parent class
    ChVolume::ArchiveIN(marchive);
    // stream in all member data:
    ChVector<> lengths;
    marchive >> CHNVP(lengths);
    SetLengths(lengths);
    marchive >> CHNVP(srad);
}

}  // end namespace geometry
}  // end namespace chrono
