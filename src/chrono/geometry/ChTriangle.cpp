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

#include "chrono/geometry/ChTriangle.h"
#include "chrono/collision/ChCollisionUtils.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTriangle)

// Tolerance for testing degenerate triangles
#define EPS_TRIDEGENERATE 1e-20

ChTriangle::ChTriangle(const ChTriangle& source) {
    p1 = source.p1;
    p2 = source.p2;
    p3 = source.p3;
}

ChTriangle& ChTriangle::operator=(const ChTriangle& source) {
    if (&source == this)
        return *this;
    p1 = source.p1;
    p2 = source.p2;
    p3 = source.p3;
    return *this;
}

ChGeometry::AABB ChTriangle::GetBoundingBox(const ChMatrix33<>& rot) const {
    ChVector<> trp1 = rot.transpose() * p1;
    ChVector<> trp2 = rot.transpose() * p2;
    ChVector<> trp3 = rot.transpose() * p3;

    AABB bbox;
    bbox.min.x() = ChMin(ChMin(trp1.x(), trp2.x()), trp3.x());
    bbox.min.y() = ChMin(ChMin(trp1.y(), trp2.y()), trp3.y());
    bbox.min.z() = ChMin(ChMin(trp1.z(), trp2.z()), trp3.z());
    bbox.max.x() = ChMax(ChMax(trp1.x(), trp2.x()), trp3.x());
    bbox.max.y() = ChMax(ChMax(trp1.y(), trp2.y()), trp3.y());
    bbox.max.z() = ChMax(ChMax(trp1.z(), trp2.z()), trp3.z());

    return bbox;
}

ChVector<> ChTriangle::Baricenter() const {
    ChVector<> mb;
    mb.x() = (p1.x() + p2.x() + p3.x()) / 3.;
    mb.y() = (p1.y() + p2.y() + p3.y()) / 3.;
    mb.z() = (p1.z() + p2.z() + p3.z()) / 3.;
    return mb;
}

bool ChTriangle::Normal(ChVector<>& N) const {
    ChVector<> u;
    u = Vsub(p2, p1);
    ChVector<> v;
    v = Vsub(p3, p1);

    ChVector<> n;
    n = Vcross(u, v);

    double len = Vlength(n);

    if (fabs(len) > EPS_TRIDEGENERATE)
        N = Vmul(n, (1.0 / len));
    else
        return false;

    return true;
}

ChVector<> ChTriangle::GetNormal() const {
    ChVector<> mn;
    Normal(mn);
    return mn;
}

bool ChTriangle::IsDegenerated() const {
    return collision::utils::DegenerateTriangle(p1, p2, p3);
}

double ChTriangle::PointTriangleDistance(ChVector<> B,           // point to be measured
                                         double& mu,             // returns U parametric coord of projection
                                         double& mv,             // returns V parametric coord of projection
                                         bool& is_into,          // returns true if projection falls on the triangle
                                         ChVector<>& Bprojected  // returns the position of the projected point
) {
    return collision::utils::PointTriangleDistance(B, p1, p2, p3, mu, mv, is_into, Bprojected);
}

void ChTriangle::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTriangle>();
    // serialize parent class
    ChGeometry::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(p1);
    marchive << CHNVP(p2);
    marchive << CHNVP(p3);
}

void ChTriangle::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChTriangle>();
    // deserialize parent class
    ChGeometry::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(p1);
    marchive >> CHNVP(p2);
    marchive >> CHNVP(p3);
}

}  // end namespace geometry
}  // end namespace chrono
