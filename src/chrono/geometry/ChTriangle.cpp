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
void ChTriangle::GetBoundingBox(double& xmin,
                                double& xmax,
                                double& ymin,
                                double& ymax,
                                double& zmin,
                                double& zmax,
                                ChMatrix33<>* Rot) const {
    if (Rot == NULL) {
        xmin = ChMin(ChMin(p1.x(), p2.x()), p3.x());
        ymin = ChMin(ChMin(p1.y(), p2.y()), p3.y());
        zmin = ChMin(ChMin(p1.z(), p2.z()), p3.z());
        xmax = ChMax(ChMax(p1.x(), p2.x()), p3.x());
        ymax = ChMax(ChMax(p1.y(), p2.y()), p3.y());
        zmax = ChMax(ChMax(p1.z(), p2.z()), p3.z());
    } else {
        ChVector<> trp1 = Rot->transpose() * p1;
        ChVector<> trp2 = Rot->transpose() * p2;
        ChVector<> trp3 = Rot->transpose() * p3;
        xmin = ChMin(ChMin(trp1.x(), trp2.x()), trp3.x());
        ymin = ChMin(ChMin(trp1.y(), trp2.y()), trp3.y());
        zmin = ChMin(ChMin(trp1.z(), trp2.z()), trp3.z());
        xmax = ChMax(ChMax(trp1.x(), trp2.x()), trp3.x());
        ymax = ChMax(ChMax(trp1.y(), trp2.y()), trp3.y());
        zmax = ChMax(ChMax(trp1.z(), trp2.z()), trp3.z());
    }
}

ChVector<> ChTriangle::Baricenter() const {
    ChVector<> mb;
    mb.x() = (p1.x() + p2.x() + p3.x()) / 3.;
    mb.y() = (p1.y() + p2.y() + p3.y()) / 3.;
    mb.z() = (p1.z() + p2.z() + p3.z()) / 3.;
    return mb;
}

void ChTriangle::CovarianceMatrix(ChMatrix33<>& C) const {
    C(0, 0) = p1.x() * p1.x() + p2.x() * p2.x() + p3.x() * p3.x();
    C(1, 1) = p1.y() * p1.y() + p2.y() * p2.y() + p3.y() * p3.y();
    C(2, 2) = p1.z() * p1.z() + p2.z() * p2.z() + p3.z() * p3.z();
    C(0, 1) = p1.x() * p1.y() + p2.x() * p2.y() + p3.x() * p3.y();
    C(0, 2) = p1.x() * p1.z() + p2.x() * p2.z() + p3.x() * p3.z();
    C(1, 2) = p1.y() * p1.z() + p2.y() * p2.z() + p3.y() * p3.z();
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
