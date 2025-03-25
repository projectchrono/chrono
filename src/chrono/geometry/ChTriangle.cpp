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
#include "chrono/utils/ChUtilsGeometry.h"

namespace chrono {

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

ChAABB ChTriangle::GetBoundingBox(const ChVector3d& P1, const ChVector3d& P2, const ChVector3d& P3) {
    ChAABB bbox;
    bbox.min.x() = std::min(std::min(P1.x(), P2.x()), P3.x());
    bbox.min.y() = std::min(std::min(P1.y(), P2.y()), P3.y());
    bbox.min.z() = std::min(std::min(P1.z(), P2.z()), P3.z());
    bbox.max.x() = std::max(std::max(P1.x(), P2.x()), P3.x());
    bbox.max.y() = std::max(std::max(P1.y(), P2.y()), P3.y());
    bbox.max.z() = std::max(std::max(P1.z(), P2.z()), P3.z());

    return bbox;
}

ChAABB ChTriangle::GetBoundingBox() const {
    return GetBoundingBox(p1, p2, p3);
}

ChVector3d ChTriangle::Baricenter() const {
    ChVector3d mb;
    mb.x() = (p1.x() + p2.x() + p3.x()) * CH_1_3;
    mb.y() = (p1.y() + p2.y() + p3.y()) * CH_1_3;
    mb.z() = (p1.z() + p2.z() + p3.z()) * CH_1_3;
    return mb;
}

bool ChTriangle::CalcNormal(const ChVector3d& p1, const ChVector3d& p2, const ChVector3d& p3, ChVector3d& N) {
    ChVector3d u;
    u = Vsub(p2, p1);
    ChVector3d v;
    v = Vsub(p3, p1);

    ChVector3d n;
    n = Vcross(u, v);

    double len = Vlength(n);

    if (fabs(len) > EPS_TRIDEGENERATE)
        N = Vmul(n, (1.0 / len));
    else
        return false;

    return true;
}

ChVector3d ChTriangle::CalcNormal(const ChVector3d& p1, const ChVector3d& p2, const ChVector3d& p3) {
    ChVector3d normal;
    CalcNormal(p1, p2, p3, normal);
    return normal;
}

bool ChTriangle::Normal(ChVector3d& N) const {
    return CalcNormal(p1, p2, p3, N);
}

ChVector3d ChTriangle::GetNormal() const {
    ChVector3d normal;
    CalcNormal(p1, p2, p3, normal);
    return normal;
}

bool ChTriangle::IsDegenerated() const {
    return utils::DegenerateTriangle(p1, p2, p3);
}

double ChTriangle::PointTriangleDistance(ChVector3d B,           // point to be measured
                                         double& mu,             // returns U parametric coord of projection
                                         double& mv,             // returns V parametric coord of projection
                                         bool& is_into,          // returns true if projection falls on the triangle
                                         ChVector3d& Bprojected  // returns the position of the projected point
) {
    return utils::PointTriangleDistance(B, p1, p2, p3, mu, mv, is_into, Bprojected);
}

void ChTriangle::SetPoints(const ChVector3d& P1, const ChVector3d& P2, const ChVector3d& P3) {
    p1 = P1;
    p2 = P2;
    p3 = P3;
}

void ChTriangle::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChTriangle>();
    // serialize parent class
    ChGeometry::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(p1);
    archive_out << CHNVP(p2);
    archive_out << CHNVP(p3);
}

void ChTriangle::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChTriangle>();
    // deserialize parent class
    ChGeometry::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(p1);
    archive_in >> CHNVP(p2);
    archive_in >> CHNVP(p3);
}

}  // end namespace chrono
