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

#include <cstdlib>
#include <cmath>

#include "chrono/collision/edgetempest/ChCAABB.h"
#include "chrono/collision/edgetempest/ChCMatVec.h"
#include "chrono/collision/ChCollisionPair.h"

namespace chrono {
namespace collision {

CHAABB::CHAABB() {
    first_child = 0;

    To = VNULL;
    d = VNULL;
}

CHAABB::~CHAABB() {
}

void CHAABB::FitToGeometries(std::vector<geometry::ChGeometry*> mgeos, int firstgeo, int ngeos, double envelope) {
    ChVector<> cmin(+std::numeric_limits<double>::max());
    ChVector<> cmax(-std::numeric_limits<double>::max());

    geometry::ChGeometry* nit = mgeos[firstgeo];
    for (int count = 0; count < ngeos; ++count) {
        nit = mgeos[firstgeo + count];
        if (nit) {
            nit->InflateBoundingBox(cmin, cmax, ChMatrix33<>(1));
        }
    }

    To = (cmin + cmax) / 2;
    d = (cmax - cmin) / 2 + ChVector<>(envelope);
}

bool CHAABB::AABB_Overlap(ChMatrix33<>& B, Vector T, CHAABB* b1, CHAABB* b2) {
    ChMatrix33<> Bf;
    const double reps = (double)1e-6;

    // Bf = fabs(B)
    Bf(0, 0) = myfabs(B(0, 0));
    Bf(0, 0) += reps;
    Bf(0, 1) = myfabs(B(0, 1));
    Bf(0, 1) += reps;
    Bf(0, 2) = myfabs(B(0, 2));
    Bf(0, 2) += reps;
    Bf(1, 0) = myfabs(B(1, 0));
    Bf(1, 0) += reps;
    Bf(1, 1) = myfabs(B(1, 1));
    Bf(1, 1) += reps;
    Bf(1, 2) = myfabs(B(1, 2));
    Bf(1, 2) += reps;
    Bf(2, 0) = myfabs(B(2, 0));
    Bf(2, 0) += reps;
    Bf(2, 1) = myfabs(B(2, 1));
    Bf(2, 1) += reps;
    Bf(2, 2) = myfabs(B(2, 2));
    Bf(2, 2) += reps;

    return AABB_Overlap(B, Bf, T, b1, b2);
}

bool CHAABB::AABB_Overlap(ChMatrix33<>& B, ChMatrix33<>& Bf, Vector T, CHAABB* b1, CHAABB* b2) {
    double t, s;
    int r;

    Vector& a = b1->d;
    Vector& b = b2->d;

    // if any of these tests are one-sided, then the polyhedra are disjoint
    r = 1;

    // A1 x A2 = A0
    t = myfabs(T.x());

    r &= (t <= (a.x() + b.x() * Bf(0, 0) + b.y() * Bf(0, 1) + b.z() * Bf(0, 2)));
    if (!r)
        return false;

    // B1 x B2 = B0
    s = T.x() * B(0, 0) + T.y() * B(1, 0) + T.z() * B(2, 0);
    t = myfabs(s);

    r &= (t <= (b.x() + a.x() * Bf(0, 0) + a.y() * Bf(1, 0) + a.z() * Bf(2, 0)));
    if (!r)
        return false;

    // A2 x A0 = A1
    t = myfabs(T.y());

    r &= (t <= (a.y() + b.x() * Bf(1, 0) + b.y() * Bf(1, 1) + b.z() * Bf(1, 2)));
    if (!r)
        return false;

    // A0 x A1 = A2
    t = myfabs(T.z());

    r &= (t <= (a.z() + b.x() * Bf(2, 0) + b.y() * Bf(2, 1) + b.z() * Bf(2, 2)));
    if (!r)
        return false;

    // B2 x B0 = B1
    s = T.x() * B(0, 1) + T.y() * B(1, 1) + T.z() * B(2, 1);
    t = myfabs(s);

    r &= (t <= (b.y() + a.x() * Bf(0, 1) + a.y() * Bf(1, 1) + a.z() * Bf(2, 1)));
    if (!r)
        return false;

    // B0 x B1 = B2
    s = T.x() * B(0, 2) + T.y() * B(1, 2) + T.z() * B(2, 2);
    t = myfabs(s);

    r &= (t <= (b.z() + a.x() * Bf(0, 2) + a.y() * Bf(1, 2) + a.z() * Bf(2, 2)));
    if (!r)
        return false;

    // A0 x B0
    s = T.z() * B(1, 0) - T.y() * B(2, 0);
    t = myfabs(s);

    r &= (t <= (a.y() * Bf(2, 0) + a.z() * Bf(1, 0) + b.y() * Bf(0, 2) +
                b.z() * Bf(0, 1)));
    if (!r)
        return false;

    // A0 x B1
    s = T.z() * B(1, 1) - T.y() * B(2, 1);
    t = myfabs(s);

    r &= (t <= (a.y() * Bf(2, 1) + a.z() * Bf(1, 1) + b.x() * Bf(0, 2) +
                b.z() * Bf(0, 0)));
    if (!r)
        return false;

    // A0 x B2
    s = T.z() * B(1, 2) - T.y() * B(2, 2);
    t = myfabs(s);

    r &= (t <= (a.y() * Bf(2, 2) + a.z() * Bf(1, 2) + b.x() * Bf(0, 1) +
                b.y() * Bf(0, 0)));
    if (!r)
        return false;

    // A1 x B0
    s = T.x() * B(2, 0) - T.z() * B(0, 0);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf(2, 0) + a.z() * Bf(0, 0) + b.y() * Bf(1, 2) +
                b.z() * Bf(1, 1)));
    if (!r)
        return false;

    // A1 x B1
    s = T.x() * B(2, 1) - T.z() * B(0, 1);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf(2, 1) + a.z() * Bf(0, 1) + b.x() * Bf(1, 2) +
                b.z() * Bf(1, 0)));
    if (!r)
        return false;

    // A1 x B2
    s = T.x() * B(2, 2) - T.z() * B(0, 2);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf(2, 2) + a.z() * Bf(0, 2) + b.x() * Bf(1, 1) +
                b.y() * Bf(1, 0)));
    if (!r)
        return false;

    // A2 x B0
    s = T.y() * B(0, 0) - T.x() * B(1, 0);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf(1, 0) + a.y() * Bf(0, 0) + b.y() * Bf(2, 2) +
                b.z() * Bf(2, 1)));
    if (!r)
        return false;

    // A2 x B1
    s = T.y() * B(0, 1) - T.x() * B(1, 1);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf(1, 1) + a.y() * Bf(0, 1) + b.x() * Bf(2, 2) +
                b.z() * Bf(2, 0)));
    if (!r)
        return false;

    // A2 x B2
    s = T.y() * B(0, 2) - T.x() * B(1, 2);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf(1, 2) + a.y() * Bf(0, 2) + b.x() * Bf(2, 1) +
                b.y() * Bf(2, 0)));
    if (!r)
        return false;

    return true;  // no separation: BV collide
}

}  // end namespace collision
}  // end namespace chrono
