//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include <cstdlib>
#include <cmath>

#include "chrono/collision/edgetempest/ChCOBB.h"
#include "chrono/collision/edgetempest/ChCMatVec.h"
#include "chrono/collision/ChCCollisionPair.h"

namespace chrono {
namespace collision {

CHOBB::CHOBB() {
    first_child = 0;

    Rot.Set33Identity();

    To = VNULL;
    d = VNULL;
}

CHOBB::~CHOBB() {
}

void CHOBB::FitToGeometries(ChMatrix33<>& O,
                            std::vector<geometry::ChGeometry*> mgeos,
                            int firstgeo,
                            int ngeos,
                            double envelope) {
    // store orientation

    this->Rot.CopyFromMatrix(O);

    double minx, maxx, miny, maxy, minz, maxz;
    minx = miny = minz = +10e20;
    maxx = maxy = maxz = -10e20;

    Vector c;

    geometry::ChGeometry* nit;
    for (int count = 0; count < ngeos; ++count) {
        nit = mgeos[firstgeo + count];
        if (nit) {
            nit->InflateBoundingBox(minx, maxx, miny, maxy, minz, maxz, &Rot);
        }
    }

    c.x() = 0.5 * (maxx + minx);
    c.y() = 0.5 * (maxy + miny);
    c.z() = 0.5 * (maxz + minz);
    To = Rot.Matr_x_Vect(c);

    d.x() = 0.5 * (maxx - minx) + envelope;
    d.y() = 0.5 * (maxy - miny) + envelope;
    d.z() = 0.5 * (maxz - minz) + envelope;
}

bool CHOBB::OBB_Overlap(ChMatrix33<>& B, Vector T, CHOBB* b1, CHOBB* b2) {
    return OBB_Overlap(B, T, b1->d, b2->d);
}

bool CHOBB::OBB_Overlap(ChMatrix33<>& B, Vector T, Vector a, Vector b) {
    double t, s;
    int r;
    static ChMatrix33<> Bf;
    const double reps = (double)1e-6;

    // Vector& a = b1->d;
    // Vector& b = b2->d;

    // Bf = fabs(B)
    Bf(0, 0) = myfabs(B.Get33Element(0, 0));
    Bf(0, 0) += reps;
    Bf(0, 1) = myfabs(B.Get33Element(0, 1));
    Bf(0, 1) += reps;
    Bf(0, 2) = myfabs(B.Get33Element(0, 2));
    Bf(0, 2) += reps;
    Bf(1, 0) = myfabs(B.Get33Element(1, 0));
    Bf(1, 0) += reps;
    Bf(1, 1) = myfabs(B.Get33Element(1, 1));
    Bf(1, 1) += reps;
    Bf(1, 2) = myfabs(B.Get33Element(1, 2));
    Bf(1, 2) += reps;
    Bf(2, 0) = myfabs(B.Get33Element(2, 0));
    Bf(2, 0) += reps;
    Bf(2, 1) = myfabs(B.Get33Element(2, 1));
    Bf(2, 1) += reps;
    Bf(2, 2) = myfabs(B.Get33Element(2, 2));
    Bf(2, 2) += reps;

    // if any of these tests are one-sided, then the polyhedra are disjoint
    r = 1;

    // A1 x A2 = A0
    t = myfabs(T.x());

    r &= (t <= (a.x() + b.x() * Bf.Get33Element(0, 0) + b.y() * Bf.Get33Element(0, 1) + b.z() * Bf.Get33Element(0, 2)));
    if (!r)
        return false;

    // B1 x B2 = B0
    s = T.x() * B.Get33Element(0, 0) + T.y() * B.Get33Element(1, 0) + T.z() * B.Get33Element(2, 0);
    t = myfabs(s);

    r &= (t <= (b.x() + a.x() * Bf.Get33Element(0, 0) + a.y() * Bf.Get33Element(1, 0) + a.z() * Bf.Get33Element(2, 0)));
    if (!r)
        return false;

    // A2 x A0 = A1
    t = myfabs(T.y());

    r &= (t <= (a.y() + b.x() * Bf.Get33Element(1, 0) + b.y() * Bf.Get33Element(1, 1) + b.z() * Bf.Get33Element(1, 2)));
    if (!r)
        return false;

    // A0 x A1 = A2
    t = myfabs(T.z());

    r &= (t <= (a.z() + b.x() * Bf.Get33Element(2, 0) + b.y() * Bf.Get33Element(2, 1) + b.z() * Bf.Get33Element(2, 2)));
    if (!r)
        return false;

    // B2 x B0 = B1
    s = T.x() * B.Get33Element(0, 1) + T.y() * B.Get33Element(1, 1) + T.z() * B.Get33Element(2, 1);
    t = myfabs(s);

    r &= (t <= (b.y() + a.x() * Bf.Get33Element(0, 1) + a.y() * Bf.Get33Element(1, 1) + a.z() * Bf.Get33Element(2, 1)));
    if (!r)
        return false;

    // B0 x B1 = B2
    s = T.x() * B.Get33Element(0, 2) + T.y() * B.Get33Element(1, 2) + T.z() * B.Get33Element(2, 2);
    t = myfabs(s);

    r &= (t <= (b.z() + a.x() * Bf.Get33Element(0, 2) + a.y() * Bf.Get33Element(1, 2) + a.z() * Bf.Get33Element(2, 2)));
    if (!r)
        return false;

    // A0 x B0
    s = T.z() * B.Get33Element(1, 0) - T.y() * B.Get33Element(2, 0);
    t = myfabs(s);

    r &= (t <= (a.y() * Bf.Get33Element(2, 0) + a.z() * Bf.Get33Element(1, 0) + b.y() * Bf.Get33Element(0, 2) +
                b.z() * Bf.Get33Element(0, 1)));
    if (!r)
        return false;

    // A0 x B1
    s = T.z() * B.Get33Element(1, 1) - T.y() * B.Get33Element(2, 1);
    t = myfabs(s);

    r &= (t <= (a.y() * Bf.Get33Element(2, 1) + a.z() * Bf.Get33Element(1, 1) + b.x() * Bf.Get33Element(0, 2) +
                b.z() * Bf.Get33Element(0, 0)));
    if (!r)
        return false;

    // A0 x B2
    s = T.z() * B.Get33Element(1, 2) - T.y() * B.Get33Element(2, 2);
    t = myfabs(s);

    r &= (t <= (a.y() * Bf.Get33Element(2, 2) + a.z() * Bf.Get33Element(1, 2) + b.x() * Bf.Get33Element(0, 1) +
                b.y() * Bf.Get33Element(0, 0)));
    if (!r)
        return false;

    // A1 x B0
    s = T.x() * B.Get33Element(2, 0) - T.z() * B.Get33Element(0, 0);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf.Get33Element(2, 0) + a.z() * Bf.Get33Element(0, 0) + b.y() * Bf.Get33Element(1, 2) +
                b.z() * Bf.Get33Element(1, 1)));
    if (!r)
        return false;

    // A1 x B1
    s = T.x() * B.Get33Element(2, 1) - T.z() * B.Get33Element(0, 1);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf.Get33Element(2, 1) + a.z() * Bf.Get33Element(0, 1) + b.x() * Bf.Get33Element(1, 2) +
                b.z() * Bf.Get33Element(1, 0)));
    if (!r)
        return false;

    // A1 x B2
    s = T.x() * B.Get33Element(2, 2) - T.z() * B.Get33Element(0, 2);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf.Get33Element(2, 2) + a.z() * Bf.Get33Element(0, 2) + b.x() * Bf.Get33Element(1, 1) +
                b.y() * Bf.Get33Element(1, 0)));
    if (!r)
        return false;

    // A2 x B0
    s = T.y() * B.Get33Element(0, 0) - T.x() * B.Get33Element(1, 0);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf.Get33Element(1, 0) + a.y() * Bf.Get33Element(0, 0) + b.y() * Bf.Get33Element(2, 2) +
                b.z() * Bf.Get33Element(2, 1)));
    if (!r)
        return false;

    // A2 x B1
    s = T.y() * B.Get33Element(0, 1) - T.x() * B.Get33Element(1, 1);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf.Get33Element(1, 1) + a.y() * Bf.Get33Element(0, 1) + b.x() * Bf.Get33Element(2, 2) +
                b.z() * Bf.Get33Element(2, 0)));
    if (!r)
        return false;

    // A2 x B2
    s = T.y() * B.Get33Element(0, 2) - T.x() * B.Get33Element(1, 2);
    t = myfabs(s);

    r &= (t <= (a.x() * Bf.Get33Element(1, 2) + a.y() * Bf.Get33Element(0, 2) + b.x() * Bf.Get33Element(2, 1) +
                b.y() * Bf.Get33Element(2, 0)));
    if (!r)
        return false;

    return true;  // no separation: BV collide!!
}

}  // end namespace collision
}  // end namespace chrono
