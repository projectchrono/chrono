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

#include <cstdio>
#include <cstring>

#include "chrono/collision/edgetempest/ChCMatVec.h"
#include "chrono/collision/edgetempest/ChCGetTime.h"
#include "chrono/collision/edgetempest/ChCAABBcollider.h"
#include "chrono/collision/edgetempest/ChCGeometryCollider.h"

namespace chrono {
namespace collision {

CHAABBcollider::CHAABBcollider() {
    this->Rabs.Set33Identity();
}

CHAABBcollider::~CHAABBcollider() {
}

void CHAABBcollider::CollideRecurse(CHAABBTree* o1, int b1, CHAABBTree* o2, int b2, eCollMode flag) {
    CHAABB* box1 = o1->child(b1);
    CHAABB* box2 = o2->child(b2);

    this->num_bv_tests++;

    // first thing, see if we're overlapping

    static Vector Translation;
    Translation = Vsub(this->T, box1->To);
    Translation = Vadd(Translation, this->R.Matr_x_Vect(box2->To));

    if (!CHAABB::AABB_Overlap(this->R, this->Rabs, Translation, box1, box2))
        return;

    // if we are, see if we test triangles next

    int l1 = box1->IsLeaf();
    int l2 = box2->IsLeaf();

    //
    // CASE TWO LEAVES
    //

    if (l1 && l2) {
        this->num_geo_tests++;

        // transform the points in b2 into space of b1, then compare

        geometry::ChGeometry* mgeo1 = o1->geometries[box1->GetGeometryIndex()];
        geometry::ChGeometry* mgeo2 = o2->geometries[box2->GetGeometryIndex()];

        bool just_intersect = false;
        if (flag == ChNarrowPhaseCollider::ChC_FIRST_CONTACT)
            just_intersect = true;

        ChGeometryCollider::ComputeCollisions(*mgeo1, &this->R1, &this->T1, *mgeo2, &this->R2, &this->T2, *this,
                                              &this->R, &this->T, just_intersect);
        return;
    }

    // we dont, so decide whose children to visit next

    double sz1 = box1->GetSize();
    double sz2 = box2->GetSize();

    if (l2 || (!l1 && (sz1 > sz2))) {
        int c1 = box1->GetFirstChildIndex();
        int c2 = box1->GetSecondChildIndex();

        CollideRecurse(o1, c1, o2, b2, flag);

        if ((flag == ChC_FIRST_CONTACT) && (this->GetNumPairs() > 0))
            return;

        CollideRecurse(o1, c2, o2, b2, flag);
    } else {
        int c1 = box2->GetFirstChildIndex();
        int c2 = box2->GetSecondChildIndex();

        CollideRecurse(o1, b1, o2, c1, flag);

        if ((flag == ChC_FIRST_CONTACT) && (this->GetNumPairs() > 0))
            return;

        CollideRecurse(o1, b1, o2, c2, flag);
    }
}

//
// PERFORM COLLISION DETECTION
//

ChNarrowPhaseCollider::eCollSuccess CHAABBcollider::ComputeCollisions(ChMatrix33<>& R1,
                                                                      Vector T1,
                                                                      ChCollisionTree* oc1,
                                                                      ChMatrix33<>& R2,
                                                                      Vector T2,
                                                                      ChCollisionTree* oc2,
                                                                      eCollMode flag) {
    double t1 = GetTime();

    // INHERIT parent class behaviour

    if (ChNarrowPhaseCollider::ComputeCollisions(R1, T1, oc1, R2, T2, oc2, flag) != ChC_RESULT_OK)
        return ChC_RESULT_GENERICERROR;

    // Downcasting
    CHAABBTree* o1 = (CHAABBTree*)oc1;
    CHAABBTree* o2 = (CHAABBTree*)oc2;

    // clear the stats

    this->num_bv_tests = 0;
    this->num_geo_tests = 0;

    // Precompute the Rabs matrix, to be used many times in AABB collisions

    const double reps = (double)1e-6;
    // Rabs = fabs(R)+eps
    Rabs(0, 0) = myfabs(R.Get33Element(0, 0));
    Rabs(0, 0) += reps;
    Rabs(0, 1) = myfabs(R.Get33Element(0, 1));
    Rabs(0, 1) += reps;
    Rabs(0, 2) = myfabs(R.Get33Element(0, 2));
    Rabs(0, 2) += reps;
    Rabs(1, 0) = myfabs(R.Get33Element(1, 0));
    Rabs(1, 0) += reps;
    Rabs(1, 1) = myfabs(R.Get33Element(1, 1));
    Rabs(1, 1) += reps;
    Rabs(1, 2) = myfabs(R.Get33Element(1, 2));
    Rabs(1, 2) += reps;
    Rabs(2, 0) = myfabs(R.Get33Element(2, 0));
    Rabs(2, 0) += reps;
    Rabs(2, 1) = myfabs(R.Get33Element(2, 1));
    Rabs(2, 1) += reps;
    Rabs(2, 2) = myfabs(R.Get33Element(2, 2));
    Rabs(2, 2) += reps;

    // Now start with both top level BVs and recurse...

    CollideRecurse(o1, 0, o2, 0, flag);

    double t2 = GetTime();
    this->query_time_secs = t2 - t1;

    return ChC_RESULT_OK;
}

}  // end namespace collision
}  // end namespace chrono
