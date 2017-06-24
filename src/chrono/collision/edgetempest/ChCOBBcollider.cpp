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
#include "chrono/collision/edgetempest/ChCOBBcollider.h"
#include "chrono/collision/edgetempest/ChCGeometryCollider.h"

#include "chrono/core/ChTransform.h"

namespace chrono {
namespace collision {

CHOBBcollider::CHOBBcollider() {
}

CHOBBcollider::~CHOBBcollider() {
}

void CHOBBcollider::CollideRecurse(ChMatrix33<>& boR,
                                   Vector& boT,
                                   CHOBBTree* o1,
                                   int b1,
                                   CHOBBTree* o2,
                                   int b2,
                                   eCollMode flag) {
    this->num_bv_tests++;

    CHOBB* box1 = o1->child(b1);
    CHOBB* box2 = o2->child(b2);

    // first thing, see if we're overlapping
    if (!CHOBB::OBB_Overlap(boR, boT, box1, box2))
        return;

    // if we are, see if we test geometries next

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

    // we don't, so decide whose children to visit next

    double sz1 = box1->GetSize();
    double sz2 = box2->GetSize();

    ChMatrix33<> Rc;
    Vector Tc;

    if (l2 || (!l1 && (sz1 > sz2))) {
        int c1 = box1->GetFirstChildIndex();
        int c2 = box1->GetSecondChildIndex();

        Rc.MatrTMultiply(o1->child(c1)->Rot, boR);

        Tc = ChTransform<>::TransformParentToLocal(boT, o1->child(c1)->To, o1->child(c1)->Rot);

        CollideRecurse(Rc, Tc, o1, c1, o2, b2, flag);

        if ((flag == ChC_FIRST_CONTACT) && (this->GetNumPairs() > 0))
            return;

        Rc.MatrTMultiply(o1->child(c2)->Rot, boR);

        Tc = ChTransform<>::TransformParentToLocal(boT, o1->child(c2)->To, o1->child(c2)->Rot);

        CollideRecurse(Rc, Tc, o1, c2, o2, b2, flag);
    } else {
        int c1 = box2->GetFirstChildIndex();
        int c2 = box2->GetSecondChildIndex();

        Rc.MatrMultiply(boR, o2->child(c1)->Rot);

        Tc = ChTransform<>::TransformLocalToParent(o2->child(c1)->To, boT, boR);

        CollideRecurse(Rc, Tc, o1, b1, o2, c1, flag);

        if ((flag == ChC_FIRST_CONTACT) && (this->GetNumPairs() > 0))
            return;

        Rc.MatrMultiply(boR, o2->child(c2)->Rot);

        Tc = ChTransform<>::TransformLocalToParent(o2->child(c2)->To, boT, boR);

        CollideRecurse(Rc, Tc, o1, b1, o2, c2, flag);
    }
}

//
// PERFORM COLLISION DETECTION
//

ChNarrowPhaseCollider::eCollSuccess CHOBBcollider::ComputeCollisions(ChMatrix33<>& R1,
                                                                     Vector T1,
                                                                     ChCollisionTree* oc1,
                                                                     ChMatrix33<>& R2,
                                                                     Vector T2,
                                                                     ChCollisionTree* oc2,
                                                                     eCollMode flag) {
    double t1 = GetTime();

    // INHERIT parent class behavior

    if (ChNarrowPhaseCollider::ComputeCollisions(R1, T1, oc1, R2, T2, oc2, flag) != ChC_RESULT_OK)
        return ChC_RESULT_GENERICERROR;

    // Downcasting
    CHOBBTree* o1 = (CHOBBTree*)oc1;
    CHOBBTree* o2 = (CHOBBTree*)oc2;

    // clear the stats

    this->num_bv_tests = 0;
    this->num_geo_tests = 0;

    // compute the transform from o1->child(0) to o2->child(0)

    static ChMatrix33<> Rtemp;
    static ChMatrix33<> bR;
    static Vector bT;
    static Vector Ttemp;

    Rtemp.MatrMultiply(this->R, o2->child(0)->Rot);  // MxM(Rtemp,this->R,o2->child(0)->R);
    bR.MatrMultiply(o1->child(0)->Rot, Rtemp);       // MTxM(R,o1->child(0)->R,Rtemp);

    Ttemp = ChTransform<>::TransformLocalToParent(o2->child(0)->To, this->T,
                                                  this->R);  // MxVpV(Ttemp,this->R,o2->child(0)->To,this->T);
    Ttemp = Vsub(Ttemp, o1->child(0)->To);                   // VmV(Ttemp,Ttemp,o1->child(0)->To);

    bT = o1->child(0)->Rot.MatrT_x_Vect(Ttemp);  // MTxV(T,o1->child(0)->R,Ttemp);

    // now start with both top level BVs

    CollideRecurse(bR, bT, o1, 0, o2, 0, flag);

    double t2 = GetTime();
    this->query_time_secs = t2 - t1;

    return ChC_RESULT_OK;
}

}  // end namespace collision
}  // end namespace chrono
