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
#include "chrono/collision/edgetempest/ChCNarrowPhaseCollider.h"

namespace chrono {
namespace collision {

ChNarrowPhaseCollider::ChNarrowPhaseCollider() {
    // initially reserve a bit of space to contact array,
    // to avoid many reallocations if resizing needed for enlarging it.
    pairs.clear();
    num_collision_pairs = 0;
    num_bv_tests = 0;
    num_geo_tests = 0;
}

ChNarrowPhaseCollider::~ChNarrowPhaseCollider() {
    pairs.clear();  // should be automatic, anyway.
}

void ChNarrowPhaseCollider::ClearPairsList() {
    num_collision_pairs = 0;
    pairs.clear();
}

void ChNarrowPhaseCollider::AddCollisionPair(ChCollisionPair* mcollision) {
    pairs.push_back(*mcollision);

    num_collision_pairs++;
}

//
// PERFORM COLLISION DETECTION
//

ChNarrowPhaseCollider::eCollSuccess ChNarrowPhaseCollider::ComputeCollisions(ChMatrix33<>& aR1,
                                                                             Vector aT1,
                                                                             ChCollisionTree* o1,
                                                                             ChMatrix33<>& aR2,
                                                                             Vector aT2,
                                                                             ChCollisionTree* o2,
                                                                             eCollMode flag) {
    // collision models must be here
    if (!o1 || !o2)
        return ChC_RESULT_GENERICERROR;

    // make sure that the models are built
    if (o1->build_state != ChCollisionTree::ChC_BUILD_STATE_PROCESSED)
        return ChC_RESULT_MODELSNOTBUILT;
    if (o2->build_state != ChCollisionTree::ChC_BUILD_STATE_PROCESSED)
        return ChC_RESULT_MODELSNOTBUILT;

    // clear the stats

    this->num_bv_tests = 0;
    this->num_geo_tests = 0;

    // don't release the memory,
    // DONT reset the num_collision_pairs counter (may be multiple calls for different object couples)
    // this->ClearPairsList();

    // Precompute useful matrices

    this->R.MatrTMultiply(aR1, aR2);  //  MTxM(this->R,R1,R2);

    static Vector Ttemp;
    Ttemp = Vsub(aT2, aT1);             // VmV(Ttemp, T2, T1);
    this->T = aR1.MatrT_x_Vect(Ttemp);  // MTxV(this->T, R1, Ttemp);

    this->T1 = aT1;
    this->T2 = aT2;
    this->R1.CopyFromMatrix(aR1);
    this->R2.CopyFromMatrix(aR2);

    //
    // CHILD CLASSES SHOULD IMPLEMENT THE REST....
    // .....(ex. code for collisions between AABB and AABB models, etc.
    //

    return ChC_RESULT_OK;
}

}  // end namespace collision
}  // end namespace chrono
