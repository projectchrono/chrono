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

#ifndef CHC_OBBCOLLIDER
#define CHC_OBBCOLLIDER

#include "chrono/collision/ChCollisionPair.h"

#include "chrono/collision/edgetempest/ChCCompile.h"
#include "chrono/collision/edgetempest/ChCOBBTree.h"
#include "chrono/collision/edgetempest/ChCNarrowPhaseCollider.h"

namespace chrono {
namespace collision {

///
/// This class specializes the ChNarrowPhaseCollider base class, in order
/// to compute the case of collisions between two collision models
/// based on AABB trees.
///

class CHOBBcollider : public ChNarrowPhaseCollider {
  public:
    CHOBBcollider();
    ~CHOBBcollider();

    /// Perform collision detection between two OBB-based
    /// collision models (each belonging to a different rigid
    /// body, for example), given the absolute orientations and
    /// translations of the two models.
    /// The identified contacts will be inserted in the Contacts vector.
    ///
    eCollSuccess ComputeCollisions(ChMatrix33<>& R1,
                                   Vector T1,
                                   ChCollisionTree* oc1,
                                   ChMatrix33<>& R2,
                                   Vector T2,
                                   ChCollisionTree* oc2,
                                   eCollMode flag);

  private:
    void CollideRecurse(ChMatrix33<>& R, Vector& T, CHOBBTree* o1, int b1, CHOBBTree* o2, int b2, eCollMode flag);
};

}  // end namespace collision
}  // end namespace chrono

#endif
