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

#ifndef CHC_OBBTREE
#define CHC_OBBTREE

#include "chrono/collision/edgetempest/ChCCollisionTree.h"
#include "chrono/collision/edgetempest/ChCOBB.h"

namespace chrono {
namespace collision {

///
/// Class containing a tree of Object oriented boxes OOB,
/// ready for collision detection.
///

class CHOBBTree : public ChCollisionTree {
  public:
    CHOBBTree();
    ~CHOBBTree();

    /// Deletes all inserted geometries and resets bounding box hierarchies
    int ResetModel();

    /// Rebuilds the OBB BV hierarchy
    /// (boxes may be inflated by an 'envelope' amount).
    int BuildModel(double envelope = 0.);

    /// This function can be used if you want to easily scan the hierarchy
    /// of bounding boxes. You must provide a callback to a function which
    /// will be automatically called per each bounding box in this tree.
    /// Note that the callback will get info about the rotation Rot of the
    /// box, its position Pos, its size d, its level. Also you can pass generic
    /// user data via a pointer.  \return the number of scanned boxes.
    int TraverseBoundingBoxes(void callback(ChMatrix33<>& Rot, Vector& Pos, Vector& d, int level, void* userdata),
                              void* userdata);

    /// Returns the n-th child of the tree.
    CHOBB* child(int n) { return &b[n]; }

  public:
    /// Vector of AABB bounding boxes.
    std::vector<CHOBB> b;

    /// Used by internal algorithms
    int current_box;

    /// From geometric objects, builds the hierarchy of BV bounding volumes
    int build_model(double envelope);
};

}  // end namespace collision
}  // end namespace chrono

#endif
