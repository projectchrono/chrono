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

#ifndef CHC_COLLISIONTREE_H
#define CHC_COLLISIONTREE_H

#include <vector>

#include "chrono/geometry/ChGeometry.h"
#include "chrono/collision/edgetempest/ChCAbsoluteAABB.h"

#define CH_COLL_ENVELOPE 0.0022

namespace chrono {

// forward references
class ChBody;

namespace collision {

///
/// Class defining a hierarchical tree of geometric trees, for
/// fast collision-detection purposes. To be specialized and
/// implemented by child classes such as CHOBBTree() or CHAABBTree()
///

class ChCollisionTree {
  public:
    ChCollisionTree();
    virtual ~ChCollisionTree();

    /// Deletes all inserted geometries
    /// MUST be inherited by child classes! (ex for resetting also BV hierarchies)
    virtual int ResetModel();

    /// Add a ChGeometry in this model.
    /// All the added geometries will be automatically deleted when this
    /// model is deleted.
    virtual int AddGeometry(geometry::ChGeometry* mgeo);

    /// Builds the BV hierarchy
    /// MUST be inherited by child classes! (ex for building BV hierarchies)
    /// The 'out_layer' parameter can be used if contacts have to be detected
    /// even at a distance='envelope' from the surface (shortly, the effect
    /// is that bounding boxes are 'inflated' by this parameter).
    virtual int BuildModel(double envelope = 0.);

    /// This function can be used if you want to easily scan the hierarchy
    /// of bounding boxes. You must provide a callback to a function which
    /// will be automatically called per each bounding box in this tree.
    /// Note that the callback will get info about the rotation Rot of the
    /// box, its position Pos, its size d, its level. Also you can pass generic
    /// user data via a pointer.
    /// This is not implemented here, since there are no boxes in the base class,
    /// but all children classes implementing some type of bounding box should implement this.
    ///  \return the number of scanned boxes.
    virtual int TraverseBoundingBoxes(
        void callback(ChMatrix33<>& Rot, Vector& Pos, Vector& d, int level, void* userdata),
        void* userdata) {
        return 0;
    };

    /// Compute bounding box (in model reference frame).
    virtual void GetBoundingBox(ChVector<>& cmin,
                                ChVector<>& cmax,
                                const ChMatrix33<>& rot);

    /// Gets the pointer to the owner rigid body
    ChBody* GetBody() { return m_body; }
    /// Sets the pointer to the owner rigid body
    void SetBody(ChBody* mbo) { m_body = mbo; }

    /// Given pos&rot of body, this function recomputes the absolute-axis-aligned
    /// bounding box in order to enclose all the geometries of the model.
    /// Also initializes the ChAbsoluteAABB pointers, useful for sweep&prune.
    void UpdateAbsoluteAABB(double envelope);

  public:
    enum eChBuildState {
        ChC_BUILD_STATE_MODIFIED,  ///< someone added/modified geometry, so it requires tree rebuilding
        ChC_BUILD_STATE_PROCESSED  ///< after tree has been built, ready to use
    };

    /// Build state.  \see eChBuildState
    eChBuildState build_state;

    /// Vector of pointers to geometric objects.
    std::vector<geometry::ChGeometry*> geometries;

    /// Total number of geometric objects added
    int num_geometries;

    /// Used by internal algorithms, contains the last accessed geometry.
    geometry::ChGeometry* last_geometry;

  private:
    ChBody* m_body;

  public:
    ChAbsoluteAABB<ChCollisionTree> m_absoluteAABB;
};

//
// ERROR CODES
//

const int ChC_OK = 0;
// Used by all API routines upon successful completion except
// constructors and destructors

const int ChC_ERR_MODEL_OUT_OF_MEMORY = -1;
// Returned when an API function cannot obtain enough memory to
// store or process a ChCollisionTree object.

const int ChC_ERR_OUT_OF_MEMORY = -2;
// Returned when a query cannot allocate enough storage to
// compute or hold query information.  In this case, the returned
// data should not be trusted.

const int ChC_ERR_UNPROCESSED_MODEL = -3;
// Returned when an unprocessed model is passed to a function which
// expects only processed models, such as PQP_Collide() or
// PQP_Distance().

const int ChC_ERR_BUILD_EMPTY_MODEL = -5;
// Returned when EndModel() is called on a model to which no
// triangles have been added.  This is similar in spirit to the
// OUT_OF_SEQUENCE return code, except that the requested operation
// has FAILED -- the model remains "unprocessed", and the client may
// NOT use it in queries.

}  // end namespace collision
}  // end namespace chrono

#endif
