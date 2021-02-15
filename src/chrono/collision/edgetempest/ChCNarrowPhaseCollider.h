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

#ifndef CHC_NARROWPHASECOLLIDER_H
#define CHC_NARROWPHASECOLLIDER_H

#include "chrono/collision/edgetempest/ChCCompile.h"
#include "chrono/collision/edgetempest/ChCCollisionTree.h"
#include "chrono/collision/ChCollisionPair.h"

namespace chrono {
/// Namespace for classes devoted to
/// collision detection.
namespace collision {

///
/// Base class for the narrow-phase collision detection algorithm (collision
/// between a single couple of models).
/// It just implements algorithms to handle the list of the contact
/// pairs, but child classes must implement the most important function,
/// that is the ComputeCollisions() function, which should find the
/// contact pairs between two collision models. \see ChCollisionTree
///

class ChNarrowPhaseCollider {
  public:
    /// Constructor
    ChNarrowPhaseCollider();
    /// Destructor
    virtual ~ChNarrowPhaseCollider();

    /// Child classes must use this function to insert a new CollisionPair
    /// into  the vector of contacts. (The pair is automatically copied and allocated).
    void AddCollisionPair(ChCollisionPair* mcollision);

    /// Free the vector of contact pairs.
    void ClearPairsList();

    /// Returns 1 if colliding, return 0 if not colliding (after calling ComputeCollisions())
    int IsColliding() { return (num_collision_pairs > 0); }

    /// Returns number of collision pairs (after calling ComputeCollisions())
    int GetNumPairs() { return num_collision_pairs; }

    /// Returns reference to vector of collision pairs (after calling ComputeCollisions())
    std::vector<ChCollisionPair>& GetPairs() { return pairs; }

    /// Returns the number of bounding volume tests, for statistics
    int GetNumBVTests() { return num_bv_tests; }
    /// Returns the number of intersection between geometric primitives, for statistics
    int NumGeoTests() { return num_geo_tests; }
    /// Time of execution of last call to ComputeCollisions()
    double QueryTimeSecs() { return query_time_secs; }

    // PERFORM THE COLLISION DETECTION

    enum eCollMode {
        ChC_ALL_CONTACTS = 1,   ///< find all possible contact pairs
        ChC_FIRST_CONTACT = 2,  ///< report first intersecting entity and shortcircuit search
    };
    enum eCollSuccess {
        ChC_RESULT_OK = 1,              ///< ComputeCollisions() performed ok
        ChC_RESULT_MODELSNOTBUILT = 2,  ///< No collision query was possible: models not correctly built!
        ChC_RESULT_GENERICERROR = 100,  ///< ComputeCollisions() generic failure to performs
    };
    /// Perform collision detection between two
    /// collision models (each belonging to a different rigid
    /// body, for example), given the absolute orientations and
    /// translations of the two models.
    /// The identified contacts will be inserted in the Contacts vector.
    /// All child classes must implement this.
    ///
    virtual eCollSuccess ComputeCollisions(ChMatrix33<>& R1,     ///< abs.rotation of first model
                                           Vector T1,            ///< abs.position of first model
                                           ChCollisionTree* o1,  ///< reference to first model
                                           ChMatrix33<>& R2,     ///< abs.rotation of second model
                                           Vector T2,            ///< abs position of second model
                                           ChCollisionTree* o2,  ///< reference to second model
                                           eCollMode flag        ///< mode of collision \see eCollMode
                                           );

  protected:
    // DATA

    int num_collision_pairs;             ///< number of collision pairs (contact points) detected
    std::vector<ChCollisionPair> pairs;  ///< vector of collision pairs (contact points)

    int num_bv_tests;        ///< number of bounding volume tests, for statistics
    int num_geo_tests;       ///< number of test of intersection between primitive pairs
    double query_time_secs;  ///< time of execution of last call to collision detection

    // Used internally, initialized after call to base function ComputeCollisions()
    // and then they can be used by children classes which inherit specific ComputeCollisions() features
    ChMatrix33<> R;   // relative rotation of model 2 respect to model 1
    Vector T;         // relative position of model 2 respect to model 1
    ChMatrix33<> R1;  // abs.rotation of model1
    Vector T1;        // abs.position of model1
    ChMatrix33<> R2;  // abs.rotation of model2
    Vector T2;        // abs.position of model2
};

}  // end namespace collision
}  // end namespace chrono

#endif
