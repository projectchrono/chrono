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

#ifndef CHC_GEOMETRYCOLLIDER_H
#define CHC_GEOMETRYCOLLIDER_H

#include <cmath>

#include "chrono/collision/edgetempest/ChCNarrowPhaseCollider.h"
#include "chrono/geometry/ChGeometry.h"
#include "chrono/geometry/ChSphere.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChBox.h"

namespace chrono {
namespace collision {

///
/// Static functions to get contacts between geometry pairs of
/// same or different kinds. These functions are automatically
/// called, for example, by the collision engine when it
/// discovers that two AABB or two OBB intersects: in such a
/// case, the enclosed geometries are tested more precisely using
/// the methods in this class.
///

class ChGeometryCollider {
  public:
    /// GEOMETRY - GEOMETRY  generic case (dispatches all pair sub cases)
    /// Function which computes collisions between two generic
    /// geometry objects \see ChGeometry, given relative orientation
    /// of second geometry respect to first.
    /// Run time type checking of geometric object will automatically
    /// call the appropriate functions for triangle-triangle or triangle-sphere
    /// etc. etc cases.
    /// The computed contacts are inserted in the collision arrays of
    /// the ChNarrowPhaseCollider object \see ChNarrowPhaseCollider passed as parameter.
    ///   \return number of found contacts
    ///
    static int ComputeCollisions(
        geometry::ChGeometry& mgeo1,       ///< first geometric primitive
        ChMatrix33<>* R1,                  ///< absolute rotation of model enclosing first geometric primitive
        Vector* T1,                        ///< absolute position of model enclosing primitive
        geometry::ChGeometry& mgeo2,       ///< second geometric primitive
        ChMatrix33<>* R2,                  ///< absolute rotation of model enclosing second geometric primitive
        Vector* T2,                        ///< absolute position of model enclosing second geometric primitive
        ChNarrowPhaseCollider& mcollider,  ///< reference to a collider to store contacts into
        ChMatrix33<>* relRot = NULL,  ///< relative rotation of mgeo2 respect to mgeo1, if available (for optimization)
        Vector* relPos = NULL,        ///< relative position of mgeo2 respect to mgeo1, if available (for optimization)
        bool just_intersection =
            false  ///< if true, just report if intersecting (no further calculus on normal, depht, etc.)
        );

    // Following functions are for specific cases of colliding
    // geometries. Using the ComputeCollisions(), one of the following is
    // automatically used.

    /// SPHERE - SPHERE specific case
    ///
    static int ComputeSphereSphereCollisions(
        geometry::ChSphere& mgeo1,         ///< first sphere
        Vector* c1,                        ///< absolute position of 1st center
        geometry::ChSphere& mgeo2,         ///< second sphere
        Vector* c2,                        ///< absolute position of 2nd center
        ChNarrowPhaseCollider& mcollider,  ///< reference to a collider to store contacts into
        bool just_intersection  ///< if true, just report if intersecting (no further calculus on normal, depht, etc.)
        );

    /// SPHERE - TRIANGLE specific case
    ///
    static int ComputeSphereTriangleCollisions(
        geometry::ChSphere& mgeo1,         ///< sphere
        Vector* c1,                        ///< absolute position of 1st center
        geometry::ChTriangle& mgeo2,       ///< triangle
        ChMatrix33<>* R2,                  ///< absolute rotation of 2nd model (with triangle)
        Vector* T2,                        ///< absolute position of 2nd model (with triangle)
        ChNarrowPhaseCollider& mcollider,  ///< reference to a collider to store contacts into
        bool just_intersection,  ///< if true, just report if intersecting (no further calculus on normal, depht, etc.)
        bool swap_pairs          ///< if true, pairs are reported as Triangle-Sphere (triangle will be the main ref.)
        );

    /// BOX - BOX specific case
    ///
    static int ComputeBoxBoxCollisions(
        geometry::ChBox& mgeo1,            ///< box 1
        ChMatrix33<>* R1,                  ///< absolute rotation of 1st model (with box 1)
        Vector* T1,                        ///< absolute position of 1st model (with box 1)
        geometry::ChBox& mgeo2,            ///< box 2
        ChMatrix33<>* R2,                  ///< absolute rotation of 2nd model (with box 2)
        Vector* T2,                        ///< absolute position of 2nd model (with box 2)
        ChNarrowPhaseCollider& mcollider,  ///< reference to a collider to store contacts into
        bool just_intersection  ///< if true, just report if intersecting (no further calculus on normal, depht, etc.)
        );

    /// SPHERE - BOX specific case
    ///
    static int ComputeSphereBoxCollisions(
        geometry::ChSphere& mgeo1,         ///< sphere
        Vector* c1,                        ///< absolute position of center
        geometry::ChBox& mgeo2,            ///< box
        ChMatrix33<>* R2,                  ///< absolute rotation of 2nd model (with box)
        Vector* T2,                        ///< absolute position of 2nd model (with box)
        ChNarrowPhaseCollider& mcollider,  ///< reference to a collider to store contacts into
        bool just_intersection,  ///< if true, just report if intersecting (no further calculus on normal, depht, etc.)
        bool swap_pairs          ///< if true, pairs are reported as Triangle-Sphere (triangle will be the main ref.)
        );

    /// BOX - TRIANGLE specific case
    ///
    static int ComputeBoxTriangleCollisions(
        geometry::ChBox& mbox,             ///< box
        ChMatrix33<>* R1,                  ///< absolute rotation of 1st model (with box)
        Vector* T1,                        ///< absolute position of 1st model (with box)
        geometry::ChTriangle& mtri,        ///< triangle
        ChMatrix33<>* R2,                  ///< absolute rotation of 2nd model (with triangle)
        Vector* T2,                        ///< absolute position of 2nd model (with triangle)
        ChNarrowPhaseCollider& mcollider,  ///< reference to a collider to store contacts into
        bool just_intersection,  ///< if true, just report if intersecting (no further calculus on normal, depht, etc.)
        bool swap_pairs          ///< if true, pairs are reported as Triangle-Box (triangle will be the main ref.)
        );
};

}  // end namespace collision
}  // end namespace chrono

#endif
