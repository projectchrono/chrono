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

#ifndef CHC_AABB_H
#define CHC_AABB_H

#include <cmath>
#include <vector>

#include "chrono/geometry/ChGeometry.h"
#include "chrono/collision/edgetempest/ChCCompile.h"

namespace chrono {
namespace collision {

///
/// Class for Axis Aligned Bounding Box (AABB).
/// This class is useful for collision tests based on
/// trees of bounding volumes.
///

class CHAABB {
  public:
    // DATA

    Vector To;  ///< Position of center of aabb

    Vector d;  ///< Size, (half) dimensions of obb

    int first_child;  ///< Positive value is index of first_child AABB.
                      ///  Negative value is -(index + 1) of geometry in geom.list

    // FUNCTIONS
    CHAABB();
    ~CHAABB();
    // Copy constructor
    CHAABB(const CHAABB& other) {
        To = other.To;
        d = other.d;
        first_child = other.first_child;
    }

    /// Return 1 if this is a leaf
    inline int IsLeaf() { return first_child < 0; }

    /// Returns the index of the geometry in the geometry vector of the model
    /// Caution: must be used only if IsLeaf()==true
    inline int GetGeometryIndex() { return (-first_child - 1); }

    /// Returns the index of the first child box, in box vector of the model
    /// Caution: must be used only if IsLeaf()==false
    inline int GetFirstChildIndex() { return first_child; }

    /// Returns the index of the second child box, in box vector of the model
    /// Caution: must be used only if IsLeaf()==false
    inline int GetSecondChildIndex() { return first_child + 1; }

    /// Returns the size of the bounding box (sphere radius)
    double GetSize() { return (d.x() * d.x() + d.y() * d.y() + d.z() * d.z()); }

    /// Given  a list of geometric object, this function recomputes the
    /// bounding box in order to enclose 'ngeos' geometries,
    /// from index 'firstgeo' up to 'firstgeo+ngeos', not included.
    ///
    void FitToGeometries(std::vector<geometry::ChGeometry*> mgeos,  ///< vector of geometric objects
                         int firstgeo,                              ///< geometries will be fit from this index...
                         int ngeos,                                 ///< .. up to this index
                         double envelope                            ///< inflate all boxes by this amount
                         );

    /// Find if two box AABB are overlapping, given relative
    /// rotation matrix, relative translation, and the two AABB
    /// \return true if overlapping
    ///
    static bool AABB_Overlap(ChMatrix33<>& B,  ///< relative rotation matrix
                             Vector T,         ///< relative translation
                             CHAABB* b1,       ///< first bounding box
                             CHAABB* b2        ///< second bounding box
                             );

    /// Find if two box AABB are overlapping, given relative
    /// rotation matrix B, relative translation T, and the two AABB,
    /// Optimized for case when the absolute fabsB matrix is precomputed, since
    /// B and fabsB do not change during the many overlap tests on AABB of two models.
    /// \return true if overlapping
    ///
    static bool AABB_Overlap(ChMatrix33<>& B,      ///< relative rotation matrix
                             ChMatrix33<>& fabsB,  ///< relative rotation, but with absolute values
                             Vector T,             ///< relative translation
                             CHAABB* b1,           ///< first bounding box
                             CHAABB* b2            ///< second bounding box
                             );
};

}  // end namespace collision
}  // end namespace chrono

#endif
