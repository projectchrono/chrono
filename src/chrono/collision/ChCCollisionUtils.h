// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHCOLLISIONUTILS_H
#define CHCOLLISIONUTILS_H

#include "chrono/collision/bullet/LinearMath/btConvexHull.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"

namespace chrono {
namespace collision {

///
/// Class with some utility functions for collision detection,
/// as static functions.
///

class ChApi ChCollisionUtils {
  public:
    /// Calculate the line segment PaPb that is the shortest route between
    /// two lines P1P2 and P3P4. Calculate also the values of mua and mub where
    ///    Pa = P1 + mua (P2 - P1)
    ///    Pb = P3 + mub (P4 - P3)
    /// Return false if no solution exists.

    static bool
    LineLineIntersect(Vector p1, Vector p2, Vector p3, Vector p4, Vector* pa, Vector* pb, double* mua, double* mub);

    /// Calculate distance between a point p and a line identified
    /// with segment dA,dB. Returns distance. Also, the mu value reference
    /// tells if the nearest projection of point on line falls into segment (for mu 0...1)

    static double PointLineDistance(Vector p, Vector dA, Vector dB, double& mu, int& is_insegment);

    /// Calculate distance of a point from a triangle surface.
    /// Also computes if projection is inside the triangle.
    /// If is_into = true, Bprojected is also computed. 
    /// Returns distance (positive if 'out' side, out is where points A1 A2 A3 can be read in clockwise fashion)

    static double PointTriangleDistance(Vector B,
                                        Vector A1,
                                        Vector A2,
                                        Vector A3,
                                        double& mu,
                                        double& mv,
                                        int& is_into,
                                        Vector& Bprojected);
};

/// Wrapper for using andexporting the Bullet implementation of the convex hull library.
class ChApi ChConvexHullLibraryWrapper {
  public:
    ChConvexHullLibraryWrapper();

    void ComputeHull(std::vector<ChVector<> >& points, geometry::ChTriangleMeshConnected& vshape);
};

}  // end namespace collision
}  // end namespace chrono

#endif
