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
// Authors: Radu Serban
// =============================================================================
//
// Simple 2D convex hull class
//
// =============================================================================

#ifndef CH_CONVEX_HULL_H
#define CH_CONVEX_HULL_H

#include <vector>
#include <unordered_map>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector2.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Simple 2D convex hull class.
class ChApi ChConvexHull2D {
  public:
    enum Method {
        JARVIS,  ///< Jarvis algorithm (wrapping)
        GRAHAM   ///< Graham scan algorithm
    };

    /// Construct the convex hull of the specified points, using the given method.
    ChConvexHull2D(std::vector<ChVector2<>>& points, Method method = JARVIS);

    ~ChConvexHull2D() {}

    /// Return the points in the convex hull.
    const std::vector<ChVector2<>>& GetHull() const { return m_hull; }

    /// Return the perimeter of the convex hull.
    double GetPerimeter() const { return m_perimeter; }

    /// Return area enclosed by convex hull.
    double GetArea() const { return m_area; }

  private:
    /// Convex hull by wrapping (Jarvis algorithm).
    /// The current implementation returns all points on the convex hull, including any points
    /// that fall on the edges of the convex hull.  However, only vertices of the convex hull
    /// are guaranteed to be in order (as moving counter clock wise from bottom-left point).
    void ComputeJarvis(const std::vector<ChVector2<>>& points, size_t n);

    /// Convex hull by Graham scan.
    /// Note that this algorithm will modify (permute) the input points.
    void ComputeGraham(std::vector<ChVector2<>>& points, size_t n);

    std::vector<ChVector2<>> m_hull;  ///< points in convex hull
    double m_perimeter;               ///< perimeter of convex hull
    double m_area;                    ///< area enclosed by convex hull
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
