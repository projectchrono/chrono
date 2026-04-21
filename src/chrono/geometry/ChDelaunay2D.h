// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Abhinov Koutharapu
// =============================================================================

#pragma once

#include <vector>
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/core/ChVector3.h"


namespace chrono {

class ChApi ChDelaunay2D {
  public:
    struct Tri {
        int a, b, c;
    };

    /// Triangulate a flat (z == 0) point set using Bowyer-Watson.
    static bool Triangulate(const std::vector<ChVector3d>& points, ChTriangleMeshConnected& mesh);

    /// Creates a Triangle mesh.
    static std::shared_ptr<ChTriangleMeshConnected> CreateMesh(const std::vector<ChVector3d>& points);

    /// Run Bowyer-Watson and return raw triangle.
    static void BowyerWatson(const std::vector<ChVector3d>& points, std::vector<Tri>& out);

  private:
    /// Circumcircle of triangle (A, B, C) in the XY plane.
    static bool Circumcircle2D(const ChVector3d& A, const ChVector3d& B, const ChVector3d& C, double& cx, double& cy, double& r2);

    /// True if P lies inside the circumcircle of (A, B, C).
    static bool InCircumcircle(const ChVector3d& P, const ChVector3d& A, const ChVector3d& B, const ChVector3d& C);
};

}  // namespace chrono