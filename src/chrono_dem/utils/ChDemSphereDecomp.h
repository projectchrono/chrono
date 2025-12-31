// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#pragma once

#include <string>
#include <vector>
#include <map>

#include "chrono/core/ChVector3.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {
namespace dem {

/// @addtogroup dem_utils
/// @{

/// Decompose a given triangle mesh surface into a minimally-overlapping collection of spheres.
/// The mesh must be stored in OBJ format and must consist of only triangles.
/// Returns the number of spheres added as a result of the decomposition.
template <typename Real>
std::vector<ChVector3<Real>> MeshSphericalDecomposition(
    std::string objfilename,  ///< OBJ mesh file path
    ChVector3<Real> scaling,  ///< Scaling vector to apply to the mesh before decomposition
    ChVector3<Real> offset,   ///< Displacement to apply to the mesh before decomposition
    Real sphere_radius        ///< Radius to use for all spheres in the decomposition
) {
    std::vector<ChVector3<Real>> sphere_points;
    std::string mesh_filename = objfilename;
    ChTriangleMeshConnected mesh;
    mesh.LoadWavefrontMesh(mesh_filename, true, false);
    mesh.Transform(offset, ChMatrix33<>(ChVector3d(scaling.x(), scaling.y(), scaling.z())));

    // For each triangle in the mesh, label the vertices as A,B,C
    // Fill in successive parallel lines starting at BC and advancing toward A
    unsigned int n_triangles_mesh = mesh.GetNumTriangles();
    for (unsigned int i = 0; i < n_triangles_mesh; i++) {
        ChTriangle tri = mesh.GetTriangle(i);
        ChVector3<Real> A = tri.p1;
        ChVector3<Real> B = tri.p2;
        ChVector3<Real> C = tri.p3;

        // Order so that A has the smallest angle
        ChVector3<Real> ab = B - A;
        ChVector3<Real> ac = C - A;
        ChVector3<Real> bc = C - B;
        Real theta_A = std::abs(std::acos(ab.Dot(ac) / (ab.Length() * ac.Length())));
        Real theta_B = std::abs(std::acos(bc.Dot(-ab) / (bc.Length() * ab.Length())));
        Real theta_C = (Real)CH_PI - (theta_A + theta_B);

        int small_i = (theta_A <= theta_C) ? ((theta_A <= theta_B) ? 0 : 1) : ((theta_C <= theta_B) ? 2 : 1);

        std::map<int, ChVector3f> i_to_vertex;
        i_to_vertex[0] = A;
        i_to_vertex[1] = B;
        i_to_vertex[2] = C;

        A = i_to_vertex[small_i];
        B = i_to_vertex[(small_i + 1) % 3];
        C = i_to_vertex[(small_i + 2) % 3];

        ChVector3<Real> BC = C - B;
        BC.Normalize();

        ChVector3<Real> BA = A - B;
        BA.Normalize();

        ChVector3<Real> CA = A - C;
        CA.Normalize();

        ChVector3<Real> n_tilde = (-BA).Cross(BC);  // Right hand normal to the triangle plane
        ChVector3<Real> n = BC.Cross(n_tilde);      // normal to the triangle edge in the plane
        n.Normalize();

        const ChVector3<Real> B_star = B + (BC + BA) * sphere_radius;
        const ChVector3<Real> C_star = C + (CA - BC) * sphere_radius;

        // Endpoints of current working line
        ChVector3<Real> start = B_star;
        ChVector3<Real> end = C_star;

        // Fill parallel lines until the width is too small
        while ((start - end).Length() > 1.5 * sphere_radius) {
            Real L = (start - end).Length();
            int n_spheres_across = (int)std::ceil(L / (2.0 * sphere_radius));
            Real increment = L / n_spheres_across;

            for (Real j = 0; j <= n_spheres_across; j += 1.0) {
                sphere_points.push_back(start + j * increment * BC);
            }

            // Set next line to be 2r farther and project the endpoints
            // onto the sides of the triangle
            start = start + (-2 * sphere_radius / BA.Dot(n)) * BA;
            end = end + (-2 * sphere_radius / CA.Dot(n)) * CA;
        }

        // Fill remaining space as a single line to the last vertex
        ChVector3<Real> A_star = (start + end) * 0.5;
        ChVector3<Real> A_starA = (A - A_star);
        A_starA.Normalize();

        start = A_star;
        Real end_product = n.Dot(A);  // Defines the end condition plane at A
        while (start.Dot(n) > end_product) {
            sphere_points.push_back(start);
            start = start + A_starA * 2.0 * sphere_radius;
        }
    }

    return sphere_points;
}

/// @} dem_utils

}  // namespace dem
}  // namespace chrono