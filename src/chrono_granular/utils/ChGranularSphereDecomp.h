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

#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangle.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {
namespace granular {

/// Decompose a given triangle mesh surface into a minimally-overlapping collection of spheres.
/// The mesh must be stored in OBJ format and must consist of only triangles.
/// Returns the number of spheres added as a result of the decomposition.
std::vector<ChVector<float>> MeshSphericalDecomposition(
    std::vector<std::string>& objfilenames,  ///!< Vector of OBJ mesh files paths
    std::vector<ChVector<float>>& scalings,  ///!< Vector of scaling vectors to apply to the mesh before decomposition
    std::vector<ChVector<float>>& offsets,   ///!< Displacement to apply to the mesh before decomposition
    float sphere_radius                      ///!< Radius to use for all spheres in the decomposition
) {
    std::vector<ChVector<float>> sphere_points;
    size_t n_triangles = 0;
    for (size_t i = 0; i < objfilenames.size(); i++) {
        std::string mesh_filename = objfilenames[i];
        geometry::ChTriangleMeshConnected mesh;
        mesh.LoadWavefrontMesh(mesh_filename, true, false);
        mesh.Transform(offsets[i], ChMatrix33<>(ChVector<>(scalings[i].x(), scalings[i].y(), scalings[i].z())));

        // For each triangle in the mesh, label the vertices as A,B,C
        // Fill in successive parallel lines starting at BC and advancing toward A
        int n_triangles_mesh = mesh.getNumTriangles();
        for (int i = 0; i < n_triangles_mesh; i++) {
            geometry::ChTriangle tri = mesh.getTriangle(i);
            ChVector<float> A = tri.p1;
            ChVector<float> B = tri.p2;
            ChVector<float> C = tri.p3;

            ChVector<float> BC = C - B;
            BC.Normalize();

            ChVector<float> BA = A - B;
            BA.Normalize();

            ChVector<float> CA = A - C;
            CA.Normalize();

            ChVector<float> n_tilde = (-BA).Cross(BC);  // Right hand normal to the triangle plane
            ChVector<float> n = BC.Cross(n_tilde);      // normal to the triangle edge in the plane
            n.Normalize();

            // Endpoints of current working line
            ChVector<float> start = B;
            ChVector<float> end = C;

            float end_product = n.Dot(A);  // Defines the end condition plane at A
            do {
                float L = (start - end).Length();
                float n_spheres_across = std::ceil(L / (2.0 * sphere_radius));
                float increment = L / n_spheres_across;

                for (float j = 0; j < n_spheres_across; j += 1.0) {
                    sphere_points.push_back(start + j * increment * BC);
                    n_triangles++;
                }

                // Set next line to be 2r farther and project the endpoints
                // onto the sides of the triangle
                start = start + (-2 * sphere_radius / BA.Dot(n)) * BA;
                end = end + (-2 * sphere_radius / CA.Dot(n)) * CA;
            } while (start.Dot(n) >= end_product);
        }
    }

    return sphere_points;
}
}  // namespace granular
}  // namespace chrono