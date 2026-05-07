// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distributri_idxon and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Abhinov Koutharapu
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono/geometry/ChDelaunay2D.h"

namespace chrono {

bool ChDelaunay2D::Circumcircle2D(const ChVector3d& A, const ChVector3d& B, const ChVector3d& C, double& cx, double& cy, double& r2) {
    // Calculate the determinant of the linear system for the circumcenter
    const double D = 2.0 * (A.x() * (B.y() - C.y()) + B.x() * (C.y() - A.y()) + C.x() * (A.y() - B.y()));

    if (std::abs(D) < 1e-14) {
        return false;  // points are collinear
    }

    const double A2 = A.x() * A.x() + A.y() * A.y();
    const double B2 = B.x() * B.x() + B.y() * B.y();
    const double C2 = C.x() * C.x() + C.y() * C.y();

    // Calculate circumcenter coordinates using Cramer's Rule.
    cx = (A2 * (B.y() - C.y()) + B2 * (C.y() - A.y()) + C2 * (A.y() - B.y())) / D;
    cy = (A2 * (C.x() - B.x()) + B2 * (A.x() - C.x()) + C2 * (B.x() - A.x())) / D;

    const double dx = A.x() - cx;
    const double dy = A.y() - cy;

    r2 = dx * dx + dy * dy;
    return true;
}

bool ChDelaunay2D::InCircumcircle(const ChVector3d& P, const ChVector3d& A, const ChVector3d& B, const ChVector3d& C) {
    double cx, cy, r2;

    if (!Circumcircle2D(A, B, C, cx, cy, r2))
        return false;

    const double dx = P.x() - cx;
    const double dy = P.y() - cy;
    const double d2 = dx * dx + dy * dy;

    // Use a scale-aware tolerance so behavior is less sensitive to geometry size
    const double tol = 1e-12 * std::max(1.0, std::abs(r2));
    return d2 < (r2 - tol);
}

void ChDelaunay2D::BowyerWatson(const std::vector<ChVector3d>& points, std::vector<Tri>& out) {
    out.clear();
    const int n = static_cast<int>(points.size());
    if (n < 3)
        return;

    double x_min = points[0].x(), x_max = x_min;
    double y_min = points[0].y(), y_max = y_min;
    for (const auto& p : points) {
        x_min = std::min(x_min, p.x());
        x_max = std::max(x_max, p.x());
        y_min = std::min(y_min, p.y());
        y_max = std::max(y_max, p.y());
    }

    const double delta = std::max(x_max - x_min, y_max - y_min) * 10.0;

    // Add the super triangle points.
    std::vector<ChVector3d> extended_points = points;
    extended_points.emplace_back(x_min - delta, y_min - delta, 0.0);              // Lower left
    extended_points.emplace_back(x_min + 2.0 * delta, y_min - delta, 0.0);        // Lower Right
    extended_points.emplace_back(x_min + 0.5 * delta, y_min + 2.0 * delta, 0.0);  // Above Center

    std::vector<Tri> triangles;
    triangles.reserve((2 * n) + 2);
    triangles.push_back({n, n + 1, n + 2});  // Indices of the super triangle

    std::vector<int> bad_triangles;
    std::vector<std::pair<int, int>> cavity_edges;

    for (int point = 0; point < n; point++) {
        const ChVector3d& P = extended_points[point];

        bad_triangles.clear();
        for (int tri_idx = 0; tri_idx < static_cast<int>(triangles.size()); tri_idx++) {
            const Tri& cur_tri = triangles[tri_idx];
            if (InCircumcircle(P, extended_points[cur_tri.a], extended_points[cur_tri.b], extended_points[cur_tri.c]))
                bad_triangles.push_back(tri_idx);
        }

        cavity_edges.clear();
        for (int tri_idx : bad_triangles) {
            const Tri& cur_tri = triangles[tri_idx];

            // Define the three potential boundary edges of the current bad triangle
            const int potential_edges[3][2] = {{cur_tri.a, cur_tri.b}, {cur_tri.b, cur_tri.c}, {cur_tri.c, cur_tri.a}};

            for (const auto& edge : potential_edges) {
                const int vert_a = edge[0];
                const int vert_b = edge[1];
                bool is_shared_internally = false;

                // Check this edge against all other bad triangles
                for (int other_tri_idx : bad_triangles) {
                    if (other_tri_idx == tri_idx)
                        continue;

                    const Tri& other_tri = triangles[other_tri_idx];

                    // An edge is shared if both vertices exist in another bad triangle.
                    bool has_vert_a = (other_tri.a == vert_a || other_tri.b == vert_a || other_tri.c == vert_a);
                    bool has_vert_b = (other_tri.a == vert_b || other_tri.b == vert_b || other_tri.c == vert_b);

                    if (has_vert_a && has_vert_b) {
                        is_shared_internally = true;
                        break;
                    }
                }

                // If the edge isn't shared with another bad triangle, it's the boundary of the cavity
                if (!is_shared_internally)
                    cavity_edges.emplace_back(vert_a, vert_b);
            }
        }
        // Remove bad triangles via back-swap
        std::sort(bad_triangles.rbegin(), bad_triangles.rend());
        for (int tri : bad_triangles) {
            triangles[tri] = triangles.back();
            triangles.pop_back();
        }

        // Fill cavity.
        for (const auto& edge : cavity_edges)
            triangles.push_back({point, edge.first, edge.second});
    }

    out.reserve(triangles.size());
    for (const Tri& tri : triangles)
        // If any indices are greater than or equal to n, then the must be apart of the super triangle
        if (tri.a < n && tri.b < n && tri.c < n)
            out.push_back(tri);
}

bool ChDelaunay2D::Triangulate(const std::vector<ChVector3d>& points, ChTriangleMeshConnected& mesh) {
    mesh.Clear();

    std::vector<Tri> triangles;
    BowyerWatson(points, triangles);

    // If the algorithm failed to find any triangles, exit early
    if (triangles.empty())
        return false;

    auto& verts = mesh.m_vertices;
    auto& faces = mesh.m_face_v_indices;
    auto& nrms = mesh.m_normals;
    auto& fn_idx = mesh.m_face_n_indices;

    const int n = static_cast<int>(points.size());
    verts = points;

    // Assign a uniform normal to all vertices
    const ChVector3d up(0.0, 0.0, 1.0);
    nrms.assign(n, up);

    faces.reserve(triangles.size());
    fn_idx.reserve(triangles.size());

    for (const Tri& t : triangles) {
        // Calculate cross product of edges (B-A) and (C-A)
        const ChVector3d& A = points[t.a];
        const ChVector3d& B = points[t.b];
        const ChVector3d& C = points[t.c];
        const double cross = (B.x() - A.x()) * (C.y() - A.y()) - (B.y() - A.y()) * (C.x() - A.x());

        // Ensure consistent Counter-Clockwise (CCW) winding order to face the +Z direction
        if (cross >= 0.0)
            faces.emplace_back(t.a, t.b, t.c);
        else
            faces.emplace_back(t.a, t.c, t.b);

        fn_idx.emplace_back(t.a, t.b, t.c);
    }

    return true;
}

std::shared_ptr<ChTriangleMeshConnected> ChDelaunay2D::CreateMesh(const std::vector<ChVector3d>& points) {
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    Triangulate(points, *mesh);
    return mesh;
}

}  // namespace chrono