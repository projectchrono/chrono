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
// Author: Abhinov Koutharapu
// =============================================================================

#include <algorithm>
#include <array>
#include <ostream>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/core/ChDataPath.h"
#include "chrono/geometry/ChDelaunay2D.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

static const std::string out_dir = GetChronoTestOutputPath() + "/delaunay/";

ChWriterCSV OutStream() {
    ChWriterCSV out(",");
    out.Stream().setf(std::ios::fixed | std::ios::showpos);
    out.Stream().precision(10);
    return out;
}

// Randomly generated points from Matlab with z = 0.
static const std::vector<ChVector3d> g_points = {
    {0.4893269800, 0.0185950709, 0.0000000000},
    {0.0593324427, 0.3089695607, 0.0000000000},
    {0.3662024256, 0.6306511235, 0.0000000000},
    {0.5188654396, 0.9835823225, 0.0000000000},
    {0.5982250112, 0.9323362351, 0.0000000000},
    {0.4306144931, 0.6192957266, 0.0000000000},
    {0.1786340527, 0.2749763685, 0.0000000000},
    {0.2852538976, 0.6330462253, 0.0000000000},
    {0.0714386299, 0.1400358160, 0.0000000000},
    {0.1847133411, 0.7997399665, 0.0000000000},
    {0.0878279230, 0.6586635474, 0.0000000000},
    {0.7364685816, 0.2541621438, 0.0000000000},
    {0.1959186804, 0.1425443687, 0.0000000000},
    {0.7683410382, 0.6430552760, 0.0000000000},
    {0.6375017549, 0.2722454486, 0.0000000000},
    {0.7964059747, 0.4504680434, 0.0000000000},
    {0.7881927117, 0.4618342410, 0.0000000000},
    {0.4168771258, 0.4282745660, 0.0000000000},
    {0.7886423520, 0.8829908928, 0.0000000000},
    {0.1029748754, 0.0736881839, 0.0000000000},
    {0.9640599044, 0.3423813471, 0.0000000000},
    {0.1368641187, 0.8054113206, 0.0000000000},
    {0.4965543628, 0.8665465984, 0.0000000000},
    {0.8945791902, 0.9943540985, 0.0000000000},
    {0.7660348629, 0.2066744633, 0.0000000000}
};

// Matlab output points for 2D Deluanay triangulation for the input points above.
static const std::vector<std::array<int, 3>> g_matlab_ref_triangles = {
    {0,12,17},
    {0,12,19},
    {0,14,17},
    {0,14,24},
    {1,6,7},
    {1,6,8},
    {1,7,10},
    {2,5,17},
    {2,5,22},
    {2,7,9},
    {2,7,17},
    {2,9,22},
    {3,4,22},
    {3,4,23},
    {3,9,21},
    {3,9,22},
    {4,13,18},
    {4,13,22},
    {4,18,23},
    {5,13,16},
    {5,13,22},
    {5,16,17},
    {6,7,17},
    {6,8,12},
    {6,12,17},
    {7,9,10},
    {8,12,19},
    {9,10,21},
    {11,14,15},
    {11,14,24},
    {11,15,20},
    {11,20,24},
    {13,15,16},
    {13,15,20},
    {13,18,23},
    {13,20,23},
    {14,15,16},
    {14,16,17}
};

int main(int argc, char* argv[]) {
    if (!filesystem::create_subdirectory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ChTriangleMeshConnected mesh;
    if (!ChDelaunay2D::Triangulate(g_points, mesh)) {
        std::cout << "UNIT TEST: FAILED" << std::endl;
        std::cout << "Reason: Triangulate() returned false." << std::endl;
        return 1;
    }

    std::vector<std::array<int, 3>> chrono_mesh_triangles;
    chrono_mesh_triangles.reserve(mesh.m_face_v_indices.size());
    for (const auto& face : mesh.m_face_v_indices) {
        std::array<int, 3> tri = {face.x(), face.y(), face.z()};
        std::sort(tri.begin(), tri.end()); // Sort the indices within a triangle.
        chrono_mesh_triangles.push_back(tri);
    }

    std::sort(chrono_mesh_triangles.begin(), chrono_mesh_triangles.end());

    std::vector<std::array<int, 3>> matlab_ref_triangles = g_matlab_ref_triangles;
    std::sort(matlab_ref_triangles.begin(), matlab_ref_triangles.end());

    bool passed = true;
    if (chrono_mesh_triangles.size() != matlab_ref_triangles.size()) {
        std::cout << "Reason: triangle count mismatch (computed=" << chrono_mesh_triangles.size() << ", reference=" << matlab_ref_triangles.size()
                  << ")." << std::endl;
        passed = false;
    }

    const size_t common = std::min(chrono_mesh_triangles.size(), matlab_ref_triangles.size());
    for (size_t i = 0; i < common; ++i) {
        if (chrono_mesh_triangles[i] != matlab_ref_triangles[i]) {
            std::cout << "First mismatch at index " << i << ": computed=[" << chrono_mesh_triangles[i][0] << "," << chrono_mesh_triangles[i][1] << ","
                      << chrono_mesh_triangles[i][2] << "], reference=[" << matlab_ref_triangles[i][0] << "," << matlab_ref_triangles[i][1] << ","
                      << matlab_ref_triangles[i][2] << "]." << std::endl;
            passed = false;
            break;
        }
    }

    // Create CSV files to output directory. 
    {
        ChWriterCSV out = OutStream();
        out << "x" << "y" << "z" << std::endl;
        for (const auto& p : g_points)
            out << p.x() << p.y() << p.z() << std::endl;
        out.WriteToFile(out_dir + "Delaunay25_CHRONO_Input.csv", "# Input points (x,y,z)");
    }

    {
        ChWriterCSV out = OutStream();
        out << "v0" << "v1" << "v2" << std::endl;
        for (const auto& t : matlab_ref_triangles)
            out << t[0] << t[1] << t[2] << std::endl;
        out.WriteToFile(out_dir + "Delaunay25_MATLAB_Reference.csv", "# MATLAB reference triangles (normalized)");
    }

    {
        ChWriterCSV out = OutStream();
        out << "v0" << "v1" << "v2" << std::endl;
        for (const auto& t : chrono_mesh_triangles)
            out << t[0] << t[1] << t[2] << std::endl;
        out.WriteToFile(out_dir + "Delaunay25_CHRONO_Output.csv", "# Chrono output triangles (normalized)");
    }

    std::cout << "Wrote CSV files to: " << out_dir << std::endl;
    std::cout << "UNIT TEST: " << (passed ? "PASSED" : "FAILED") << std::endl;
    return passed ? 0 : 1;
}