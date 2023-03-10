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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
// =============================================================================

//// RADU TODO
//// (1) Merge() does nothing for the face material indices.
////     The problem is that we cannot merge the visual material lists (not present here)
//// (2) SplitEdge() does not do anything for face material indices.
///      This could be implemented such that the two new faces point to the same material.

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <map>

#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTriangleMeshConnected)

// -----------------------------------------------------------------------------

ChTriangleMeshConnected::ChTriangleMeshConnected(const ChTriangleMeshConnected& source) {
    m_vertices = source.m_vertices;
    m_normals = source.m_normals;
    m_UV = source.m_UV;
    m_colors = source.m_colors;

    m_face_v_indices = source.m_face_v_indices;
    m_face_n_indices = source.m_face_n_indices;
    m_face_uv_indices = source.m_face_uv_indices;
    m_face_col_indices = source.m_face_col_indices;
    m_face_mat_indices = source.m_face_mat_indices;
    
    // deep copies of properties
    this->m_properties_per_vertex.resize(source.m_properties_per_vertex.size());
    for (size_t i = 0; i < source.m_properties_per_vertex.size(); ++i)
        this->m_properties_per_vertex[i]=source.m_properties_per_vertex[i]->clone();
    // deep copies of properties
    this->m_properties_per_face.resize(source.m_properties_per_face.size());
    for (size_t i = 0; i < source.m_properties_per_face.size(); ++i)
        this->m_properties_per_face[i]=source.m_properties_per_face[i]->clone();

    m_filename = source.m_filename;
}

ChTriangleMeshConnected::~ChTriangleMeshConnected() {
    for (ChProperty* id : this->m_properties_per_vertex)
        delete(id);
    for (ChProperty* id : this->m_properties_per_face)
        delete(id);
}


void ChTriangleMeshConnected::addTriangle(const ChVector<>& vertex0,
                                          const ChVector<>& vertex1,
                                          const ChVector<>& vertex2) {
    int base_v = (int)m_vertices.size();
    m_vertices.push_back(vertex0);
    m_vertices.push_back(vertex1);
    m_vertices.push_back(vertex2);
    m_face_v_indices.push_back(ChVector<int>(base_v, base_v + 1, base_v + 2));
}

void ChTriangleMeshConnected::addTriangle(const ChTriangle& atriangle) {
    int base_v = (int)m_vertices.size();
    m_vertices.push_back(atriangle.p1);
    m_vertices.push_back(atriangle.p2);
    m_vertices.push_back(atriangle.p3);
    m_face_v_indices.push_back(ChVector<int>(base_v, base_v + 1, base_v + 2));
}

void ChTriangleMeshConnected::Clear() {
    m_vertices.clear();
    m_normals.clear();
    m_UV.clear();
    m_colors.clear();
    m_face_v_indices.clear();
    m_face_n_indices.clear();
    m_face_uv_indices.clear();
    m_face_col_indices.clear();
    m_face_mat_indices.clear();

    for (ChProperty* id : this->m_properties_per_vertex)
        delete(id);
    m_properties_per_vertex.clear();

    for (ChProperty* id : this->m_properties_per_face)
        delete(id);
    m_properties_per_vertex.clear();
}

void ChTriangleMeshConnected::GetBoundingBox(ChVector<>& cmin, ChVector<>& cmax, const ChMatrix33<>& rot) const {
    cmin = ChVector<>(+std::numeric_limits<double>::max());
    cmax = ChVector<>(-std::numeric_limits<double>::max());

    for (const auto& v : m_vertices) {
        auto p = rot.transpose() * v;

        cmin.x() = ChMin(cmin.x(), p.x());
        cmin.y() = ChMin(cmin.y(), p.y());
        cmin.z() = ChMin(cmin.z(), p.z());

        cmax.x() = ChMax(cmax.x(), p.x());
        cmax.y() = ChMax(cmax.y(), p.y());
        cmax.z() = ChMax(cmax.z(), p.z());
    }
}


// Following function is a modified version of:
//
// Geometric Tools, LLC
// Copyright (c) 1998-2014
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)
//
// void ComputeMassProperties (const Vector3<Real>* vertices, int numTriangles,
//                             const int* indices, bool bodyCoords, Real& mass,
//                             Vector3<Real>& center, Matrix3<Real>& inertia)
//
void ChTriangleMeshConnected::ComputeMassProperties(bool bodyCoords,
                                                    double& mass,
                                                    ChVector<>& center,
                                                    ChMatrix33<>& inertia) {
    const double oneDiv6 = (double)(1.0 / 6.0);
    const double oneDiv24 = (double)(1.0 / 24.0);
    const double oneDiv60 = (double)(1.0 / 60.0);
    const double oneDiv120 = (double)(1.0 / 120.0);

    // order:  1, x, y, z, x^2, y^2, z^2, xy, yz, zx
    double integral[10] = {(double)0.0, (double)0.0, (double)0.0, (double)0.0, (double)0.0,
                           (double)0.0, (double)0.0, (double)0.0, (double)0.0, (double)0.0};

    for (int i = 0; i < this->getNumTriangles(); i++) {
        // Get vertices of triangle i.
        ChVector<double> v0 = this->m_vertices[m_face_v_indices[i].x()];
        ChVector<double> v1 = this->m_vertices[m_face_v_indices[i].y()];
        ChVector<double> v2 = this->m_vertices[m_face_v_indices[i].z()];

        // Get cross product of edges and normal vector.
        ChVector<double> V1mV0 = v1 - v0;
        ChVector<double> V2mV0 = v2 - v0;
        ChVector<double> N = Vcross(V1mV0, V2mV0);

        // Compute integral terms.
        double tmp0, tmp1, tmp2;
        double f1x, f2x, f3x, g0x, g1x, g2x;
        tmp0 = v0.x() + v1.x();
        f1x = tmp0 + v2.x();
        tmp1 = v0.x() * v0.x();
        tmp2 = tmp1 + v1.x() * tmp0;
        f2x = tmp2 + v2.x() * f1x;
        f3x = v0.x() * tmp1 + v1.x() * tmp2 + v2.x() * f2x;
        g0x = f2x + v0.x() * (f1x + v0.x());
        g1x = f2x + v1.x() * (f1x + v1.x());
        g2x = f2x + v2.x() * (f1x + v2.x());

        double f1y, f2y, f3y, g0y, g1y, g2y;
        tmp0 = v0.y() + v1.y();
        f1y = tmp0 + v2.y();
        tmp1 = v0.y() * v0.y();
        tmp2 = tmp1 + v1.y() * tmp0;
        f2y = tmp2 + v2.y() * f1y;
        f3y = v0.y() * tmp1 + v1.y() * tmp2 + v2.y() * f2y;
        g0y = f2y + v0.y() * (f1y + v0.y());
        g1y = f2y + v1.y() * (f1y + v1.y());
        g2y = f2y + v2.y() * (f1y + v2.y());

        double f1z, f2z, f3z, g0z, g1z, g2z;
        tmp0 = v0.z() + v1.z();
        f1z = tmp0 + v2.z();
        tmp1 = v0.z() * v0.z();
        tmp2 = tmp1 + v1.z() * tmp0;
        f2z = tmp2 + v2.z() * f1z;
        f3z = v0.z() * tmp1 + v1.z() * tmp2 + v2.z() * f2z;
        g0z = f2z + v0.z() * (f1z + v0.z());
        g1z = f2z + v1.z() * (f1z + v1.z());
        g2z = f2z + v2.z() * (f1z + v2.z());

        // Update integrals.
        integral[0] += N.x() * f1x;
        integral[1] += N.x() * f2x;
        integral[2] += N.y() * f2y;
        integral[3] += N.z() * f2z;
        integral[4] += N.x() * f3x;
        integral[5] += N.y() * f3y;
        integral[6] += N.z() * f3z;
        integral[7] += N.x() * (v0.y() * g0x + v1.y() * g1x + v2.y() * g2x);
        integral[8] += N.y() * (v0.z() * g0y + v1.z() * g1y + v2.z() * g2y);
        integral[9] += N.z() * (v0.x() * g0z + v1.x() * g1z + v2.x() * g2z);
    }

    integral[0] *= oneDiv6;
    integral[1] *= oneDiv24;
    integral[2] *= oneDiv24;
    integral[3] *= oneDiv24;
    integral[4] *= oneDiv60;
    integral[5] *= oneDiv60;
    integral[6] *= oneDiv60;
    integral[7] *= oneDiv120;
    integral[8] *= oneDiv120;
    integral[9] *= oneDiv120;

    // mass
    mass = integral[0];

    // center of mass
    center = ChVector<double>(integral[1], integral[2], integral[3]) / mass;

    // inertia relative to world origin
    inertia(0, 0) = integral[5] + integral[6];
    inertia(0, 1) = -integral[7];
    inertia(0, 2) = -integral[9];
    inertia(1, 0) = inertia(0, 1);
    inertia(1, 1) = integral[4] + integral[6];
    inertia(1, 2) = -integral[8];
    inertia(2, 0) = inertia(0, 2);
    inertia(2, 1) = inertia(1, 2);
    inertia(2, 2) = integral[4] + integral[5];

    // inertia relative to center of mass
    if (bodyCoords) {
        inertia(0, 0) -= mass * (center.y() * center.y() + center.z() * center.z());
        inertia(0, 1) += mass * center.x() * center.y();
        inertia(0, 2) += mass * center.z() * center.x();
        inertia(1, 0) = inertia(0, 1);
        inertia(1, 1) -= mass * (center.z() * center.z() + center.x() * center.x());
        inertia(1, 2) += mass * center.y() * center.z();
        inertia(2, 0) = inertia(0, 2);
        inertia(2, 1) = inertia(1, 2);
        inertia(2, 2) -= mass * (center.x() * center.x() + center.y() * center.y());
    }
}

std::shared_ptr<ChTriangleMeshConnected> ChTriangleMeshConnected::CreateFromWavefrontFile(const std::string& filename,
                                                                                          bool load_normals,
                                                                                          bool load_uv) {
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    if (!trimesh->LoadWavefrontMesh(filename, load_normals, load_uv))
        return nullptr;
    return trimesh;
}

bool ChTriangleMeshConnected::LoadWavefrontMesh(const std::string& filename, bool load_normals, bool load_uv) {
    assert(filesystem::path(filename).is_file());

    std::vector<tinyobj::shape_t> shapes;
    tinyobj::attrib_t att;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    bool success = tinyobj::LoadObj(&att, &shapes, &materials, &warn, &err, filename.c_str());
    if (!success) {
        std::cerr << "Error loading OBJ file " << filename << std::endl;
        std::cerr << "   tiny_obj warning message: " << warn << std::endl;
        std::cerr << "   tiny_obj error message:   " << err << std::endl;
        return false;
    }

    this->Clear();

    m_filename = filename;

    for (size_t i = 0; i < att.vertices.size() / 3; i++) {
        m_vertices.push_back(ChVector<>(att.vertices[3 * i + 0], att.vertices[3 * i + 1], att.vertices[3 * i + 2]));
    }
    if (load_normals) {
        for (size_t i = 0; i < att.normals.size() / 3; i++) {
            m_normals.push_back(ChVector<>(att.normals[3 * i + 0], att.normals[3 * i + 1], att.normals[3 * i + 2]));
        }
    }
    if (load_uv) {
        for (size_t i = 0; i < att.texcoords.size() / 2; i++) {
            m_UV.push_back(ChVector2<>(att.texcoords[2 * i + 0], att.texcoords[2 * i + 1]));
        }
    }

    for (size_t i = 0; i < shapes.size(); i++) {
        for (size_t j = 0; j < shapes[i].mesh.indices.size() / 3; j++) {
            m_face_v_indices.push_back(ChVector<int>(shapes[i].mesh.indices[3 * j + 0].vertex_index,
                                                     shapes[i].mesh.indices[3 * j + 1].vertex_index,
                                                     shapes[i].mesh.indices[3 * j + 2].vertex_index));
            if (m_normals.size() > 0) {
                m_face_n_indices.push_back(ChVector<int>(shapes[i].mesh.indices[3 * j + 0].normal_index,
                                                         shapes[i].mesh.indices[3 * j + 1].normal_index,
                                                         shapes[i].mesh.indices[3 * j + 2].normal_index));
            }
            if (m_UV.size() > 0) {
                m_face_uv_indices.push_back(ChVector<int>(shapes[i].mesh.indices[3 * j + 0].texcoord_index,
                                                          shapes[i].mesh.indices[3 * j + 1].texcoord_index,
                                                          shapes[i].mesh.indices[3 * j + 2].texcoord_index));
            }
        }
    }

    return true;
}

// Write the specified meshes in a Wavefront .obj file
void ChTriangleMeshConnected::WriteWavefront(const std::string& filename,
                                             const std::vector<ChTriangleMeshConnected>& meshes) {
    std::ofstream mf(filename);

    //// TODO: include normal information if available

    /*
        // Create a separate object for each mesh
        int i = 0;
        for (auto& m : meshes) {
            mf << "o mesh_" << std::to_string(i++) << std::endl;

            for (auto& v : m.m_vertices) {
                mf << "v " << v.x() << " " << v.y() << " " << v.z() << std::endl;
            }

            for (auto& f : m.m_face_v_indices) {
                mf << "f " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            }
        }
    */

    // Create a single object mesh
    std::vector<int> v_offsets;
    int v_off = 1;
    for (auto& m : meshes) {
        for (auto& v : m.m_vertices) {
            mf << "v " << v.x() << " " << v.y() << " " << v.z() << std::endl;
        }
        v_offsets.push_back(v_off);
        v_off += static_cast<int>(m.m_vertices.size());
    }

    std::vector<bool> has_normals;
    std::vector<int> vn_offsets;
    int vn_off = 1;
    for (auto& m : meshes) {
        has_normals.push_back(m.m_normals.size() > 0);
        for (auto& v : m.m_normals) {
            mf << "vn " << v.x() << " " << v.y() << " " << v.z() << std::endl;
        }
        vn_offsets.push_back(vn_off);
        vn_off += static_cast<int>(m.m_normals.size());
    }

    for (size_t i = 0; i < meshes.size(); i++) {
        v_off = v_offsets[i];
        if (has_normals[i]) {
            auto& idxV = meshes[i].m_face_v_indices;
            auto& idxN = meshes[i].m_face_n_indices;
            assert(idxV.size() == idxN.size());
            vn_off = vn_offsets[i];
            for (int j = 0; j < idxV.size(); j++) {
                mf << "f " <<                                                      //
                    idxV[j].x() + v_off << "//" << idxN[j].x() + vn_off << " " <<  //
                    idxV[j].y() + v_off << "//" << idxN[j].y() + vn_off << " " <<  //
                    idxV[j].z() + v_off << "//" << idxN[j].z() + vn_off <<         //
                    std::endl;
            }
        } else {
            for (auto& f : meshes[i].m_face_v_indices) {
                mf << "f " << f.x() + v_off << " " << f.y() + v_off << " " << f.z() + v_off << std::endl;
            }
        }
    }

    mf.close();
}

/// Utility function for merging multiple meshes.
ChTriangleMeshConnected ChTriangleMeshConnected::Merge(std::vector<ChTriangleMeshConnected>& meshes) {
    ChTriangleMeshConnected trimesh;
    auto& vertices = trimesh.m_vertices;
    auto& normals = trimesh.m_normals;
    auto& uvs = trimesh.m_UV;
    auto& colors = trimesh.m_colors;
    auto& idx_vertices = trimesh.m_face_v_indices;
    auto& idx_normals = trimesh.m_face_n_indices;
    auto& idx_uvs = trimesh.m_face_uv_indices;
    auto& idx_colors = trimesh.m_face_col_indices;

    int v_off = 0;
    int n_off = 0;
    int uv_off = 0;
    int c_off = 0;
    for (auto& m : meshes) {
        {
            vertices.insert(vertices.end(), m.m_vertices.begin(), m.m_vertices.end());
            std::vector<ChVector<int>> tmp;
            tmp.reserve(m.m_face_v_indices.size());
            std::transform(m.m_face_v_indices.begin(), m.m_face_v_indices.end(), std::back_inserter(tmp),
                           [&v_off](ChVector<int>& a) { return a + v_off; });
            idx_vertices.insert(idx_vertices.end(), tmp.begin(), tmp.end());
            v_off += static_cast<int>(m.m_vertices.size());
        }

        {
            normals.insert(normals.end(), m.m_normals.begin(), m.m_normals.end());
            std::vector<ChVector<int>> tmp;
            tmp.reserve(m.m_face_n_indices.size());
            std::transform(m.m_face_n_indices.begin(), m.m_face_n_indices.end(), std::back_inserter(tmp),
                           [&n_off](ChVector<int>& a) { return a + n_off; });
            idx_normals.insert(idx_normals.end(), tmp.begin(), tmp.end());
            n_off += static_cast<int>(m.m_normals.size());
        }

        {
            uvs.insert(uvs.end(), m.m_UV.begin(), m.m_UV.end());
            std::vector<ChVector<int>> tmp;
            tmp.reserve(m.m_face_uv_indices.size());
            std::transform(m.m_face_uv_indices.begin(), m.m_face_uv_indices.end(), std::back_inserter(tmp),
                           [&uv_off](ChVector<int>& a) { return a + uv_off; });
            idx_uvs.insert(idx_uvs.end(), tmp.begin(), tmp.end());
            uv_off += static_cast<int>(m.m_UV.size());
        }

        {
            colors.insert(colors.end(), m.m_colors.begin(), m.m_colors.end());
            std::vector<ChVector<int>> tmp;
            tmp.reserve(m.m_face_col_indices.size());
            std::transform(m.m_face_col_indices.begin(), m.m_face_col_indices.end(), std::back_inserter(tmp),
                           [&c_off](ChVector<int>& a) { return a + c_off; });
            idx_colors.insert(idx_colors.end(), tmp.begin(), tmp.end());
            c_off += static_cast<int>(m.m_colors.size());
        }
    }

    return trimesh;
}

void ChTriangleMeshConnected::Transform(const ChVector<> displ, const ChMatrix33<> rotscale) {
    for (int i = 0; i < m_vertices.size(); ++i) {
        m_vertices[i] = rotscale * m_vertices[i];
        m_vertices[i] += displ;
    }
    for (int i = 0; i < m_normals.size(); ++i) {
        m_normals[i] = rotscale * m_normals[i];
        m_normals[i].Normalize();
    }
}

bool ChTriangleMeshConnected::ComputeNeighbouringTriangleMap(std::vector<std::array<int, 4>>& tri_map) const {
    bool pathological_edges = false;

    std::multimap<std::pair<int, int>, int> edge_map;

    for (int it = 0; it < this->m_face_v_indices.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<int, int> medgeA(this->m_face_v_indices[it].x(), this->m_face_v_indices[it].y());
        std::pair<int, int> medgeB(this->m_face_v_indices[it].y(), this->m_face_v_indices[it].z());
        std::pair<int, int> medgeC(this->m_face_v_indices[it].z(), this->m_face_v_indices[it].x());
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<int, int>(medgeC.second, medgeC.first);
        edge_map.insert({medgeA, it});
        edge_map.insert({medgeB, it});
        edge_map.insert({medgeC, it});
    }

    // Create a map of neighboring triangles, vector of:
    // [Ti TieA TieB TieC]
    tri_map.resize(this->m_face_v_indices.size());
    for (int it = 0; it < this->m_face_v_indices.size(); ++it) {
        tri_map[it][0] = it;
        tri_map[it][1] = -1;  // default no neighbour
        tri_map[it][2] = -1;  // default no neighbour
        tri_map[it][3] = -1;  // default no neighbour
        // edges = pairs of vertexes indexes
        std::pair<int, int> medgeA(this->m_face_v_indices[it].x(), this->m_face_v_indices[it].y());
        std::pair<int, int> medgeB(this->m_face_v_indices[it].y(), this->m_face_v_indices[it].z());
        std::pair<int, int> medgeC(this->m_face_v_indices[it].z(), this->m_face_v_indices[it].x());
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<int, int>(medgeC.second, medgeC.first);
        if (edge_map.count(medgeA) > 2 || edge_map.count(medgeB) > 2 || edge_map.count(medgeC) > 2) {
            pathological_edges = true;
            // GetLog() << "Warning, edge shared with more than two triangles! \n";
        }
        auto retA = edge_map.equal_range(medgeA);
        for (auto fedge = retA.first; fedge != retA.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][1] = fedge->second;
                break;
            }
        }
        auto retB = edge_map.equal_range(medgeB);
        for (auto fedge = retB.first; fedge != retB.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][2] = fedge->second;
                break;
            }
        }
        auto retC = edge_map.equal_range(medgeC);
        for (auto fedge = retC.first; fedge != retC.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][3] = fedge->second;
                break;
            }
        }
    }
    return pathological_edges;
}

bool ChTriangleMeshConnected::ComputeWingedEdges(std::map<std::pair<int, int>, std::pair<int, int>>& winged_edges,
                                                 bool allow_single_wing) const {
    bool pathological_edges = false;

    std::multimap<std::pair<int, int>, int> edge_map;

    for (int it = 0; it < this->m_face_v_indices.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<int, int> medgeA(this->m_face_v_indices[it].x(), this->m_face_v_indices[it].y());
        std::pair<int, int> medgeB(this->m_face_v_indices[it].y(), this->m_face_v_indices[it].z());
        std::pair<int, int> medgeC(this->m_face_v_indices[it].z(), this->m_face_v_indices[it].x());
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<int, int>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<int, int>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<int, int>(medgeC.second, medgeC.first);
        edge_map.insert({medgeA, it});
        edge_map.insert({medgeB, it});
        edge_map.insert({medgeC, it});
    }

    for (auto aedge = edge_map.begin(); aedge != edge_map.end(); ++aedge) {
        auto ret = edge_map.equal_range(aedge->first);
        int nt = 0;
        std::pair<int, int> wingedge;
        std::pair<int, int> wingtri;
        wingtri.first = -1;
        wingtri.second = -1;
        for (auto fedge = ret.first; fedge != ret.second; ++fedge) {
            if (fedge->second == -1)
                break;
            wingedge.first = fedge->first.first;
            wingedge.second = fedge->first.second;
            if (nt == 0)
                wingtri.first = fedge->second;
            if (nt == 1)
                wingtri.second = fedge->second;
            ++nt;
            if (nt == 2)
                break;
        }
        if ((nt == 2) || ((nt == 1) && allow_single_wing)) {
            winged_edges.insert(
                std::pair<std::pair<int, int>, std::pair<int, int>>(wingedge, wingtri));  // ok found winged edge!
            aedge->second = -1;  // deactivate this way otherwise found again by sister
        }
        if (nt == 3) {
            pathological_edges = true;
            // GetLog() << "Warning: winged edge between "<< wing[0] << " and " << wing[1]  << " shared with more than
            // two triangles.\n";
        }
    }
    return pathological_edges;
}

int ChTriangleMeshConnected::RepairDuplicateVertexes(const double tolerance) {
    int nmerged = 0;
    std::vector<ChVector<>> processed_verts;
    std::vector<int> new_indexes(m_vertices.size());

    // merge vertexes
    for (int i = 0; i < m_vertices.size(); ++i) {
        bool tomerge = false;
        for (int j = 0; j < processed_verts.size(); ++j) {
            if ((m_vertices[i] - processed_verts[j]).Length2() < tolerance) {
                tomerge = true;
                ++nmerged;
                new_indexes[i] = j;
                break;
            }
        }
        if (!tomerge) {
            processed_verts.push_back(m_vertices[i]);
            new_indexes[i] = (int)processed_verts.size() - 1;
        }
    }

    m_vertices = processed_verts;

    // Update the merged vertexes also in face indexes to vertexes
    // Note: we DO NOT update the normal, color, UV, or material indices!
    for (int i = 0; i < this->m_face_v_indices.size(); ++i) {
        m_face_v_indices[i].x() = new_indexes[m_face_v_indices[i].x()];
        m_face_v_indices[i].y() = new_indexes[m_face_v_indices[i].y()];
        m_face_v_indices[i].z() = new_indexes[m_face_v_indices[i].z()];
    }

    return nmerged;
}

// Offset algorithm based on:
// " A 3D surface offset method for STL-format models"
//   Xiuzhi Qu and Brent Stucker

bool ChTriangleMeshConnected::MakeOffset(const double moffset) {
    std::map<int, std::vector<int>> map_vertex_triangles;
    std::vector<ChVector<>> voffsets(this->m_vertices.size());

    // build the topological info for triangles connected to vertex
    for (int i = 0; i < this->m_face_v_indices.size(); ++i) {
        map_vertex_triangles[m_face_v_indices[i].x()].push_back(i);
        map_vertex_triangles[m_face_v_indices[i].y()].push_back(i);
        map_vertex_triangles[m_face_v_indices[i].z()].push_back(i);
    }

    // scan through vertexes and offset them
    for (int i = 0; i < this->m_vertices.size(); ++i) {
        auto mpair = map_vertex_triangles.find(i);
        if (mpair != map_vertex_triangles.end()) {
            std::vector<int>& mverttriangles = mpair->second;
            int ntri = (int)mverttriangles.size();
            ChMatrixDynamic<> A(ntri, ntri);
            ChVectorDynamic<> b(ntri);
            ChVectorDynamic<> x(ntri);
            for (int j = 0; j < ntri; ++j) {
                b(j, 0) = 1;
                for (int k = 0; k < ntri; ++k) {
                    A(j, k) = Vdot(this->getTriangle(mverttriangles[j]).GetNormal(),
                                   this->getTriangle(mverttriangles[k]).GetNormal());
                }
            }

            // If any 2 face normals are (close to) colinear, A will be singular!
            // In such a case, one of the two faces should be discarded.  We can achieve this by setting
            // the corresponding row and column to 0 (except the diagonal entry which stays at 1) and
            // setting the corresponding right hand side to 0.
            // Traverse the lower triangle of A and look for entries close to +1 or -1.
            for (int j = 0; j < ntri; ++j) {
                for (int k = j + 1; k < ntri; ++k) {
                    if (std::abs(A(j, k) - 1) < 1e-4) {
                        // eliminate weight k
                        A.col(k).setZero();
                        A.row(k).setZero();
                        A(k, k) = 1;
                        b(k) = 0;
                    }
                }
            }

            x = A.colPivHouseholderQr().solve(b);

            // weighted sum as offset vector
            voffsets[i] = VNULL;
            for (int j = 0; j < ntri; ++j) {
                voffsets[i] += this->getTriangle(mverttriangles[j]).GetNormal() * x(j);
            }
        }
    }

    // apply offset vectors to itself:
    for (int i = 0; i < this->m_vertices.size(); ++i) {
        m_vertices[i] += voffsets[i] * moffset;
    }

    return true;
}

// Return the indexes of the two vertexes of the specified triangle edge.
// If unique = true, swap the pair so that 1st < 2nd, to permit test sharing with other triangle.
std::pair<int, int> ChTriangleMeshConnected::GetTriangleEdgeIndexes(
    const ChVector<int>& face_indices,  // indices of a triangular face
    int nedge,                          // number of edge: 0, 1, 2
    bool unique                         // swap?
) {
    std::pair<int, int> medge{face_indices[nedge], face_indices[(nedge + 1) % 3]};
    if (unique && medge.first > medge.second)
        medge = std::pair<int, int>(medge.second, medge.first);

    return medge;
}

bool InterpolateAndInsert(ChTriangleMeshConnected& mesh, int ibuffer, int i1, int i2, int& created_index) {
    switch (ibuffer) {
        case 0: {
            if (mesh.m_vertices.empty())
                return false;
            ChVector<> Vnew = (mesh.m_vertices[i1] + mesh.m_vertices[i2]) * 0.5;
            mesh.m_vertices.push_back(Vnew);
            created_index = (int)mesh.m_vertices.size() - 1;
            return true;
        }
        case 1: {
            if (mesh.m_normals.empty())
                return false;
            ChVector<> Vnew = (mesh.m_normals[i1] + mesh.m_normals[i2]) * 0.5;
            Vnew.Normalize();
            mesh.m_normals.push_back(Vnew);
            created_index = (int)mesh.m_normals.size() - 1;
            return true;
        }
        case 2: {
            if (mesh.m_UV.empty())
                return false;
            ChVector2<> Vnew = (mesh.m_UV[i1] + mesh.m_UV[i2]) * 0.5;
            mesh.m_UV.push_back(Vnew);
            created_index = (int)mesh.m_UV.size() - 1;
            return true;
        }
        case 3: {
            if (mesh.m_colors.empty())
                return false;
            ChColor Vnew = {(mesh.m_colors[i1].R + mesh.m_colors[i2].R) * 0.5f,
                            (mesh.m_colors[i1].G + mesh.m_colors[i2].G) * 0.5f,
                            (mesh.m_colors[i1].B + mesh.m_colors[i2].B) * 0.5f};
            mesh.m_colors.push_back(Vnew);
            created_index = (int)mesh.m_colors.size() - 1;
            return true;
        }
        default:
            return false;
    }
}

// Split an edge by inserting a point in the middle
// It also updates uv buffer, normals buffer, etc. and recomputes neighboring map.
bool ChTriangleMeshConnected::SplitEdge(
    int itA,                                              // triangle index,
    int itB,                                              // triangle index, -1 if not existing (means free edge on A)
    int neA,                                              // n.edge on tri A: 0,1,2
    int neB,                                              // n.edge on tri B: 0,1,2
    int& itA_1,                                           // returns the index of split triangle A, part1
    int& itA_2,                                           // returns the index of split triangle A, part2
    int& itB_1,                                           // returns the index of split triangle B, part1
    int& itB_2,                                           // returns the index of split triangle B, part2
    std::vector<std::array<int, 4>>& tri_map,             // triangle neighboring map
    std::vector<std::vector<double>*>& aux_data_double,   // auxiliary buffers to interpolate
    std::vector<std::vector<int>*>& aux_data_int,         // auxiliary buffers to interpolate
    std::vector<std::vector<bool>*>& aux_data_bool,       // auxiliary buffers to interpolate
    std::vector<std::vector<ChVector<>>*>& aux_data_vect  // auxiliary buffers to interpolate
) {
    std::array<std::vector<ChVector<int>>*, 4> face_indexes{
        &m_face_v_indices,   //
        &m_face_n_indices,   //
        &m_face_uv_indices,  //
        &m_face_col_indices  //
    };

    int iea = 0;
    int ieb = 0;

    for (int ibuffer = 0; ibuffer < 4; ++ibuffer) {
        if (face_indexes[ibuffer]->size()) {
            // case where one used normals, uv, color buffers, with custom face indexes:
            std::pair<int, int> eA;
            eA = ChTriangleMeshConnected::GetTriangleEdgeIndexes(face_indexes[ibuffer]->at(itA), neA, false);
            std::pair<int, int> eAB = eA;
            std::pair<int, int> eB;
            if (itB != -1)
                eB = ChTriangleMeshConnected::GetTriangleEdgeIndexes(face_indexes[ibuffer]->at(itB), neB, false);
            bool swapA = false;
            if (eA.first > eA.second) {
                swapA = true;
                eAB = {eA.second, eA.first};
            }
            bool swapB = false;
            if (eB.first > eB.second)
                swapB = true;

            // average new vertex/property and add it
            int iVnew = -1;
            InterpolateAndInsert(*this, ibuffer, eAB.first, eAB.second, iVnew);

            if (ibuffer == 0) {
                iea = eAB.first;
                ieb = eAB.second;
            }

            itA_1 = -1;
            itA_2 = -1;
            itB_1 = -1;
            itB_2 = -1;

            // Split triangle A in two (reuse existing, and allocate one new)
            ChVector<int> tA_1 = face_indexes[ibuffer]->at(itA);
            if (tA_1.x() == eAB.first)
                tA_1.x() = iVnew;
            if (tA_1.y() == eAB.first)
                tA_1.y() = iVnew;
            if (tA_1.z() == eAB.first)
                tA_1.z() = iVnew;
            ChVector<int> tA_2 = face_indexes[ibuffer]->at(itA);
            if (tA_2.x() == eAB.second)
                tA_2.x() = iVnew;
            if (tA_2.y() == eAB.second)
                tA_2.y() = iVnew;
            if (tA_2.z() == eAB.second)
                tA_2.z() = iVnew;
            face_indexes[ibuffer]->at(itA) = tA_1;  // reuse face
            itA_1 = itA;
            face_indexes[ibuffer]->push_back(tA_2);  // allocate new face
            itA_2 = (int)face_indexes[ibuffer]->size() - 1;

            // Split triangle B in two (reuse existing, and allocate one new)
            if (itB != -1) {
                ChVector<int> tB_1 = face_indexes[ibuffer]->at(itB);
                if (tB_1.x() == eAB.first)
                    tB_1.x() = iVnew;
                if (tB_1.y() == eAB.first)
                    tB_1.y() = iVnew;
                if (tB_1.z() == eAB.first)
                    tB_1.z() = iVnew;
                ChVector<int> tB_2 = face_indexes[ibuffer]->at(itB);
                if (tB_2.x() == eAB.second)
                    tB_2.x() = iVnew;
                if (tB_2.y() == eAB.second)
                    tB_2.y() = iVnew;
                if (tB_2.z() == eAB.second)
                    tB_2.z() = iVnew;
                face_indexes[ibuffer]->at(itB) = tB_1;  // reuse face
                itB_1 = itB;
                face_indexes[ibuffer]->push_back(tB_2);  // allocate new face
                itB_2 = (int)face_indexes[ibuffer]->size() - 1;
            }

            // for m_face_v_indices buffer (vertex indexes) only:
            if (ibuffer == 0) {
                // Update triangle neighboring map

                std::array<int, 4> topo_A_1 = tri_map[itA];
                std::array<int, 4> topo_A_2 = tri_map[itA];
                topo_A_1[1 + neA] = itB_1;
                topo_A_2[1 + neA] = itB_2;
                int is1 = 1 + ((neA + 2) % 3);
                int is2 = 1 + ((neA + 1) % 3);
                if (swapA)
                    std::swap(is1, is2);
                topo_A_1[is1] = itA_2;
                topo_A_2[is2] = itA_1;
                int itD = topo_A_1[is2];
                int itC = topo_A_2[is1];
                for (int in = 1; in < 4; ++in)
                    if (tri_map[itD][in] == itA)
                        tri_map[itD][in] = itA_1;  // not needed?
                for (int in = 1; in < 4; ++in)
                    if (tri_map[itC][in] == itA)
                        tri_map[itC][in] = itA_2;
                tri_map[itA] = topo_A_1;      // reuse
                tri_map.push_back(topo_A_2);  // allocate
                topo_A_2[0] = (int)tri_map.size() - 1;

                if (itB != -1) {
                    std::array<int, 4> topo_B_1 = tri_map[itB];
                    std::array<int, 4> topo_B_2 = tri_map[itB];
                    topo_B_1[1 + neB] = itA_1;
                    topo_B_2[1 + neB] = itA_2;
                    is1 = 1 + ((neB + 2) % 3);
                    is2 = 1 + ((neB + 1) % 3);
                    if (swapB)
                        std::swap(is1, is2);
                    topo_B_1[is1] = itB_2;
                    topo_B_2[is2] = itB_1;
                    int itF = topo_B_1[is2];
                    int itE = topo_B_2[is1];
                    for (int in = 1; in < 4; ++in)
                        if (tri_map[itF][in] == itB)
                            tri_map[itF][in] = itB_1;  // not needed?
                    for (int in = 1; in < 4; ++in)
                        if (tri_map[itE][in] == itB)
                            tri_map[itE][in] = itB_2;
                    tri_map[itB] = topo_B_1;      // reuse
                    tri_map.push_back(topo_B_2);  // allocate
                    topo_B_2[0] = (int)tri_map.size() - 1;
                }
            }
        } else {
            // case of n or uv or color buffers without indexes, because assumed matching the vertex pos buffer:
            int iVnew = -1;
            InterpolateAndInsert(*this, ibuffer, iea, ieb, iVnew);
        }

    }  // end loop on buffers

    // just in case the user populated the vector of external auxiliary data buffers,
    // interpolate and store the created value. Assume those have same size of m_vertices
    for (auto data_buffer : aux_data_double) {
        double data = (data_buffer->at(iea) + data_buffer->at(ieb)) * 0.5;
        data_buffer->push_back(data);
    }
    for (auto data_buffer : aux_data_vect) {
        ChVector<> data = (data_buffer->at(iea) + data_buffer->at(ieb)) * 0.5;
        data_buffer->push_back(data);
    }
    for (auto data_buffer : aux_data_int) {
        int data = std::max(data_buffer->at(iea), data_buffer->at(ieb));
        data_buffer->push_back(data);
    }
    for (auto data_buffer : aux_data_bool) {
        bool data = data_buffer->at(iea) || data_buffer->at(ieb);
        data_buffer->push_back(data);
    }

    return true;
}

// Performs mesh refinement using Rivara LEPP long-edge bisection algorithm.
// Based on "Multithread parallelization of Lepp-bisection algorithms"
//    M.-C. Rivara et al., Applied Numerical Mathematics 62 (2012) 473�488

void ChTriangleMeshConnected::RefineMeshEdges(
    std::vector<int>&
        marked_tris,     ///< triangles to refine (aso surrounding triangles might be affected by refinements)
    double edge_maxlen,  ///< maximum length of edge (small values give higher resolution)
    ChRefineEdgeCriterion* criterion,  ///< criterion for computing lenght (or other merit function) of edge, if =0 uses
                                       ///< default (euclidean length)
    std::vector<std::array<int, 4>>* atri_map,  ///< triangle connectivity map: use and modify it. Optional. If =0,
                                                ///< creates a temporary one just for life span of function.
    std::vector<std::vector<double>*>& aux_data_double,  ///< auxiliary buffers to refine (assuming indexed as vertexes:
                                                         ///< each with same size as vertex buffer)
    std::vector<std::vector<int>*>& aux_data_int,  ///< auxiliary buffers to refine (assuming indexed as vertexes: each
                                                   ///< with same size as vertex buffer)
    std::vector<std::vector<bool>*>& aux_data_bool,  ///< auxiliary buffers to refine (assuming indexed as vertexes:
                                                     ///< each with same size as vertex buffer)
    std::vector<std::vector<ChVector<>>*>& aux_data_vect  ///< auxiliary buffers to refine (assuming indexed as
                                                          ///< vertexes: each with same size as vertex buffer)
) {
    // initialize the list of triangles to refine, copying from marked triangles:
    std::list<int> S(marked_tris.begin(), marked_tris.end());

    // compute the connectivity map between triangles:
    std::vector<std::array<int, 4>> tmp_tri_map;
    std::vector<std::array<int, 4>>& tri_map = tmp_tri_map;
    if (atri_map) {
        tri_map = *atri_map;
    } else {
        this->ComputeNeighbouringTriangleMap(tri_map);
    }

    std::vector<bool> marked_tris_flagged;
    marked_tris_flagged.resize(this->m_face_v_indices.size());
    for (auto i : marked_tris)
        marked_tris_flagged[i] = true;

    std::vector<int> new_marked_tris;

    for (auto t_0 : S) {
        // Insert-lepp-points

        std::list<int> mlist;
        mlist.push_back(t_0);

        while (!mlist.empty()) {
            //  find last triangle in ordered list
            auto t_N = mlist.back();

            //  find longest-edge neighbour
            int t_N1 = 0;
            double L_max = 0;
            int edge_N = 0;
            double L_e;
            for (int ie = 0; ie < 3; ++ie) {
                std::pair<int, int> ie_verts = this->GetTriangleEdgeIndexes(m_face_v_indices[t_N], ie, true);
                if (criterion)
                    L_e = criterion->ComputeLength(ie_verts.first, ie_verts.second, this);
                else
                    L_e = (this->m_vertices[ie_verts.first] - this->m_vertices[ie_verts.second]).Length();
                if (L_e > L_max) {
                    L_max = L_e;
                    edge_N = ie;
                    t_N1 = tri_map[t_N][1 + ie];  // neighbour triangle
                }
            }

            if (L_max < edge_maxlen) {
                //  GetLog() << "  already small triangle - pop it and break while " << "\n";
                mlist.pop_back();
                break;
            }

            // add longest-edge neighbour to the list
            mlist.push_back(t_N1);

            if (mlist.size() > 1000)
                throw ChException("overflow in ChTriangleMeshConnected::RefineMeshEdges");

            // if boundary edge: always terminal edge
            if (t_N1 == -1) {
                // split triangle edge
                if (L_max > edge_maxlen) {
                    // do edge split
                    int itA_1, itA_2, itB_1, itB_2;
                    SplitEdge(t_N, -1, edge_N, 0, itA_1, itA_2, itB_1, itB_2, tri_map, aux_data_double, aux_data_int,
                              aux_data_bool, aux_data_vect);

                    // prepare list of original triangles after split, for the next iteration of bisection
                    if (t_N < marked_tris_flagged.size() && marked_tris_flagged[t_N] == true) {
                        new_marked_tris.push_back(itA_1);
                        new_marked_tris.push_back(itA_2);
                    }
                }

                // remove from list
                mlist.pop_back();
                mlist.pop_back();

            } else {
                //  find longest-edge in neighboring triangle
                double T1_L_max = 0;
                int edge_N1 = 0;
                int t_shared = 0;
                for (int ie = 0; ie < 3; ++ie) {
                    std::pair<int, int> T1_ie_verts = this->GetTriangleEdgeIndexes(m_face_v_indices[t_N1], ie, true);
                    if (criterion)
                        L_e = criterion->ComputeLength(T1_ie_verts.first, T1_ie_verts.second, this);
                    else
                        L_e = (this->m_vertices[T1_ie_verts.first] - this->m_vertices[T1_ie_verts.second]).Length();
                    if (L_e > T1_L_max) {
                        T1_L_max = L_e;
                        edge_N1 = ie;
                        t_shared = tri_map[t_N1][1 + ie];  // neighbour triangle
                    }
                }
                // shared terminal edge (it is the longest edge in both triangles)
                if (t_shared == t_N) {
                    // split triangle edge
                    if (L_max > edge_maxlen) {
                        // do edge split
                        int itA_1, itA_2, itB_1, itB_2;
                        SplitEdge(t_N, t_N1, edge_N, edge_N1, itA_1, itA_2, itB_1, itB_2, tri_map, aux_data_double,
                                  aux_data_int, aux_data_bool, aux_data_vect);

                        // prepare list of original triangles after split, for the next iteration of bisection
                        if (t_N < marked_tris_flagged.size() && marked_tris_flagged[t_N] == true) {
                            new_marked_tris.push_back(itA_1);
                            new_marked_tris.push_back(itA_2);
                        }
                        if (t_N1 < marked_tris_flagged.size() && marked_tris_flagged[t_N1] == true) {
                            new_marked_tris.push_back(itB_1);
                            new_marked_tris.push_back(itB_2);
                        }
                    }

                    // remove from list
                    mlist.pop_back();
                    mlist.pop_back();
                }
            }
        }
    }
    marked_tris = new_marked_tris;
}

void ChTriangleMeshConnected::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTriangleMeshConnected>();
    // serialize parent class
    ChTriangleMesh::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(m_vertices);
    marchive << CHNVP(m_normals);
    marchive << CHNVP(m_UV);
    marchive << CHNVP(m_colors);
    marchive << CHNVP(m_face_v_indices);
    marchive << CHNVP(m_face_n_indices);
    marchive << CHNVP(m_face_uv_indices);
    marchive << CHNVP(m_face_col_indices);
    marchive << CHNVP(m_face_mat_indices);
    marchive << CHNVP(m_filename);
    marchive << CHNVP(m_properties_per_vertex);
    marchive << CHNVP(m_properties_per_face);
}

void ChTriangleMeshConnected::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChTriangleMeshConnected>();
    // deserialize parent class
    ChTriangleMesh::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_vertices);
    marchive >> CHNVP(m_normals);
    marchive >> CHNVP(m_UV);
    marchive >> CHNVP(m_colors);
    marchive >> CHNVP(m_face_v_indices);
    marchive >> CHNVP(m_face_n_indices);
    marchive >> CHNVP(m_face_uv_indices);
    marchive >> CHNVP(m_face_col_indices);
    marchive >> CHNVP(m_face_mat_indices);
    marchive >> CHNVP(m_filename);
    marchive >> CHNVP(m_properties_per_vertex);
    marchive >> CHNVP(m_properties_per_face);
}

}  // end namespace geometry
}  // end namespace chrono
