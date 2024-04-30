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

#include <cstdio>
#include <iostream>

#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono_thirdparty/tinyobjloader/tiny_obj_loader.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTriangleMeshSoup)

ChTriangleMeshSoup::ChTriangleMeshSoup(const ChTriangleMeshSoup& source) {
    m_triangles = source.m_triangles;
}

std::shared_ptr<ChTriangleMeshSoup> ChTriangleMeshSoup::CreateFromWavefrontFile(const std::string& filename) {
    auto trimesh = chrono_types::make_shared<ChTriangleMeshSoup>();
    if (!trimesh->LoadWavefrontMesh(filename))
        return nullptr;
    return trimesh;
}

bool ChTriangleMeshSoup::LoadWavefrontMesh(std::string filename) {
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

    for (size_t i = 0; i < shapes.size(); i++) {
        assert((shapes[i].mesh.indices.size() % 3) == 0);
        for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {
            auto& indices = shapes[i].mesh.indices;
            int i0 = indices[3 * f + 0].vertex_index;
            int i1 = indices[3 * f + 1].vertex_index;
            int i2 = indices[3 * f + 2].vertex_index;
            auto v0 = ChVector3d(att.vertices[3 * i0 + 0], att.vertices[3 * i0 + 1], att.vertices[3 * i0 + 2]);
            auto v1 = ChVector3d(att.vertices[3 * i1 + 0], att.vertices[3 * i1 + 1], att.vertices[3 * i1 + 2]);
            auto v2 = ChVector3d(att.vertices[3 * i2 + 0], att.vertices[3 * i2 + 1], att.vertices[3 * i2 + 2]);
            AddTriangle(v0, v1, v2);
        }
    }

    return true;
}

void ChTriangleMeshSoup::AddTriangle(const ChVector3d& vertex0, const ChVector3d& vertex1, const ChVector3d& vertex2) {
    ChTriangle tri(vertex0, vertex1, vertex2);
    m_triangles.push_back(tri);
}

void ChTriangleMeshSoup::Transform(const ChVector3d displ, const ChMatrix33<> rotscale) {
    for (int i = 0; i < this->m_triangles.size(); ++i) {
        m_triangles[i].p1 = rotscale * m_triangles[i].p1;
        m_triangles[i].p1 += displ;
        m_triangles[i].p2 = rotscale * m_triangles[i].p2;
        m_triangles[i].p2 += displ;
        m_triangles[i].p3 = rotscale * m_triangles[i].p3;
        m_triangles[i].p3 += displ;
    }
}

void ChTriangleMeshSoup::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChTriangleMeshSoup>();
    // serialize parent class
    ChTriangleMesh::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_triangles);
}

void ChTriangleMeshSoup::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChTriangleMeshSoup>();
    // deserialize parent class
    ChTriangleMesh::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_triangles);
}

}  // end namespace chrono
