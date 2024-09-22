// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Base class for interfacing between a Chrono system and an FSI system
// =============================================================================

#include "chrono_fsi/ChFsiInterface.h"

namespace chrono {
namespace fsi {

ChFsiInterface::ChFsiInterface(bool verbose) : m_verbose(verbose) {}

ChFsiInterface::~ChFsiInterface() {}

// -----------------------------------------------------------------------------

unsigned int ChFsiInterface::GetNumBodies() const {
    return (unsigned int)m_fsi_bodies.size();
}

unsigned int ChFsiInterface::GetNumMeshes1D() const {
    return (unsigned int)m_fsi_meshes1D.size();
}

unsigned int ChFsiInterface::GetNumMeshes2D() const {
    return (unsigned int)m_fsi_meshes2D.size();
}

// -----------------------------------------------------------------------------

ChFsiInterface::FsiBody& ChFsiInterface::AddFsiBody(std::shared_ptr<ChBody> body) {
    FsiBody fsi_body;
    fsi_body.body = body;
    fsi_body.fsi_force = VNULL;
    fsi_body.fsi_torque = VNULL;

    // Store the body
    m_fsi_bodies.push_back(fsi_body);

    return m_fsi_bodies.back();
}

ChFsiInterface::FsiMesh1D& ChFsiInterface::AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface) {
    FsiMesh1D fsi_mesh;
    fsi_mesh.contact_surface = surface;

    // Create maps from pointer-based to index-based for the nodes in the mesh contact segments.
    // These maps index only the nodes that are in ANCF cable elements (and not all nodes in the given FEA mesh).
    int vertex_index = 0;
    for (const auto& seg : surface->GetSegmentsXYZ()) {
        if (fsi_mesh.ptr2ind_map.insert({seg->GetNode(0), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, seg->GetNode(0)});
            ++vertex_index;
        }
        if (fsi_mesh.ptr2ind_map.insert({seg->GetNode(1), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, seg->GetNode(1)});
            ++vertex_index;
        }
    }

    assert(fsi_mesh.ptr2ind_map.size() == surface->GetNumVertices());
    assert(fsi_mesh.ind2ptr_map.size() == surface->GetNumVertices());

    // Store the mesh contact surface
    m_fsi_meshes1D.push_back(fsi_mesh);

    return m_fsi_meshes1D.back();
}

ChFsiInterface::FsiMesh2D& ChFsiInterface::AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface) {
    FsiMesh2D fsi_mesh;
    fsi_mesh.contact_surface = surface;

    // Create maps from pointer-based to index-based for the nodes in the mesh contact surface.
    // These maps index only the nodes that are in the contact surface (and not all nodes in the given FEA mesh).
    int vertex_index = 0;
    for (const auto& tri : surface->GetTrianglesXYZ()) {
        if (fsi_mesh.ptr2ind_map.insert({tri->GetNode(0), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, tri->GetNode(0)});
            ++vertex_index;
        }
        if (fsi_mesh.ptr2ind_map.insert({tri->GetNode(1), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, tri->GetNode(1)});
            ++vertex_index;
        }
        if (fsi_mesh.ptr2ind_map.insert({tri->GetNode(2), vertex_index}).second) {
            fsi_mesh.ind2ptr_map.insert({vertex_index, tri->GetNode(2)});
            ++vertex_index;
        }
    }

    assert(fsi_mesh.ptr2ind_map.size() == surface->GetNumVertices());
    assert(fsi_mesh.ind2ptr_map.size() == surface->GetNumVertices());

    // Store the mesh contact surface
    m_fsi_meshes2D.push_back(fsi_mesh);

    return m_fsi_meshes2D.back();
}

}  // end namespace fsi
}  // end namespace chrono
