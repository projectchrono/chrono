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

using std::cout;
using std::cerr;
using std::endl;

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

unsigned int ChFsiInterface::GetNumElements1D() const {
    unsigned int n = 0;
    for (const auto& m : m_fsi_meshes1D)
        n += m.GetNumElements();
    return n;
}

unsigned int ChFsiInterface::GetNumNodes1D() const {
    unsigned int n = 0;
    for (const auto& m : m_fsi_meshes1D)
        n += m.GetNumNodes();
    return n;
}

unsigned int ChFsiInterface::GetNumElements2D() const {
    unsigned int n = 0;
    for (const auto& m : m_fsi_meshes2D)
        n += m.GetNumElements();
    return n;
}

unsigned int ChFsiInterface::GetNumNodes2D() const {
    unsigned int n = 0;
    for (const auto& m : m_fsi_meshes2D)
        n += m.GetNumNodes();
    return n;
}

const ChVector3d& ChFsiInterface::GetFsiBodyForce(size_t i) const {
    return m_fsi_bodies[i].fsi_force;
}

const ChVector3d& ChFsiInterface::GetFsiBodyTorque(size_t i) const {
    return m_fsi_bodies[i].fsi_torque;
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

void ChFsiInterface::Initialize() {
    if (m_verbose) {
        cout << "FSI interface solids" << endl;
        cout << "  Num. bodies:     " << GetNumBodies() << endl;
        cout << "  Num meshes 1D:   " << GetNumMeshes1D() << endl;
        cout << "  Num nodes 1D:    " << GetNumNodes1D() << endl;
        cout << "  Num elements 1D: " << GetNumElements1D() << endl;
        cout << "  Num meshes 2D:   " << GetNumMeshes2D() << endl;
        cout << "  Num nodes 2D:    " << GetNumNodes2D() << endl;
        cout << "  Num elements 2D: " << GetNumElements2D() << endl;
    }
}

// -----------------------------------------------------------------------------

unsigned int ChFsiInterface::FsiMesh1D::GetNumElements() const {
    return contact_surface->GetNumSegments();
}

unsigned int ChFsiInterface::FsiMesh1D::GetNumNodes() const {
    return (unsigned int)ind2ptr_map.size();
}

unsigned int ChFsiInterface::FsiMesh2D::GetNumElements() const {
    return contact_surface->GetNumTriangles();
}

unsigned int ChFsiInterface::FsiMesh2D::GetNumNodes() const {
    return (unsigned int)ind2ptr_map.size();
}

}  // end namespace fsi
}  // end namespace chrono
