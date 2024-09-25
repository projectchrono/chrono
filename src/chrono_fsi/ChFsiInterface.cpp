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

#include <vector>

#include "chrono_fsi/ChFsiInterface.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

// =============================================================================

ChFsiInterface::ChFsiInterface(bool verbose) : m_verbose(verbose) {}

ChFsiInterface::~ChFsiInterface() {}

// ------------

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

// ------------

const ChVector3d& ChFsiInterface::GetFsiBodyForce(size_t i) const {
    return m_fsi_bodies[i].fsi_force;
}

const ChVector3d& ChFsiInterface::GetFsiBodyTorque(size_t i) const {
    return m_fsi_bodies[i].fsi_torque;
}

// ------------

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

// ------------

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

// ------------

void ChFsiInterface::AllocateStateVectors(std::vector<FsiBodyState>& body_states,
                                          std::vector<FsiMeshState>& mesh1D_states,
                                          std::vector<FsiMeshState>& mesh2D_states) const {
    unsigned int num_bodies = GetNumBodies();
    unsigned int num_meshes1D = GetNumMeshes1D();
    unsigned int num_meshes2D = GetNumMeshes2D();

    body_states.resize(num_bodies);

    mesh1D_states.resize(num_meshes1D);
    for (unsigned int imesh = 0; imesh < num_meshes1D; imesh++) {
        unsigned int num_nodes = m_fsi_meshes1D[imesh].GetNumNodes();
        mesh1D_states[imesh].pos.resize(num_nodes);
        mesh1D_states[imesh].vel.resize(num_nodes);
        mesh1D_states[imesh].acc.resize(num_nodes);
    }

    mesh2D_states.resize(num_meshes2D);
    for (unsigned int imesh = 0; imesh < num_meshes2D; imesh++) {
        unsigned int num_nodes = m_fsi_meshes2D[imesh].GetNumNodes();
        mesh2D_states[imesh].pos.resize(num_nodes);
        mesh2D_states[imesh].vel.resize(num_nodes);
        mesh2D_states[imesh].acc.resize(num_nodes);
    }
}

bool ChFsiInterface::CheckStateVectors(const std::vector<FsiBodyState>& body_states,
                                       const std::vector<FsiMeshState>& mesh1D_states,
                                       const std::vector<FsiMeshState>& mesh2D_states) const {
    if (body_states.size() != m_fsi_bodies.size()) {
        cerr << "ERROR (ChFsiInterface::GetInitialBodyStates) incorrect size for vector of body states.";
        return false;
    }

    if (mesh1D_states.size() != m_fsi_meshes1D.size()) {
        cerr << "ERROR (ChFsiInterface::GetInitialMeshStates1D) incorrect size for vector of mesh node states.";
        return false;
    }

    //// TODO: check nodes

    if (mesh2D_states.size() != m_fsi_meshes2D.size()) {
        cerr << "ERROR (ChFsiInterface::GetInitialMeshStates2D) incorrect size for vector of mesh node states.";
        return false;
    }

    //// TODO: check nodes

    return true;
}

void ChFsiInterface::LoadStateVectors(std::vector<FsiBodyState>& body_states,
                                      std::vector<FsiMeshState>& mesh1D_states,
                                      std::vector<FsiMeshState>& mesh2D_states) {
    if (!CheckStateVectors(body_states, mesh1D_states, mesh2D_states)) {
        throw std::runtime_error("(ChFsiInterface::LoadStateVectors) incorrect state vector sizes.");
    }

    {
        size_t num_bodies = m_fsi_bodies.size();
        for (size_t ibody = 0; ibody < num_bodies; ibody++) {
            std::shared_ptr<ChBody> body = m_fsi_bodies[ibody].body;

            body_states[ibody].pos = body->GetPos();
            body_states[ibody].lin_vel = body->GetPosDt();
            body_states[ibody].lin_acc = body->GetPosDt2();
            body_states[ibody].rot = body->GetRot();
            body_states[ibody].ang_vel = body->GetAngVelLocal();
            body_states[ibody].ang_acc = body->GetAngAccLocal();
        }
    }

    {
        int imesh = 0;
        for (const auto& fsi_mesh : m_fsi_meshes1D) {
            int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (int inode = 0; inode < num_nodes; inode++) {
                const auto& node = fsi_mesh.ind2ptr_map.at(inode);
                mesh1D_states[imesh].pos[inode] = node->GetPos();
                mesh1D_states[imesh].vel[inode] = node->GetPosDt();
                mesh1D_states[imesh].acc[inode] = node->GetPosDt2();
            }
            imesh++;
        }
    }

    {
        int imesh = 0;
        for (const auto& fsi_mesh : m_fsi_meshes2D) {
            int num_nodes = (int)fsi_mesh.ind2ptr_map.size();
            for (int inode = 0; inode < num_nodes; inode++) {
                const auto& node = fsi_mesh.ind2ptr_map.at(inode);
                mesh2D_states[imesh].pos[inode] = node->GetPos();
                mesh2D_states[imesh].vel[inode] = node->GetPosDt();
                mesh2D_states[imesh].acc[inode] = node->GetPosDt2();
            }
            imesh++;
        }
    }
}

void ChFsiInterface::AllocateForceVectors(std::vector<FsiBodyForce>& body_forces,
                                          std::vector<FsiMeshForce>& mesh_forces1D,
                                          std::vector<FsiMeshForce>& mesh_forces2D) const {
    unsigned int num_bodies = GetNumBodies();
    unsigned int num_meshes1D = GetNumMeshes1D();
    unsigned int num_meshes2D = GetNumMeshes2D();

    body_forces.resize(num_bodies);

    mesh_forces1D.resize(num_meshes1D);
    for (unsigned int imesh = 0; imesh < num_meshes1D; imesh++) {
        unsigned int num_nodes = m_fsi_meshes1D[imesh].GetNumNodes();
        mesh_forces1D[imesh].force.resize(num_nodes);
    }

    mesh_forces2D.resize(num_meshes2D);
    for (unsigned int imesh = 0; imesh < num_meshes2D; imesh++) {
        unsigned int num_nodes = m_fsi_meshes2D[imesh].GetNumNodes();
        mesh_forces2D[imesh].force.resize(num_nodes);
    }
}

bool ChFsiInterface::CheckForceVectors(const std::vector<FsiBodyForce>& body_forces,
                                       const std::vector<FsiMeshForce>& mesh_forces1D,
                                       const std::vector<FsiMeshForce>& mesh_forces2D) const {
    //// TODO

    return true;
}

// =============================================================================

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
