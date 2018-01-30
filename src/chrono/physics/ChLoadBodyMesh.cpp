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

#include "chrono/physics/ChLoadBodyMesh.h"

namespace chrono {

ChLoadBodyMesh::ChLoadBodyMesh(std::shared_ptr<ChBody> cbody, geometry::ChTriangleMeshConnected& cmesh) {
    contactbody = cbody;
    contactmesh = cmesh;
}

void ChLoadBodyMesh::OutputSimpleMesh(
    std::vector<ChVector<>>& vert_pos,     // array of vertexes (absolute xyz positions)
    std::vector<ChVector<>>& vert_vel,     // array of vertexes (absolute xyz velocities, might be useful)
    std::vector<ChVector<int>>& triangles  // array of triangles (indexes to vertexes, ccw)
) {
    vert_pos.resize(contactmesh.m_vertices.size());
    vert_vel.resize(contactmesh.m_vertices.size());
    triangles = contactmesh.m_face_v_indices;
    // Transform the body-relative collision mesh into the output vectors with positions and speeds in absolute coords
    for (size_t i = 0; i < contactmesh.m_vertices.size(); ++i) {
        vert_pos[i] = contactbody->TransformPointLocalToParent(contactmesh.m_vertices[i]);
        vert_vel[i] = contactbody->PointSpeedLocalToParent(contactmesh.m_vertices[i]);
    }
}

void ChLoadBodyMesh::InputSimpleForces(
    const std::vector<ChVector<>> vert_forces,  // array of forces (absolute xyz forces in [N])
    const std::vector<int> vert_ind             // array of indexes to vertexes to whom you apply forces
) {
    // check the vert_forces and vert_ind arrays must have same size:
    assert(vert_forces.size() == vert_ind.size());
    // reset the previously applied forces if any:
    this->forces.clear();

    // Populate the array of applied loads to nodes
    for (size_t i = 0; i < vert_forces.size(); ++i) {
        ChVector<> rel_application = contactmesh.m_vertices[vert_ind[i]];

        std::shared_ptr<ChLoadBodyForce> mforce(
            new ChLoadBodyForce(contactbody, vert_forces[i], false, rel_application, true));
        this->forces.push_back(mforce);
    }
}

void ChLoadBodyMesh::SetContactMesh(geometry::ChTriangleMeshConnected& mmesh) {
    this->contactmesh = mmesh;
    this->forces.clear();
}

int ChLoadBodyMesh::LoadGet_ndof_x() {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i)
        ndoftot += forces[i]->LoadGet_ndof_x();
    return ndoftot;
}

int ChLoadBodyMesh::LoadGet_ndof_w() {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i)
        ndoftot += forces[i]->LoadGet_ndof_w();
    return ndoftot;
}

void ChLoadBodyMesh::LoadGetStateBlock_x(ChState& mD) {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i) {
        ChState mDi(forces[i]->LoadGet_ndof_x(), nullptr);
        forces[i]->LoadGetStateBlock_x(mDi);
        mD.PasteMatrix(mDi, ndoftot, 0);
        ndoftot += forces[i]->LoadGet_ndof_x();
    }
}

void ChLoadBodyMesh::LoadGetStateBlock_w(ChStateDelta& mD) {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i) {
        ChStateDelta mDi(forces[i]->LoadGet_ndof_w(), nullptr);
        forces[i]->LoadGetStateBlock_w(mDi);
        mD.PasteMatrix(mDi, ndoftot, 0);
        ndoftot += forces[i]->LoadGet_ndof_w();
    }
}

void ChLoadBodyMesh::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    int ndoftotx = 0;
    int ndoftotw = 0;
    for (int i = 0; i < forces.size(); ++i) {
        ChState mx_inc(forces[i]->LoadGet_ndof_x(), nullptr);
        ChState mx(forces[i]->LoadGet_ndof_x(), nullptr);
        ChStateDelta mDi(forces[i]->LoadGet_ndof_w(), nullptr);
        mx.PasteClippedMatrix(x, ndoftotx, 0, forces[i]->LoadGet_ndof_x(), 1, 0, 0);
        mDi.PasteClippedMatrix(dw, ndoftotw, 0, forces[i]->LoadGet_ndof_w(), 1, 0, 0);
        forces[i]->LoadStateIncrement(mx, mDi, mx_inc);
        x_new.PasteMatrix(mx_inc, ndoftotx, 0);
        ndoftotx += forces[i]->LoadGet_ndof_x();
        ndoftotw += forces[i]->LoadGet_ndof_w();
    }
}

void ChLoadBodyMesh::ComputeQ(ChState* state_x,      // state position to evaluate Q
                              ChStateDelta* state_w  // state speed to evaluate Q
) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->ComputeQ(state_x, state_w);
    }
}

void ChLoadBodyMesh::ComputeJacobian(ChState* state_x,       // state position to evaluate jacobians
                                     ChStateDelta* state_w,  // state speed to evaluate jacobians
                                     ChMatrix<>& mK,         // result dQ/dx
                                     ChMatrix<>& mR,         // result dQ/dv
                                     ChMatrix<>& mM)         // result dQ/da
{
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->ComputeJacobian(state_x, state_w, mK, mR, mM);
    }
}

void ChLoadBodyMesh::CreateJacobianMatrices() {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->CreateJacobianMatrices();
    }
}

void ChLoadBodyMesh::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->LoadIntLoadResidual_F(R, c);
    }
};

void ChLoadBodyMesh::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->InjectKRMmatrices(mdescriptor);
    }
}

void ChLoadBodyMesh::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
}

}  // end namespace chrono