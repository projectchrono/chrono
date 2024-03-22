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
#include "chrono/physics/ChSystem.h"

namespace chrono {

ChLoadBodyMesh::ChLoadBodyMesh(std::shared_ptr<ChBody> body, const ChTriangleMeshConnected& mesh)
    : m_body(body), m_mesh(mesh) {}

void ChLoadBodyMesh::OutputSimpleMesh(std::vector<ChVector3d>& vert_pos,
                                      std::vector<ChVector3d>& vert_vel,
                                      std::vector<ChVector3i>& triangles) {
    vert_pos.resize(m_mesh.m_vertices.size());
    vert_vel.resize(m_mesh.m_vertices.size());
    triangles = m_mesh.m_face_v_indices;

    // Transform the body-relative collision mesh into the output vectors with positions and speeds in absolute coords
    for (size_t i = 0; i < m_mesh.m_vertices.size(); ++i) {
        vert_pos[i] = m_body->TransformPointLocalToParent(m_mesh.m_vertices[i]);
        vert_vel[i] = m_body->PointSpeedLocalToParent(m_mesh.m_vertices[i]);
    }
}

void ChLoadBodyMesh::InputSimpleForces(const std::vector<ChVector3d> vert_forces, const std::vector<int> vert_indices) {
    // check the vert_forces and vert_ind arrays must have same size
    assert(vert_forces.size() == vert_indices.size());

    // reset the previously applied forces if any
    forces.clear();

    // Populate the array of applied loads to nodes
    for (size_t i = 0; i < vert_forces.size(); ++i) {
        ChVector3d rel_application = m_mesh.m_vertices[vert_indices[i]];

        std::shared_ptr<ChLoadBodyForce> force(
            new ChLoadBodyForce(m_body, vert_forces[i], false, rel_application, true));
        forces.push_back(force);
    }

    // Force an update of the system containing the associated body
    m_body->GetSystem()->ForceUpdate();
}

void ChLoadBodyMesh::SetContactMesh(const ChTriangleMeshConnected& mesh) {
    m_mesh = mesh;
    forces.clear();
}

int ChLoadBodyMesh::LoadGetNumCoordsPosLevel() {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i)
        ndoftot += forces[i]->LoadGetNumCoordsPosLevel();
    return ndoftot;
}

int ChLoadBodyMesh::LoadGetNumCoordsVelLevel() {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i)
        ndoftot += forces[i]->LoadGetNumCoordsVelLevel();
    return ndoftot;
}

void ChLoadBodyMesh::LoadGetStateBlock_x(ChState& mD) {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i) {
        ChState mDi(forces[i]->LoadGetNumCoordsPosLevel(), nullptr);
        forces[i]->LoadGetStateBlock_x(mDi);
        mD.segment(ndoftot, mDi.size()) = mDi;
        ndoftot += forces[i]->LoadGetNumCoordsPosLevel();
    }
}

void ChLoadBodyMesh::LoadGetStateBlock_w(ChStateDelta& mD) {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i) {
        ChStateDelta mDi(forces[i]->LoadGetNumCoordsVelLevel(), nullptr);
        forces[i]->LoadGetStateBlock_w(mDi);
        mD.segment(ndoftot, mDi.size()) = mDi;
        ndoftot += forces[i]->LoadGetNumCoordsVelLevel();
    }
}

void ChLoadBodyMesh::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    int ndoftotx = 0;
    int ndoftotw = 0;
    for (int i = 0; i < forces.size(); ++i) {
        ChState mx_inc(forces[i]->LoadGetNumCoordsPosLevel(), nullptr);
        ChState mx(forces[i]->LoadGetNumCoordsPosLevel(), nullptr);
        ChStateDelta mDi(forces[i]->LoadGetNumCoordsVelLevel(), nullptr);
        mx = x.segment(ndoftotx, mx.size());
        mDi = dw.segment(ndoftotw, mDi.size());
        forces[i]->LoadStateIncrement(mx, mDi, mx_inc);
        x_new.segment(ndoftotx, mx_inc.size()) = mx_inc;
        ndoftotx += forces[i]->LoadGetNumCoordsPosLevel();
        ndoftotw += forces[i]->LoadGetNumCoordsVelLevel();
    }
}

void ChLoadBodyMesh::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->ComputeQ(state_x, state_w);
    }
}

void ChLoadBodyMesh::ComputeJacobian(ChState* state_x, ChStateDelta* state_w) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->ComputeJacobian(state_x, state_w);
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

void ChLoadBodyMesh::InjectKRMMatrices(ChSystemDescriptor& descriptor) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->InjectKRMMatrices(descriptor);
    }
}

void ChLoadBodyMesh::LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->LoadKRMMatrices(Kfactor, Rfactor, Mfactor);
    }
}

}  // end namespace chrono