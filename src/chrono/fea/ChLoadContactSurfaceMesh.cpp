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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono_fea/ChLoadContactSurfaceMesh.h"

namespace chrono {
namespace fea {

ChLoadContactSurfaceMesh::ChLoadContactSurfaceMesh(std::shared_ptr<ChContactSurfaceMesh> cmesh) : contactmesh(cmesh) {}

void ChLoadContactSurfaceMesh::OutputSimpleMesh(std::vector<ChVector<>>& vert_pos,
                                                std::vector<ChVector<>>& vert_vel,
                                                std::vector<ChVector<int>>& triangles) {
    vert_pos.clear();
    vert_vel.clear();
    triangles.clear();
    size_t vertex_index = 0;
    auto trilist = this->contactmesh->GetTriangleList();
    // auxiliary map container to go from pointer-based mesh to index-based mesh:
    std::map<ChNodeFEAxyz*, size_t> ptr_ind_map;
    for (size_t i = 0; i < trilist.size(); ++i) {
        if (!ptr_ind_map.count(trilist[i]->GetNode1().get())) {
            ptr_ind_map.insert({trilist[i]->GetNode1().get(), vertex_index});
            vert_pos.push_back(trilist[i]->GetNode1()->GetPos());
            vert_vel.push_back(trilist[i]->GetNode1()->GetPos_dt());
            ++vertex_index;
        }
        if (!ptr_ind_map.count(trilist[i]->GetNode2().get())) {
            ptr_ind_map.insert({trilist[i]->GetNode2().get(), vertex_index});
            vert_pos.push_back(trilist[i]->GetNode2()->GetPos());
            vert_vel.push_back(trilist[i]->GetNode2()->GetPos_dt());
            ++vertex_index;
        }
        if (!ptr_ind_map.count(trilist[i]->GetNode3().get())) {
            ptr_ind_map.insert({trilist[i]->GetNode3().get(), vertex_index});
            vert_pos.push_back(trilist[i]->GetNode3()->GetPos());
            vert_vel.push_back(trilist[i]->GetNode3()->GetPos_dt());
            ++vertex_index;
        }
    }
    for (size_t i = 0; i < trilist.size(); ++i) {
        triangles.push_back(ChVector<int>((int)ptr_ind_map.at(trilist[i]->GetNode1().get()),
                                          (int)ptr_ind_map.at(trilist[i]->GetNode2().get()),
                                          (int)ptr_ind_map.at(trilist[i]->GetNode3().get())));
    }
}

void ChLoadContactSurfaceMesh::InputSimpleForces(const std::vector<ChVector<>> vert_forces,
                                                 const std::vector<int> vert_ind) {
    // check the vert_forces and vert_ind arrays must have same size:
    assert(vert_forces.size() == vert_ind.size());
    // reset the previously applied forces if any:
    this->forces.clear();

    // prepare auxiliary map container to go from index-based mesh to pointer-based mesh:
    size_t vertex_index = 0;
    auto trilist = this->contactmesh->GetTriangleList();
    std::map<ChNodeFEAxyz*, size_t> ptr_ind_map;
    std::vector<std::shared_ptr<ChNodeFEAxyz>> ind_ptr_map;
    for (size_t i = 0; i < trilist.size(); ++i) {
        if (!ptr_ind_map.count(trilist[i]->GetNode1().get())) {
            ptr_ind_map.insert({trilist[i]->GetNode1().get(), vertex_index});
            ind_ptr_map.push_back(trilist[i]->GetNode1());
            ++vertex_index;
        }
        if (!ptr_ind_map.count(trilist[i]->GetNode2().get())) {
            ptr_ind_map.insert({trilist[i]->GetNode2().get(), vertex_index});
            ind_ptr_map.push_back(trilist[i]->GetNode2());
            ++vertex_index;
        }
        if (!ptr_ind_map.count(trilist[i]->GetNode3().get())) {
            ptr_ind_map.insert({trilist[i]->GetNode3().get(), vertex_index});
            ind_ptr_map.push_back(trilist[i]->GetNode3());
            ++vertex_index;
        }
    }
    // Populate the array of applied loads to nodes
    for (size_t i = 0; i < vert_forces.size(); ++i) {
        std::shared_ptr<ChNodeFEAxyz> mnode = ind_ptr_map[vert_ind[i]];
        auto mforce = std::make_shared<ChLoadXYZnode>(mnode);
        mforce->loader.SetForce(vert_forces[i]);
        this->forces.push_back(mforce);
    }
}

void ChLoadContactSurfaceMesh::SetContactMesh(std::shared_ptr<ChContactSurfaceMesh> mmesh) {
    contactmesh = mmesh;
    forces.clear();
}

// -----------------------------------------------------------------------------

int ChLoadContactSurfaceMesh::LoadGet_ndof_x() {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i)
        ndoftot += forces[i]->LoadGet_ndof_x();
    return ndoftot;
}

int ChLoadContactSurfaceMesh::LoadGet_ndof_w() {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i)
        ndoftot += forces[i]->LoadGet_ndof_w();
    return ndoftot;
}

void ChLoadContactSurfaceMesh::LoadGetStateBlock_x(ChState& mD) {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->loader.GetLoadable()->LoadableGetStateBlock_x(ndoftot, mD);
        ndoftot += forces[i]->loader.GetLoadable()->LoadableGet_ndof_x();
    }
}

void ChLoadContactSurfaceMesh::LoadGetStateBlock_w(ChStateDelta& mD) {
    int ndoftot = 0;
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->loader.GetLoadable()->LoadableGetStateBlock_w(ndoftot, mD);
        ndoftot += forces[i]->loader.GetLoadable()->LoadableGet_ndof_w();
    }
}

void ChLoadContactSurfaceMesh::LoadStateIncrement(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    int ndoftotx = 0;
    int ndoftotw = 0;
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->loader.GetLoadable()->LoadableStateIncrement(ndoftotx, x_new, x, ndoftotw, dw);
        ndoftotx += forces[i]->loader.GetLoadable()->LoadableGet_ndof_x();
        ndoftotw += forces[i]->loader.GetLoadable()->LoadableGet_ndof_w();
    }
}

// -----------------------------------------------------------------------------

void ChLoadContactSurfaceMesh::ComputeQ(ChState* state_x, ChStateDelta* state_w) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->ComputeQ(state_x, state_w);
    }
}

void ChLoadContactSurfaceMesh::ComputeJacobian(ChState* state_x,
                                               ChStateDelta* state_w,
                                               ChMatrix<>& mK,
                                               ChMatrix<>& mR,
                                               ChMatrix<>& mM) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->ComputeJacobian(state_x, state_w, mK, mR, mM);
    }
}

// -----------------------------------------------------------------------------

void ChLoadContactSurfaceMesh::CreateJacobianMatrices() {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->CreateJacobianMatrices();
    }
}

void ChLoadContactSurfaceMesh::LoadIntLoadResidual_F(ChVectorDynamic<>& R, const double c) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->LoadIntLoadResidual_F(R, c);
    }
}

void ChLoadContactSurfaceMesh::InjectKRMmatrices(ChSystemDescriptor& mdescriptor) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->InjectKRMmatrices(mdescriptor);
    }
}

void ChLoadContactSurfaceMesh::KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) {
    for (int i = 0; i < forces.size(); ++i) {
        forces[i]->KRMmatricesLoad(Kfactor, Rfactor, Mfactor);
    }
}

}  // end namespace fea
}  // end namespace chrono
