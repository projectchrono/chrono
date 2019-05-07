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

#include "chrono/fea/ChElementSpring.h"

namespace chrono {
namespace fea {

ChElementSpring::ChElementSpring() : spring_k(1.0), damper_r(0.01) {
    nodes.resize(2);
}

ChElementSpring::~ChElementSpring() {}

void ChElementSpring::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA, std::shared_ptr<ChNodeFEAxyz> nodeB) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementSpring::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.Reset(this->GetNdofs(), 1);
    mD.PasteVector(this->nodes[0]->GetPos(), 0, 0);
    mD.PasteVector(this->nodes[1]->GetPos(), 3, 0);
}

void ChElementSpring::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 6) && (H.GetColumns() == 6));

    // compute stiffness matrix (this is already the explicit
    // formulation of the corotational stiffness matrix in 3D)

    ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    ChMatrixNM<double, 3, 1> dircolumn;
    dircolumn.PasteVector(dir, 0, 0);

    ChMatrix33<> submatr;
    submatr.MatrMultiplyT(dircolumn, dircolumn);

    // note that stiffness and damping matrices are the same, so join stuff here
    double commonfactor = this->spring_k * Kfactor + this->damper_r * Rfactor;
    submatr.MatrScale(commonfactor);
    H.PasteMatrix(submatr, 0, 0);
    H.PasteMatrix(submatr, 3, 3);
    submatr.MatrNeg();
    H.PasteMatrix(submatr, 0, 3);
    H.PasteMatrix(submatr, 3, 0);

    // finally, do nothing about mass matrix because this element is mass-less
}

void ChElementSpring::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    assert((Fi.GetRows() == 6) && (Fi.GetColumns() == 1));

    ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    double L_dt = Vdot((nodes[1]->GetPos_dt() - nodes[0]->GetPos_dt()), dir);
    double internal_Kforce_local = this->spring_k * (L - L_ref);
    double internal_Rforce_local = this->damper_r * L_dt;
    double internal_force_local = internal_Kforce_local + internal_Rforce_local;
    ChVector<> int_forceA = dir * internal_force_local;
    ChVector<> int_forceB = -dir * internal_force_local;
    Fi.PasteVector(int_forceA, 0, 0);
    Fi.PasteVector(int_forceB, 3, 0);
}

}  // end namespace fea
}  // end namespace chrono
