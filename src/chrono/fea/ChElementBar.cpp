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

#include "chrono/fea/ChElementBar.h"

namespace chrono {
namespace fea {

ChElementBar::ChElementBar() : area(0.01 * 0.01), density(1000), E(0.01e9), rdamping(0.01), length(0), mass(0) {
    nodes.resize(2);
}

ChElementBar::~ChElementBar() {}

void ChElementBar::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA, std::shared_ptr<ChNodeFEAxyz> nodeB) {
    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementBar::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.Reset(this->GetNdofs(), 1);
    mD.PasteVector(this->nodes[0]->GetPos(), 0, 0);
    mD.PasteVector(this->nodes[1]->GetPos(), 3, 0);
}

void ChElementBar::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 6) && (H.GetColumns() == 6));

    // For K stiffness matrix and R damping matrix:
    // compute stiffness matrix (this is already the explicit
    // formulation of the corotational stiffness matrix in 3D)

    ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    ChMatrixNM<double, 3, 1> dircolumn;
    dircolumn.PasteVector(dir, 0, 0);

    ChMatrix33<> submatr;
    submatr.MatrMultiplyT(dircolumn, dircolumn);

    double Kstiffness = ((this->area * this->E) / this->length);
    double Rdamping = this->rdamping * Kstiffness;

    // note that stiffness and damping matrices are the same, so join stuff here
    double commonfactor = Kstiffness * Kfactor + Rdamping * Rfactor;
    submatr.MatrScale(commonfactor);
    H.PasteMatrix(submatr, 0, 0);
    H.PasteMatrix(submatr, 3, 3);
    submatr.MatrNeg();
    H.PasteMatrix(submatr, 0, 3);
    H.PasteMatrix(submatr, 3, 0);

    // For M mass matrix, do mass lumping:
    H(0, 0) += Mfactor * mass * 0.5;  // node A x,y,z
    H(1, 1) += Mfactor * mass * 0.5;
    H(2, 2) += Mfactor * mass * 0.5;
    H(3, 3) += Mfactor * mass * 0.5;  // node B x,y,z
    H(4, 4) += Mfactor * mass * 0.5;
    H(5, 5) += Mfactor * mass * 0.5;
}

void ChElementBar::SetupInitial(ChSystem* system) {
    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
    this->mass = this->length * this->area * this->density;
}

void ChElementBar::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    assert((Fi.GetRows() == 6) && (Fi.GetColumns() == 1));

    ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    double L_dt = Vdot((nodes[1]->GetPos_dt() - nodes[0]->GetPos_dt()), dir);
    double Kstiffness = ((this->area * this->E) / this->length);
    double Rdamping = this->rdamping * Kstiffness;
    double internal_Kforce_local = Kstiffness * (L - L_ref);
    double internal_Rforce_local = Rdamping * L_dt;
    double internal_force_local = internal_Kforce_local + internal_Rforce_local;
    ChVector<> int_forceA = dir * internal_force_local;
    ChVector<> int_forceB = -dir * internal_force_local;
    Fi.PasteVector(int_forceA, 0, 0);
    Fi.PasteVector(int_forceB, 3, 0);
}

}  // end namespace fea
}  // end namespace chrono
