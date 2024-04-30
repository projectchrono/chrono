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

void ChElementSpring::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNumCoordsPosLevel());
    mD.segment(0, 3) = this->nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = this->nodes[1]->GetPos().eigen();
}

void ChElementSpring::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 6) && (H.cols() == 6));

    // compute stiffness matrix (this is already the explicit
    // formulation of the corotational stiffness matrix in 3D)
    ChVector3d dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    ChVectorN<double, 3> dircolumn = dir.eigen();

    // note that stiffness and damping matrices are the same, so join stuff here
    double commonfactor = this->spring_k * Kfactor + this->damper_r * Rfactor;
    ChMatrix33<> V = dircolumn * dircolumn.transpose();
    ChMatrix33<> keV = commonfactor * V;

    H.block(0, 0, 3, 3) = keV;
    H.block(3, 3, 3, 3) = keV;
    H.block(0, 3, 3, 3) = -keV;
    H.block(3, 0, 3, 3) = -keV;

    // add geometric stiffness - in future it might become an option to switch off if not needed.
    // See for ex. http://shodhbhagirathi.iitr.ac.in:8081/jspui/handle/123456789/8433 pag. 14-15
    if (true) {
        double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
        double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
        double internal_Kforce_local = this->spring_k * (L - L_ref);

        ChMatrix33<> kgV = Kfactor * (internal_Kforce_local / L_ref) * (ChMatrix33<>(1) - V);
        H.block(0, 0, 3, 3) += kgV;
        H.block(3, 3, 3, 3) += kgV;
        H.block(0, 3, 3, 3) += -kgV;
        H.block(3, 0, 3, 3) += -kgV;
    }

    // finally, do nothing about mass matrix because this element is mass-less
}

void ChElementSpring::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 6);

    ChVector3d dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    double L_dt = Vdot((nodes[1]->GetPosDt() - nodes[0]->GetPosDt()), dir);
    double internal_Kforce_local = this->spring_k * (L - L_ref);
    double internal_Rforce_local = this->damper_r * L_dt;
    double internal_force_local = internal_Kforce_local + internal_Rforce_local;
    ChVector3d int_forceA = dir * internal_force_local;
    ChVector3d int_forceB = -dir * internal_force_local;
    Fi.segment(0, 3) = int_forceA.eigen();
    Fi.segment(3, 3) = int_forceB.eigen();
}

double ChElementSpring::GetCurrentForce() {
    ChVector3d dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    double L_dt = Vdot((nodes[1]->GetPosDt() - nodes[0]->GetPosDt()), dir);
    double internal_Kforce_local = this->spring_k * (L - L_ref);
    double internal_Rforce_local = this->damper_r * L_dt;
    return internal_Kforce_local + internal_Rforce_local;
}

}  // end namespace fea
}  // end namespace chrono
