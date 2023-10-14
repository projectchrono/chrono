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

#include "chrono/fea/ChElementBar.h"

namespace chrono {
namespace fea {

ChElementBar::ChElementBar() : area(0.01 * 0.01), density(1000), E(0.01e9), rdamping(0.01), mass(0), length(0) {
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

void ChElementBar::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.setZero(this->GetNdofs());
    mD.segment(0, 3) = this->nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = this->nodes[1]->GetPos().eigen();
}

// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass
void ChElementBar::ComputeNodalMass() {
    for (int i = 0; i < nodes.size(); ++i)
        nodes[i]->m_TotalMass += this->mass / nodes.size();
}

void ChElementBar::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 6) && (H.cols() == 6));

    // For K stiffness matrix and R damping matrix:
    // compute stiffness matrix (this is already the explicit
    // formulation of the corotational stiffness matrix in 3D)

    ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    ChVectorN<double, 3> dircolumn = dir.eigen();

    double Kstiffness = ((this->area * this->E) / this->length);
    double Rdamping = this->rdamping * Kstiffness;
    // note that stiffness and damping matrices are the same, so join stuff here
    double commonfactor = Kstiffness * Kfactor + Rdamping * Rfactor;
	ChMatrix33<> V = dircolumn * dircolumn.transpose();
    ChMatrix33<> keV = commonfactor * V;

    H.block(0,0,3,3) = keV;
    H.block(3,3,3,3) = keV;
    H.block(0,3,3,3) = -keV;
    H.block(3,0,3,3) = -keV;

	// add geometric stiffness - in future it might become an option to switch off if not needed.
	// See for ex. http://shodhbhagirathi.iitr.ac.in:8081/jspui/handle/123456789/8433 pag. 14-15
	if (true) {
		double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
		double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
		double Kstiffness1 = ((this->area * this->E) / this->length);
		double internal_Kforce_local = Kstiffness1 * (L - L_ref);

		ChMatrix33<> kgV = Kfactor * (internal_Kforce_local / L_ref) * (ChMatrix33<>(1) - V);
		H.block(0, 0, 3, 3) += kgV;
		H.block(3, 3, 3, 3) += kgV;
		H.block(0, 3, 3, 3) += -kgV;
		H.block(3, 0, 3, 3) += -kgV;
	}

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

void ChElementBar::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 6);

	ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
	double internal_force_local = GetCurrentForce();
    ChVector<> int_forceA = dir * internal_force_local;
    ChVector<> int_forceB = -dir * internal_force_local;

    Fi.segment(0, 3) = int_forceA.eigen();
    Fi.segment(3, 3) = int_forceB.eigen();
}

double ChElementBar::GetCurrentForce() {
	ChVector<> dir = (nodes[1]->GetPos() - nodes[0]->GetPos()).GetNormalized();
    double L_ref = (nodes[1]->GetX0() - nodes[0]->GetX0()).Length();
    double L = (nodes[1]->GetPos() - nodes[0]->GetPos()).Length();
    double L_dt = Vdot((nodes[1]->GetPos_dt() - nodes[0]->GetPos_dt()), dir);
    double Kstiffness = ((this->area * this->E) / this->length);
    double Rdamping = this->rdamping * Kstiffness;
    double internal_Kforce_local = Kstiffness * (L - L_ref);
    double internal_Rforce_local = Rdamping * L_dt;
    return internal_Kforce_local + internal_Rforce_local;
}


}  // end namespace fea
}  // end namespace chrono
