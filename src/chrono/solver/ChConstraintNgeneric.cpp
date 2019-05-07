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

#include "chrono/solver/ChConstraintNgeneric.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConstraintNgeneric)


ChConstraintNgeneric::ChConstraintNgeneric(const ChConstraintNgeneric& other) : ChConstraint(other) {
	this->variables = other.variables;
	this->Cq = other.Cq;
	this->Eq = other.Eq;
}


ChConstraintNgeneric& ChConstraintNgeneric::operator=(const ChConstraintNgeneric& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraint::operator=(other);

	this->variables = other.variables;
	this->Cq = other.Cq;
	this->Eq = other.Eq;

    return *this;
}

void ChConstraintNgeneric::SetVariables(std::vector<ChVariables*> mvars) {
    SetValid(true);

    this->variables = mvars;

    Cq.clear();
    Eq.clear();

    for (size_t i = 0; i < variables.size(); ++i) {
        if (!variables[i]) {
            SetValid(false);
            return;
        }

        Cq.push_back(ChMatrixDynamic<double>(1, variables[i]->Get_ndof()));
        Eq.push_back(ChMatrixDynamic<double>(variables[i]->Get_ndof(), 1));
    }

    for (size_t i = 0; i < variables.size(); ++i) {
        Cq[i].Reset();
    }
}

void ChConstraintNgeneric::Update_auxiliary() {
    // 1- Assuming jacobians are already computed, now compute
    //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]

	for (size_t i=0; i <  variables.size(); ++i) {
		if (variables[i]->IsActive())
			if (variables[i]->Get_ndof()) {
				ChMatrixDynamic<double> mtemp1(variables[i]->Get_ndof(), 1);
				mtemp1.CopyFromMatrixT(Cq[i]);
				variables[i]->Compute_invMb_v(Eq[i], mtemp1);
			}
	}

    // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    ChMatrixDynamic<double> res(1, 1);
    g_i = 0;
	for (size_t i=0; i < variables.size(); ++i) {
		if (variables[i]->IsActive())
			if (variables[i]->Get_ndof()) {
				res.MatrMultiply(Cq[i], Eq[i]);
				g_i += res(0, 0);
			}
	}

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i)
        g_i += cfm_i;
}

double ChConstraintNgeneric::Compute_Cq_q() {
    double ret = 0;

	for (size_t i=0; i <  variables.size(); ++i) {
		if (variables[i]->IsActive())
			for (int i = 0; i < Cq[i].GetColumns(); i++)
				ret += Cq[i].ElementN(i) * variables[i]->Get_qb().ElementN(i);
	}
    
    return ret;
}

void ChConstraintNgeneric::Increment_q(const double deltal) {

	for (size_t i=0; i <  variables.size(); ++i) {
		if (variables[i]->IsActive())
			for (int i = 0; i < Eq[i].GetRows(); i++)
				variables[i]->Get_qb()(i) += Eq[i].ElementN(i) * deltal;
	}
 
}

void ChConstraintNgeneric::MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {

	for (size_t i=0; i <  variables.size(); ++i) {
		if (variables[i]->IsActive())
			for (int i = 0; i < Cq[i].GetRows(); i++)
				result += vect(variables[i]->GetOffset() + i) * Cq[i].ElementN(i);
	}
}

void ChConstraintNgeneric::MultiplyTandAdd(ChMatrix<double>& result, double l) {
	for (size_t i=0; i <  variables.size(); ++i) {
		if (variables[i]->IsActive())
			for (int i = 0; i < Cq[i].GetRows(); i++)
				result(variables[i]->GetOffset() + i) += Cq[i].ElementN(i) * l;
	}
}

void ChConstraintNgeneric::Build_Cq(ChSparseMatrix& storage, int insrow) {
	for (size_t i=0; i <  variables.size(); ++i) {
		if (variables[i]->IsActive())
			storage.PasteMatrix(Cq[i], insrow, variables[i]->GetOffset());
	}
}

void ChConstraintNgeneric::Build_CqT(ChSparseMatrix& storage, int inscol) {
	for (size_t i=0; i <  variables.size(); ++i) {
		if (variables[i]->IsActive())
			storage.PasteTranspMatrix(Cq[i], variables[i]->GetOffset(), inscol);
	}
}

void ChConstraintNgeneric::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConstraintNgeneric>();

    // serialize the parent class data too
    ChConstraint::ArchiveOUT(marchive);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE 
}

void ChConstraintNgeneric::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChConstraintNgeneric>();

    // deserialize the parent class data too
    ChConstraint::ArchiveIN(marchive);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE 
}

}  // end namespace chrono
