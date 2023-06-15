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

#include "chrono/solver/ChConstraintNgeneric.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConstraintNgeneric)

ChConstraintNgeneric::ChConstraintNgeneric(const ChConstraintNgeneric& other) : ChConstraint(other) {
    variables = other.variables;
    Cq = other.Cq;
    Eq = other.Eq;
}

ChConstraintNgeneric& ChConstraintNgeneric::operator=(const ChConstraintNgeneric& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraint::operator=(other);

    variables = other.variables;
    Cq = other.Cq;
    Eq = other.Eq;

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

        Cq.push_back(ChRowVectorDynamic<double>(variables[i]->Get_ndof()));
        Eq.push_back(ChRowVectorDynamic<double>(variables[i]->Get_ndof()));

        Cq.back().setZero();
    }
}

void ChConstraintNgeneric::Update_auxiliary() {
    // 1- Assuming jacobians are already computed, now compute
    //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]

    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive())
            if (variables[i]->Get_ndof()) {
                variables[i]->Compute_invMb_v(Eq[i], Cq[i].transpose());
            }
    }

    // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    g_i = 0;
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive() && variables[i]->Get_ndof() > 0) {
            g_i += Cq[i] * Eq[i];
        }
    }

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i)
        g_i += cfm_i;
}

double ChConstraintNgeneric::Compute_Cq_q() {
    double ret = 0;

    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            ret += Cq[i] * variables[i]->Get_qb();
        }
    }

    return ret;
}

void ChConstraintNgeneric::Increment_q(const double deltal) {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            variables[i]->Get_qb() += Eq[i] * deltal;
        }
    }
}

void ChConstraintNgeneric::MultiplyAndAdd(double& result, const ChVectorDynamic<double>& vect) const {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            result += Cq[i] * vect.segment(variables[i]->GetOffset(), Cq[i].size());
        }
    }
}

void ChConstraintNgeneric::MultiplyTandAdd(ChVectorDynamic<double>& result, double l) {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            result.segment(variables[i]->GetOffset(), Cq[i].size()) += Cq[i].transpose() * l;
        }
    }
}

void ChConstraintNgeneric::Build_Cq(ChSparseMatrix& storage, int insrow) {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive())
            PasteMatrix(storage, Cq[i], insrow, variables[i]->GetOffset());
    }
}

void ChConstraintNgeneric::Build_CqT(ChSparseMatrix& storage, int inscol) {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive())
            PasteMatrix(storage, Cq[i].transpose(), variables[i]->GetOffset(), inscol);
    }
}

void ChConstraintNgeneric::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConstraintNgeneric>();

    // serialize the parent class data too
    ChConstraint::ArchiveOut(marchive);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE
}

void ChConstraintNgeneric::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChConstraintNgeneric>();

    // deserialize the parent class data too
    ChConstraint::ArchiveIn(marchive);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE
}

}  // end namespace chrono
