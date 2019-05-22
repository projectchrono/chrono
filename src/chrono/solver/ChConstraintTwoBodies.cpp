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

#include "chrono/solver/ChConstraintTwoBodies.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConstraintTwoBodies)

ChConstraintTwoBodies::ChConstraintTwoBodies() {
    Cq_a.Reset();
    Cq_b.Reset();
    Eq_a.Reset();
    Eq_b.Reset();
}

ChConstraintTwoBodies::ChConstraintTwoBodies(ChVariablesBody* mvariables_a, ChVariablesBody* mvariables_b) {
    Cq_a.Reset();
    Cq_b.Reset();
    Eq_a.Reset();
    Eq_b.Reset();
    SetVariables(mvariables_a, mvariables_b);
}

ChConstraintTwoBodies::ChConstraintTwoBodies(const ChConstraintTwoBodies& other) : ChConstraintTwo(other) {
    Cq_a = other.Cq_a;
    Cq_b = other.Cq_b;
    Eq_a = other.Eq_a;
    Eq_b = other.Eq_b;
}

ChConstraintTwoBodies& ChConstraintTwoBodies::operator=(const ChConstraintTwoBodies& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintTwo::operator=(other);

    Cq_a = other.Cq_a;
    Cq_b = other.Cq_b;
    Eq_a = other.Eq_a;
    Eq_b = other.Eq_b;

    this->variables_a = other.variables_a;
    this->variables_b = other.variables_b;

    return *this;
}

void ChConstraintTwoBodies::SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b) {
    assert(dynamic_cast<ChVariablesBody*>(mvariables_a));
    assert(dynamic_cast<ChVariablesBody*>(mvariables_b));

    if (!mvariables_a || !mvariables_b) {
        SetValid(false);
        return;
    }

    SetValid(true);
    variables_a = mvariables_a;
    variables_b = mvariables_b;
}

void ChConstraintTwoBodies::Update_auxiliary() {
    // 1- Assuming jacobians are already computed, now compute
    //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
    if (variables_a->IsActive()) {
        ChMatrixNM<double, 6, 1> mtemp1;
        mtemp1.CopyFromMatrixT(Cq_a);
        variables_a->Compute_invMb_v(Eq_a, mtemp1);
    }
    if (variables_b->IsActive()) {
        ChMatrixNM<double, 6, 1> mtemp1;
        mtemp1.CopyFromMatrixT(Cq_b);
        variables_b->Compute_invMb_v(Eq_b, mtemp1);
    }

    // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    ChMatrixNM<double, 1, 1> res;
    g_i = 0;
    if (variables_a->IsActive()) {
        res.MatrMultiply(Cq_a, Eq_a);
        g_i = res(0, 0);
    }
    if (variables_b->IsActive()) {
        res.MatrMultiply(Cq_b, Eq_b);
        g_i += res(0, 0);
    }

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i)
        g_i += cfm_i;
}

double ChConstraintTwoBodies::Compute_Cq_q() {
    double ret = 0;

    if (variables_a->IsActive())
        for (int i = 0; i < 6; i++)
            ret += Cq_a.ElementN(i) * variables_a->Get_qb().ElementN(i);

    if (variables_b->IsActive())
        for (int i = 0; i < 6; i++)
            ret += Cq_b.ElementN(i) * variables_b->Get_qb().ElementN(i);

    return ret;
}

void ChConstraintTwoBodies::Increment_q(const double deltal) {
    if (variables_a->IsActive())
        for (int i = 0; i < 6; i++)
            variables_a->Get_qb()(i) += Eq_a.ElementN(i) * deltal;

    if (variables_b->IsActive())
        for (int i = 0; i < 6; i++)
            variables_b->Get_qb()(i) += Eq_b.ElementN(i) * deltal;
}

void ChConstraintTwoBodies::MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {
    int off_a = variables_a->GetOffset();
    int off_b = variables_b->GetOffset();

    if (variables_a->IsActive())
        for (int i = 0; i < 6; i++)
            result += vect(off_a + i) * Cq_a.ElementN(i);

    if (variables_b->IsActive())
        for (int i = 0; i < 6; i++)
            result += vect(off_b + i) * Cq_b.ElementN(i);
}

void ChConstraintTwoBodies::MultiplyTandAdd(ChMatrix<double>& result, double l) {
    int off_a = variables_a->GetOffset();
    int off_b = variables_b->GetOffset();

    if (variables_a->IsActive())
        for (int i = 0; i < 6; i++)
            result(off_a + i) += Cq_a.ElementN(i) * l;

    if (variables_b->IsActive())
        for (int i = 0; i < 6; i++)
            result(off_b + i) += Cq_b.ElementN(i) * l;
}

void ChConstraintTwoBodies::Build_Cq(ChSparseMatrix& storage, int insrow) {
    if (variables_a->IsActive())
        storage.PasteMatrix(Cq_a, insrow, variables_a->GetOffset());
    if (variables_b->IsActive())
        storage.PasteMatrix(Cq_b, insrow, variables_b->GetOffset());
}

void ChConstraintTwoBodies::Build_CqT(ChSparseMatrix& storage, int inscol) {
    if (variables_a->IsActive())
        storage.PasteTranspMatrix(Cq_a, variables_a->GetOffset(), inscol);
    if (variables_b->IsActive())
        storage.PasteTranspMatrix(Cq_b, variables_b->GetOffset(), inscol);
}

void ChConstraintTwoBodies::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConstraintTwoBodies>();

    // serialize the parent class data too
    ChConstraintTwo::ArchiveOUT(marchive);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

void ChConstraintTwoBodies::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChConstraintTwoBodies>();

    // deserialize the parent class data too
    ChConstraintTwo::ArchiveIN(marchive);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

}  // end namespace chrono
