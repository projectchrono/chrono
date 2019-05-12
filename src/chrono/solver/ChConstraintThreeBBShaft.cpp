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

#include "chrono/solver/ChConstraintThreeBBShaft.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConstraintThreeBBShaft)

ChConstraintThreeBBShaft::ChConstraintThreeBBShaft(ChVariablesBody* mvariables_a,
                                                   ChVariablesBody* mvariables_b,
                                                   ChVariables* mvariables_c) {
    assert(mvariables_c->Get_ndof() == 1);
    SetVariables(mvariables_a, mvariables_b, mvariables_c);
}

ChConstraintThreeBBShaft::ChConstraintThreeBBShaft(const ChConstraintThreeBBShaft& other) : ChConstraintThree(other) {
    Cq_a = other.Cq_a;
    Cq_b = other.Cq_b;
    Cq_c = other.Cq_c;
    Eq_a = other.Eq_a;
    Eq_b = other.Eq_b;
    Eq_c = other.Eq_c;
}

ChConstraintThreeBBShaft& ChConstraintThreeBBShaft::operator=(const ChConstraintThreeBBShaft& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintThree::operator=(other);

    Cq_a = other.Cq_a;
    Cq_b = other.Cq_b;
    Cq_c = other.Cq_c;
    Eq_a = other.Eq_a;
    Eq_b = other.Eq_b;
    Eq_c = other.Eq_c;

    this->variables_a = other.variables_a;
    this->variables_b = other.variables_b;
    this->variables_c = other.variables_c;

    return *this;
}

void ChConstraintThreeBBShaft::SetVariables(ChVariables* mvariables_a,
                                            ChVariables* mvariables_b,
                                            ChVariables* mvariables_c) {
    assert(dynamic_cast<ChVariablesBody*>(mvariables_a));
    assert(dynamic_cast<ChVariablesBody*>(mvariables_b));
    assert(dynamic_cast<ChVariablesBody*>(mvariables_c));

    if (!mvariables_a || !mvariables_b || !mvariables_c) {
        SetValid(false);
        return;
    }

    SetValid(true);
    variables_a = mvariables_a;
    variables_b = mvariables_b;
    variables_c = mvariables_c;
    Cq_a.Reset();
    Cq_b.Reset();
    Cq_c.Reset();
}

void ChConstraintThreeBBShaft::Update_auxiliary() {
    // 1- Assuming jacobians are already computed, now compute
    //   the matrices [Eq_a]=[invM_a]*[Cq_a]'  etc
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
    if (variables_c->IsActive()) {
        ChMatrixNM<double, 1, 1> mtemp1;
        mtemp1.CopyFromMatrixT(Cq_c);
        variables_c->Compute_invMb_v(Eq_c, mtemp1);
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
    if (variables_c->IsActive()) {
        res.MatrMultiply(Cq_c, Eq_c);
        g_i += res(0, 0);
    }

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i)
        g_i += cfm_i;
}

double ChConstraintThreeBBShaft::Compute_Cq_q() {
    double ret = 0;

    if (variables_a->IsActive())
        for (int i = 0; i < 6; i++)
            ret += Cq_a.ElementN(i) * variables_a->Get_qb().ElementN(i);

    if (variables_b->IsActive())
        for (int i = 0; i < 6; i++)
            ret += Cq_b.ElementN(i) * variables_b->Get_qb().ElementN(i);

    if (variables_c->IsActive())
        ret += Cq_c.ElementN(0) * variables_c->Get_qb().ElementN(0);

    return ret;
}

void ChConstraintThreeBBShaft::Increment_q(const double deltal) {
    if (variables_a->IsActive())
        for (int i = 0; i < Eq_a.GetRows(); i++)
            variables_a->Get_qb()(i) += Eq_a.ElementN(i) * deltal;

    if (variables_b->IsActive())
        for (int i = 0; i < Eq_b.GetRows(); i++)
            variables_b->Get_qb()(i) += Eq_b.ElementN(i) * deltal;

    if (variables_c->IsActive())
        variables_c->Get_qb()(0) += Eq_c.ElementN(0) * deltal;
}

void ChConstraintThreeBBShaft::MultiplyAndAdd(double& result, const ChMatrix<double>& vect) const {
    if (variables_a->IsActive())
        for (int i = 0; i < Cq_a.GetRows(); i++)
            result += vect(variables_a->GetOffset() + i) * Cq_a.ElementN(i);

    if (variables_b->IsActive())
        for (int i = 0; i < Cq_b.GetRows(); i++)
            result += vect(variables_b->GetOffset() + i) * Cq_b.ElementN(i);

    if (variables_c->IsActive())
        result += vect(variables_c->GetOffset()) * Cq_c.ElementN(0);
}

void ChConstraintThreeBBShaft::MultiplyTandAdd(ChMatrix<double>& result, double l) {
    if (variables_a->IsActive())
        for (int i = 0; i < Cq_a.GetRows(); i++)
            result(variables_a->GetOffset() + i) += Cq_a.ElementN(i) * l;

    if (variables_b->IsActive())
        for (int i = 0; i < Cq_b.GetRows(); i++)
            result(variables_b->GetOffset() + i) += Cq_b.ElementN(i) * l;

    if (variables_c->IsActive())
        result(variables_c->GetOffset()) += Cq_c.ElementN(0) * l;
}

void ChConstraintThreeBBShaft::Build_Cq(ChSparseMatrix& storage, int insrow) {
    if (variables_a->IsActive())
        storage.PasteMatrix(Cq_a, insrow, variables_a->GetOffset());
    if (variables_b->IsActive())
        storage.PasteMatrix(Cq_b, insrow, variables_b->GetOffset());
    if (variables_c->IsActive())
        storage.PasteMatrix(Cq_c, insrow, variables_c->GetOffset());
}

void ChConstraintThreeBBShaft::Build_CqT(ChSparseMatrix& storage, int inscol) {
    if (variables_a->IsActive())
        storage.PasteTranspMatrix(Cq_a, variables_a->GetOffset(), inscol);
    if (variables_b->IsActive())
        storage.PasteTranspMatrix(Cq_b, variables_b->GetOffset(), inscol);
    if (variables_c->IsActive())
        storage.PasteTranspMatrix(Cq_c, variables_c->GetOffset(), inscol);
}

void ChConstraintThreeBBShaft::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConstraintThreeBBShaft>();

    // serialize the parent class data too
    ChConstraintThree::ArchiveOUT(marchive);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

void ChConstraintThreeBBShaft::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChConstraintThreeBBShaft>();

    // deserialize the parent class data too
    ChConstraintThree::ArchiveIN(marchive);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}



}  // end namespace chrono
