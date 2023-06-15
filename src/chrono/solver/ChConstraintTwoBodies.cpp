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
    Cq_a.setZero();
    Cq_b.setZero();
    Eq_a.setZero();
    Eq_b.setZero();
}

ChConstraintTwoBodies::ChConstraintTwoBodies(ChVariablesBody* mvariables_a, ChVariablesBody* mvariables_b) {
    Cq_a.setZero();
    Cq_b.setZero();
    Eq_a.setZero();
    Eq_b.setZero();
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
        variables_a->Compute_invMb_v(Eq_a, Cq_a.transpose());
    }
    if (variables_b->IsActive()) {
        variables_b->Compute_invMb_v(Eq_b, Cq_b.transpose());
    }

    // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    g_i = 0;
    if (variables_a->IsActive()) {
        g_i += Cq_a * Eq_a;
    }
    if (variables_b->IsActive()) {
        g_i += Cq_b * Eq_b;
    }

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i)
        g_i += cfm_i;
}

double ChConstraintTwoBodies::Compute_Cq_q() {
    double ret = 0;

    if (variables_a->IsActive()) {
        ret += Cq_a * variables_a->Get_qb();
    }

    if (variables_b->IsActive()) {
        ret += Cq_b * variables_b->Get_qb();
    }

    return ret;
}

void ChConstraintTwoBodies::Increment_q(const double deltal) {
    if (variables_a->IsActive()) {
        variables_a->Get_qb() += Eq_a * deltal;
    }

    if (variables_b->IsActive()) {
        variables_b->Get_qb() += Eq_b * deltal;
    }
}

void ChConstraintTwoBodies::MultiplyAndAdd(double& result, const ChVectorDynamic<double>& vect) const {
    if (variables_a->IsActive()) {
        result += Cq_a * vect.segment(variables_a->GetOffset(), 6);
    }

    if (variables_b->IsActive()) {
        result += Cq_b * vect.segment(variables_b->GetOffset(), 6);
    }
}

void ChConstraintTwoBodies::MultiplyTandAdd(ChVectorDynamic<double>& result, double l) {
    if (variables_a->IsActive()) {
        result.segment(variables_a->GetOffset(), 6) += Cq_a.transpose() * l;
    }

    if (variables_b->IsActive()) {
        result.segment(variables_b->GetOffset(), 6) += Cq_b.transpose() * l;
    }
}

void ChConstraintTwoBodies::Build_Cq(ChSparseMatrix& storage, int insrow) {
    if (variables_a->IsActive())
        PasteMatrix(storage, Cq_a, insrow, variables_a->GetOffset());
    if (variables_b->IsActive())
        PasteMatrix(storage, Cq_b, insrow, variables_b->GetOffset());
}

void ChConstraintTwoBodies::Build_CqT(ChSparseMatrix& storage, int inscol) {
    if (variables_a->IsActive())
        PasteMatrix(storage, Cq_a.transpose(), variables_a->GetOffset(), inscol);
    if (variables_b->IsActive())
        PasteMatrix(storage, Cq_b.transpose(), variables_b->GetOffset(), inscol);
}

void ChConstraintTwoBodies::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChConstraintTwoBodies>();

    // serialize the parent class data too
    ChConstraintTwo::ArchiveOut(marchive);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

void ChConstraintTwoBodies::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChConstraintTwoBodies>();

    // deserialize the parent class data too
    ChConstraintTwo::ArchiveIn(marchive);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

}  // end namespace chrono
