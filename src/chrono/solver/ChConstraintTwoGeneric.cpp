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

#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConstraintTwoGeneric)

ChConstraintTwoGeneric::ChConstraintTwoGeneric(ChVariables* mvariables_a, ChVariables* mvariables_b) {
    SetVariables(mvariables_a, mvariables_b);
}

ChConstraintTwoGeneric::ChConstraintTwoGeneric(const ChConstraintTwoGeneric& other) : ChConstraintTwo(other) {
    Cq_a = other.Cq_a;
    Cq_b = other.Cq_b;
    Eq_a = other.Eq_a;
    Eq_b = other.Eq_b;
}

ChConstraintTwoGeneric& ChConstraintTwoGeneric::operator=(const ChConstraintTwoGeneric& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintTwo::operator=(other);

    Cq_a = other.Cq_a;
    Cq_b = other.Cq_b;
    Eq_a = other.Eq_a;
    Eq_b = other.Eq_b;

    return *this;
}

void ChConstraintTwoGeneric::SetVariables(ChVariables* mvariables_a, ChVariables* mvariables_b) {
    if (!mvariables_a || !mvariables_b) {
        SetValid(false);
        return;
    }

    SetValid(true);
    variables_a = mvariables_a;
    variables_b = mvariables_b;

    if (variables_a->GetDOF() > 0) {
        Cq_a.resize(variables_a->GetDOF());
        Eq_a.resize(variables_a->GetDOF());
        Cq_a.setZero();
    }

    if (variables_b->GetDOF() > 0) {
        Cq_b.resize(variables_b->GetDOF());
        Eq_b.resize(variables_b->GetDOF());
        Cq_b.setZero();
    }
}

void ChConstraintTwoGeneric::UpdateAuxiliary() {
    // 1- Assuming jacobians are already computed, now compute
    //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]
    if (variables_a->IsActive() && variables_a->GetDOF() > 0) {
        variables_a->ComputeMassInverseTimesVector(Eq_a, Cq_a.transpose());
    }
    if (variables_b->IsActive() && variables_b->GetDOF() > 0) {
        variables_b->ComputeMassInverseTimesVector(Eq_b, Cq_b.transpose());
    }

    // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    g_i = 0;
    if (variables_a->IsActive() && variables_a->GetDOF() > 0) {
        g_i += Cq_a * Eq_a;
    }
    if (variables_b->IsActive() && variables_b->GetDOF() > 0) {
        g_i += Cq_b * Eq_b;
    }

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i != 0)
        g_i += cfm_i;
}

double ChConstraintTwoGeneric::ComputeJacobianTimesState() {
    double ret = 0;

    if (variables_a->IsActive()) {
        ret += Cq_a * variables_a->State();
    }

    if (variables_b->IsActive()) {
        ret += Cq_b * variables_b->State();
    }

    return ret;
}

void ChConstraintTwoGeneric::IncrementState(double deltal) {
    if (variables_a->IsActive()) {
        variables_a->State() += Eq_a * deltal;
    }

    if (variables_b->IsActive()) {
        variables_b->State() += Eq_b * deltal;
    }
}

void ChConstraintTwoGeneric::AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
    if (variables_a->IsActive()) {
        result += Cq_a * vect.segment(variables_a->GetOffset(), Cq_a.size());
    }

    if (variables_b->IsActive()) {
        result += Cq_b * vect.segment(variables_b->GetOffset(), Cq_b.size());
    }
}

void ChConstraintTwoGeneric::AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
    if (variables_a->IsActive()) {
        result.segment(variables_a->GetOffset(), Cq_a.size()) += Cq_a.transpose() * l;
    }

    if (variables_b->IsActive()) {
        result.segment(variables_b->GetOffset(), Cq_b.size()) += Cq_b.transpose() * l;
    }
}

void ChConstraintTwoGeneric::PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
    if (variables_a->IsActive())
        PasteMatrix(mat, Cq_a, start_row, variables_a->GetOffset() + start_col);
    if (variables_b->IsActive())
        PasteMatrix(mat, Cq_b, start_row, variables_b->GetOffset() + start_col);
}

void ChConstraintTwoGeneric::PasteJacobianTransposedInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) const {
    // Recall that Cq_a and Cq_b are column vectors.
    if (variables_a->IsActive())
        PasteMatrix(mat, Cq_a.transpose(), variables_a->GetOffset() + start_row, start_col);
    if (variables_b->IsActive())
        PasteMatrix(mat, Cq_b.transpose(), variables_b->GetOffset() + start_row, start_col);
}

void ChConstraintTwoGeneric::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChConstraintTwoGeneric>();

    // serialize the parent class data too
    ChConstraintTwo::ArchiveOut(archive_out);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

void ChConstraintTwoGeneric::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChConstraintTwoGeneric>();

    // deserialize the parent class data too
    ChConstraintTwo::ArchiveIn(archive_in);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
    // important to waste disk space.. they may be recomputed run-time,
    // and pointers to variables must be rebound in run-time.)
    // mstream << Cq_a;
    // mstream << Cq_b;
}

}  // end namespace chrono
