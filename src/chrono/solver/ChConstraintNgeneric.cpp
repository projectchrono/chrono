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

        Cq.push_back(ChRowVectorDynamic<double>(variables[i]->GetDOF()));
        Eq.push_back(ChRowVectorDynamic<double>(variables[i]->GetDOF()));

        Cq.back().setZero();
    }
}

void ChConstraintNgeneric::UpdateAuxiliary() {
    // 1- Assuming jacobians are already computed, now compute
    //   the matrices [Eq_a]=[invM_a]*[Cq_a]' and [Eq_b]

    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive())
            if (variables[i]->GetDOF()) {
                variables[i]->ComputeMassInverseTimesVector(Eq[i], Cq[i].transpose());
            }
    }

    // 2- Compute g_i = [Cq_i]*[invM_i]*[Cq_i]' + cfm_i
    g_i = 0;
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive() && variables[i]->GetDOF() > 0) {
            g_i += Cq[i] * Eq[i];
        }
    }

    // 3- adds the constraint force mixing term (usually zero):
    if (cfm_i)
        g_i += cfm_i;
}

double ChConstraintNgeneric::ComputeJacobianTimesState() {
    double ret = 0;

    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            ret += Cq[i] * variables[i]->State();
        }
    }

    return ret;
}

void ChConstraintNgeneric::IncrementState(double deltal) {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            variables[i]->State() += Eq[i] * deltal;
        }
    }
}

void ChConstraintNgeneric::AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            result += Cq[i] * vect.segment(variables[i]->GetOffset(), Cq[i].size());
        }
    }
}

void ChConstraintNgeneric::AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive()) {
            result.segment(variables[i]->GetOffset(), Cq[i].size()) += Cq[i].transpose() * l;
        }
    }
}

void ChConstraintNgeneric::PasteJacobianInto(ChSparseMatrix& mat,
                                             unsigned int start_row,
                                             unsigned int start_col) const {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive())
            PasteMatrix(mat, Cq[i], start_row, variables[i]->GetOffset() + start_col);
    }
}

void ChConstraintNgeneric::PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                                       unsigned int start_row,
                                                       unsigned int start_col) const {
    for (size_t i = 0; i < variables.size(); ++i) {
        if (variables[i]->IsActive())
            PasteMatrix(mat, Cq[i].transpose(), variables[i]->GetOffset() + start_row, start_col);
    }
}

void ChConstraintNgeneric::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChConstraintNgeneric>();

    // serialize the parent class data too
    ChConstraint::ArchiveOut(archive_out);

    // serialize all member data:
    // NOTHING INTERESTING TO SERIALIZE
}

void ChConstraintNgeneric::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChConstraintNgeneric>();

    // deserialize the parent class data too
    ChConstraint::ArchiveIn(archive_in);

    // deserialize all member data:
    // NOTHING INTERESTING TO SERIALIZE
}

}  // end namespace chrono
