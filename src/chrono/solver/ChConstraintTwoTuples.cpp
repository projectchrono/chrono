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

#include "chrono/solver/ChConstraintTwoTuples.h"

namespace chrono {

ChConstraintTwoTuples::ChConstraintTwoTuples() : tuple_a(nullptr), tuple_b(nullptr) {}

ChConstraintTwoTuples::ChConstraintTwoTuples(const ChConstraintTwoTuples& other)
    : ChConstraint(other), tuple_a(other.tuple_a), tuple_b(other.tuple_b) {}

ChConstraintTwoTuples::~ChConstraintTwoTuples() {
    delete tuple_a;
    delete tuple_b;
}

ChConstraintTwoTuples& ChConstraintTwoTuples::operator=(const ChConstraintTwoTuples& other) {
    tuple_a = other.tuple_a;
    tuple_b = other.tuple_b;
    return *this;
}

void ChConstraintTwoTuples::SetTuples(ChConstraintTuple* tupleA, ChConstraintTuple* tupleB) {
    delete tuple_a;
    delete tuple_b;
    tuple_a = tupleA;
    tuple_b = tupleB;
}

void ChConstraintTwoTuples::SetTuplesFromContactables(ChContactable* objA, ChContactable* objB) {
    delete tuple_a;
    delete tuple_b;
    tuple_a = objA->CreateConstraintTuple();
    tuple_b = objB->CreateConstraintTuple();
}

void ChConstraintTwoTuples::UpdateAuxiliary() {
    g_i = 0;
    tuple_a->UpdateAuxiliary(g_i);
    tuple_b->UpdateAuxiliary(g_i);
    //  adds the constraint force mixing term (usually zero):
    if (cfm_i != 0)
        g_i += cfm_i;
}

double ChConstraintTwoTuples::ComputeJacobianTimesState() {
    double ret = 0;
    ret += tuple_a->ComputeJacobianTimesState();
    ret += tuple_b->ComputeJacobianTimesState();
    return ret;
}

void ChConstraintTwoTuples::IncrementState(double deltal) {
    tuple_a->IncrementState(deltal);
    tuple_b->IncrementState(deltal);
}

void ChConstraintTwoTuples::AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) const {
    tuple_a->AddJacobianTimesVectorInto(result, vect);
    tuple_b->AddJacobianTimesVectorInto(result, vect);
}

void ChConstraintTwoTuples::AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) const {
    tuple_a->AddJacobianTransposedTimesScalarInto(result, l);
    tuple_b->AddJacobianTransposedTimesScalarInto(result, l);
}

void ChConstraintTwoTuples::PasteJacobianInto(ChSparseMatrix& mat,
                                              unsigned int start_row,
                                              unsigned int start_col) const {
    tuple_a->PasteJacobianInto(mat, start_row, start_col);
    tuple_b->PasteJacobianInto(mat, start_row, start_col);
}

void ChConstraintTwoTuples::PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                                        unsigned int start_row,
                                                        unsigned int start_col) const {
    tuple_a->PasteJacobianTransposedInto(mat, start_row, start_col);
    tuple_b->PasteJacobianTransposedInto(mat, start_row, start_col);
}

}  // end namespace chrono
