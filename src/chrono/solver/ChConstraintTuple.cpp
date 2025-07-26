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

#include "chrono/solver/ChConstraintTuple.h"

namespace chrono {

ChConstraintTuple::ChConstraintTuple() {
    for (int i = 0; i < 3; i++) {
        variables[i] = nullptr;
    }
}

void ChConstraintTuple::UpdateAuxiliary(double& g_i) {
    CalculateEq();
    for (int i = 0; i < num_variables; i++) {
        if (variables[i]->IsActive()) {
            g_i += Cq(i) * Eq(i);
        }
    }
}

double ChConstraintTuple::ComputeJacobianTimesState() {
    double result = 0;
    for (int i = 0; i < num_variables; i++) {
        if (variables[i]->IsActive()) {
            result += Cq(i) * variables[i]->State();
        }
    }
    return result;
}

void ChConstraintTuple::IncrementState(double deltal) {
    for (int i = 0; i < num_variables; i++) {
        if (variables[i]->IsActive()) {
            variables[i]->State() += Eq(i) * deltal;
        }
    }
}

void ChConstraintTuple::AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) {
    for (int i = 0; i < num_variables; i++) {
        if (variables[i]->IsActive()) {
            result += Cq(i) * vect.segment(variables[i]->GetOffset(), variables[i]->GetDOF());
        }
    }
}

void ChConstraintTuple::AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) {
    for (int i = 0; i < num_variables; i++) {
        if (variables[i]->IsActive()) {
            result.segment(variables[i]->GetOffset(), variables[i]->GetDOF()) += Cq(i).transpose() * l;
        }
    }
}

void ChConstraintTuple::PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) {
    for (int i = 0; i < num_variables; i++) {
        if (variables[i]->IsActive()) {
            PasteMatrix(mat, Cq(i), start_row, variables[i]->GetOffset() + start_col);
        }
    }
}

void ChConstraintTuple::PasteJacobianTransposedInto(ChSparseMatrix& mat,
                                                    unsigned int start_row,
                                                    unsigned int start_col) {
    for (int i = 0; i < num_variables; i++) {
        if (variables[i]->IsActive()) {
            PasteMatrix(mat, Cq(i).transpose(), variables[i]->GetOffset() + start_row, start_col);
        }
    }
}

// -----------------------------------------------------------------------------

// Constraint tuple representing a collision object with 1 variable of size 1.
ChConstraintTuple_1::ChConstraintTuple_1(ChVariables* variables1) {
    assert(variables1->GetDOF() == 1);
    num_variables = 1;
    variables[0] = variables1;

    Cq_1.setZero();
    Eq_1.setZero();
}

ChRowVectorRef ChConstraintTuple_1::Cq(int i) {
    assert(i == 0);
    return Cq_1;
}

ChVectorRef ChConstraintTuple_1::Eq(int i) {
    assert(i == 0);
    return Eq_1;
}

void ChConstraintTuple_1::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
}

void ChConstraintTuple_1::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
}

double ChConstraintTuple_1::ComputeJacobianTimesState() {
    if (variables[0]->IsActive())
        return Cq_1 * variables[0]->State();
    return 0;
}

void ChConstraintTuple_1::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
}

// Constraint tuple representing a collision object with 1 variable of size 2.
ChConstraintTuple_2::ChConstraintTuple_2(ChVariables* variables1) {
    assert(variables1->GetDOF() == 2);
    num_variables = 1;
    variables[0] = variables1;

    Cq_1.setZero();
    Eq_1.setZero();
}

ChRowVectorRef ChConstraintTuple_2::Cq(int i) {
    assert(i == 0);
    return Cq_1;
}

ChVectorRef ChConstraintTuple_2::Eq(int i) {
    assert(i == 0);
    return Eq_1;
}

void ChConstraintTuple_2::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
}

void ChConstraintTuple_2::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
}

double ChConstraintTuple_2::ComputeJacobianTimesState() {
    if (variables[0]->IsActive())
        return Cq_1 * variables[0]->State();
    return 0;
}

void ChConstraintTuple_2::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
}

// Constraint tuple representing a collision object with 1 variable of size 3.
ChConstraintTuple_3::ChConstraintTuple_3(ChVariables* variables1) {
    assert(variables1->GetDOF() == 3);
    num_variables = 1;
    variables[0] = variables1;

    Cq_1.setZero();
    Eq_1.setZero();
}

ChRowVectorRef ChConstraintTuple_3::Cq(int i) {
    assert(i == 0);
    return Cq_1;
}

ChVectorRef ChConstraintTuple_3::Eq(int i) {
    assert(i == 0);
    return Eq_1;
}

void ChConstraintTuple_3::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
}

void ChConstraintTuple_3::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
}

double ChConstraintTuple_3::ComputeJacobianTimesState() {
    if (variables[0]->IsActive())
        return Cq_1 * variables[0]->State();
    return 0;
}

void ChConstraintTuple_3::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
}

// Constraint tuple representing a collision object with 1 variable of size 4.
ChConstraintTuple_4::ChConstraintTuple_4(ChVariables* variables1) {
    assert(variables1->GetDOF() == 4);
    num_variables = 1;
    variables[0] = variables1;

    Cq_1.setZero();
    Eq_1.setZero();
}

ChRowVectorRef ChConstraintTuple_4::Cq(int i) {
    assert(i == 0);
    return Cq_1;
}

ChVectorRef ChConstraintTuple_4::Eq(int i) {
    assert(i == 0);
    return Eq_1;
}

void ChConstraintTuple_4::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
}

void ChConstraintTuple_4::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
}

double ChConstraintTuple_4::ComputeJacobianTimesState() {
    if (variables[0]->IsActive())
        return Cq_1 * variables[0]->State();
    return 0;
}

void ChConstraintTuple_4::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
}

// Constraint tuple representing a collision object with 1 variable of size 5.
ChConstraintTuple_5::ChConstraintTuple_5(ChVariables* variables1) {
    assert(variables1->GetDOF() == 5);
    num_variables = 1;
    variables[0] = variables1;

    Cq_1.setZero();
    Eq_1.setZero();
}

ChRowVectorRef ChConstraintTuple_5::Cq(int i) {
    assert(i == 0);
    return Cq_1;
}

ChVectorRef ChConstraintTuple_5::Eq(int i) {
    assert(i == 0);
    return Eq_1;
}

void ChConstraintTuple_5::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
}

void ChConstraintTuple_5::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
}

double ChConstraintTuple_5::ComputeJacobianTimesState() {
    if (variables[0]->IsActive())
        return Cq_1 * variables[0]->State();
    return 0;
}

void ChConstraintTuple_5::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
}

// Constraint tuple representing a collision object with 1 variable of size 6.
ChConstraintTuple_6::ChConstraintTuple_6(ChVariables* variables1) {
    assert(variables1->GetDOF() == 6);
    num_variables = 1;
    variables[0] = variables1;

    Cq_1.setZero();
    Eq_1.setZero();
}

ChRowVectorRef ChConstraintTuple_6::Cq(int i) {
    assert(i == 0);
    return Cq_1;
}

ChVectorRef ChConstraintTuple_6::Eq(int i) {
    assert(i == 0);
    return Eq_1;
}

void ChConstraintTuple_6::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
}

void ChConstraintTuple_6::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
}

double ChConstraintTuple_6::ComputeJacobianTimesState() {
    if (variables[0]->IsActive())
        return Cq_1 * variables[0]->State();
    return 0;
}

void ChConstraintTuple_6::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
}

// -----------------------------------------------------------------------------

// Constraint tuple representing a collision object with 2 variable, each of size 3.
ChConstraintTuple_33::ChConstraintTuple_33(ChVariables* variables1, ChVariables* variables2) {
    assert(variables1->GetDOF() == 3);
    assert(variables2->GetDOF() == 3);
    num_variables = 2;
    variables[0] = variables1;
    variables[1] = variables2;

    Cq_1.setZero();
    Cq_2.setZero();
    Eq_1.setZero();
    Eq_2.setZero();
}

ChRowVectorRef ChConstraintTuple_33::Cq(int i) {
    switch (i) {
        case 0: {
            return Cq_1;
        }
        case 1: {
            return Cq_2;
        }
        default:
            assert(false);
    }
}

ChVectorRef ChConstraintTuple_33::Eq(int i) {
    switch (i) {
        case 0: {
            return Eq_1;
        }
        case 1: {
            return Eq_2;
        }
        default:
            assert(false);
    }
}

void ChConstraintTuple_33::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
    variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
}

void ChConstraintTuple_33::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
    if (variables[1]->IsActive()) {
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        g_i += Cq_2 * Eq_2;
    }
}

double ChConstraintTuple_33::ComputeJacobianTimesState() {
    double result = 0;
    if (variables[0]->IsActive())
        result += Cq_1 * variables[0]->State();
    if (variables[1]->IsActive())
        result += Cq_2 * variables[1]->State();
    return result;
}

void ChConstraintTuple_33::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
    if (variables[1]->IsActive())
        variables[1]->State() += Eq_2 * deltal;
}

// Constraint tuple representing a collision object with 2 variable, each of size 6.
ChConstraintTuple_66::ChConstraintTuple_66(ChVariables* variables1, ChVariables* variables2) {
    assert(variables1->GetDOF() == 6);
    assert(variables2->GetDOF() == 6);
    num_variables = 2;
    variables[0] = variables1;
    variables[1] = variables2;

    Cq_1.setZero();
    Cq_2.setZero();
    Eq_1.setZero();
    Eq_2.setZero();
}

ChRowVectorRef ChConstraintTuple_66::Cq(int i) {
    switch (i) {
        case 0: {
            return Cq_1;
        }
        case 1: {
            return Cq_2;
        }
        default:
            assert(false);
    }
}

ChVectorRef ChConstraintTuple_66::Eq(int i) {
    switch (i) {
        case 0: {
            return Eq_1;
        }
        case 1: {
            return Eq_2;
        }
        default:
            assert(false);
    }
}

void ChConstraintTuple_66::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
    variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
}

void ChConstraintTuple_66::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
    if (variables[1]->IsActive()) {
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        g_i += Cq_2 * Eq_2;
    }
}

double ChConstraintTuple_66::ComputeJacobianTimesState() {
    double result = 0;
    if (variables[0]->IsActive())
        result += Cq_1 * variables[0]->State();
    if (variables[1]->IsActive())
        result += Cq_2 * variables[1]->State();
    return result;
}

void ChConstraintTuple_66::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
    if (variables[1]->IsActive())
        variables[1]->State() += Eq_2 * deltal;
}

// -----------------------------------------------------------------------------

// Constraint tuple representing a collision object with 3 variable, each of size 3.
ChConstraintTuple_333::ChConstraintTuple_333(ChVariables* variables1,
                                             ChVariables* variables2,
                                             ChVariables* variables3) {
    assert(variables1->GetDOF() == 3);
    assert(variables2->GetDOF() == 3);
    assert(variables3->GetDOF() == 3);
    num_variables = 3;
    variables[0] = variables1;
    variables[1] = variables2;
    variables[2] = variables3;

    Cq_1.setZero();
    Cq_2.setZero();
    Cq_3.setZero();
    Eq_1.setZero();
    Eq_2.setZero();
    Eq_3.setZero();
}

ChRowVectorRef ChConstraintTuple_333::Cq(int i) {
    switch (i) {
        case 0:
            return Cq_1;
        case 1:
            return Cq_2;
        case 2:
            return Cq_3;
    }
}

ChVectorRef ChConstraintTuple_333::Eq(int i) {
    switch (i) {
        case 0:
            return Eq_1;
        case 1:
            return Eq_2;
        case 2:
            return Eq_3;
    }
}

void ChConstraintTuple_333::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
    variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
    variables[2]->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
}

void ChConstraintTuple_333::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
    if (variables[1]->IsActive()) {
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        g_i += Cq_2 * Eq_2;
    }
    if (variables[2]->IsActive()) {
        variables[2]->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
        g_i += Cq_3 * Eq_3;
    }
}

double ChConstraintTuple_333::ComputeJacobianTimesState() {
    double result = 0;
    if (variables[0]->IsActive())
        result += Cq_1 * variables[0]->State();
    if (variables[1]->IsActive())
        result += Cq_2 * variables[1]->State();
    if (variables[2]->IsActive())
        result += Cq_3 * variables[2]->State();
    return result;
}

void ChConstraintTuple_333::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
    if (variables[1]->IsActive())
        variables[1]->State() += Eq_2 * deltal;
    if (variables[2]->IsActive())
        variables[2]->State() += Eq_3 * deltal;
}

// Constraint tuple representing a collision object with 3 variable, each of size 6.
ChConstraintTuple_666::ChConstraintTuple_666(ChVariables* variables1,
                                             ChVariables* variables2,
                                             ChVariables* variables3) {
    assert(variables1->GetDOF() == 6);
    assert(variables2->GetDOF() == 6);
    assert(variables3->GetDOF() == 6);
    num_variables = 3;
    variables[0] = variables1;
    variables[1] = variables2;
    variables[2] = variables3;

    Cq_1.setZero();
    Cq_2.setZero();
    Cq_3.setZero();
    Eq_1.setZero();
    Eq_2.setZero();
    Eq_3.setZero();
}

ChRowVectorRef ChConstraintTuple_666::Cq(int i) {
    switch (i) {
        case 0:
            return Cq_1;
        case 1:
            return Cq_2;
        case 2:
            return Cq_3;
    }
}

ChVectorRef ChConstraintTuple_666::Eq(int i) {
    switch (i) {
        case 0:
            return Eq_1;
        case 1:
            return Eq_2;
        case 2:
            return Eq_3;
    }
}

void ChConstraintTuple_666::CalculateEq() {
    variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
    variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
    variables[2]->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
}

void ChConstraintTuple_666::UpdateAuxiliary(double& g_i) {
    if (variables[0]->IsActive()) {
        variables[0]->ComputeMassInverseTimesVector(Eq_1, Cq_1.transpose());
        g_i += Cq_1 * Eq_1;
    }
    if (variables[1]->IsActive()) {
        variables[1]->ComputeMassInverseTimesVector(Eq_2, Cq_2.transpose());
        g_i += Cq_2 * Eq_2;
    }
    if (variables[2]->IsActive()) {
        variables[2]->ComputeMassInverseTimesVector(Eq_3, Cq_3.transpose());
        g_i += Cq_3 * Eq_3;
    }
}

double ChConstraintTuple_666::ComputeJacobianTimesState() {
    double result = 0;
    if (variables[0]->IsActive())
        result += Cq_1 * variables[0]->State();
    if (variables[1]->IsActive())
        result += Cq_2 * variables[1]->State();
    if (variables[2]->IsActive())
        result += Cq_3 * variables[2]->State();
    return result;
}

void ChConstraintTuple_666::IncrementState(double deltal) {
    if (variables[0]->IsActive())
        variables[0]->State() += Eq_1 * deltal;
    if (variables[1]->IsActive())
        variables[1]->State() += Eq_2 * deltal;
    if (variables[2]->IsActive())
        variables[2]->State() += Eq_3 * deltal;
}

}  // end namespace chrono
