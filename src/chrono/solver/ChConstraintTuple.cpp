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

//ChConstraintTuple::ChConstraintTuple() {
//    for (int i = 0; i < 3; i++) {
//        variables[i] = nullptr;
//    }
//}

//void ChConstraintTuple::UpdateAuxiliary(double& g_i) {
//    CalculateEq();
//    for (int i = 0; i < num_variables; i++) {
//        if (variables[i]->IsActive()) {
//            g_i += Cq(i) * Eq(i);
//        }
//    }
//}

//double ChConstraintTuple::ComputeJacobianTimesState() {
//    double result = 0;
//    for (int i = 0; i < num_variables; i++) {
//        if (variables[i]->IsActive()) {
//            result += Cq(i) * variables[i]->State();
//        }
//    }
//    return result;
//}

//void ChConstraintTuple::IncrementState(double deltal) {
//    for (int i = 0; i < num_variables; i++) {
//        if (variables[i]->IsActive()) {
//            variables[i]->State() += Eq(i) * deltal;
//        }
//    }
//}

//void ChConstraintTuple::AddJacobianTimesVectorInto(double& result, ChVectorConstRef vect) {
//    for (int i = 0; i < num_variables; i++) {
//        if (variables[i]->IsActive()) {
//            result += Cq(i) * vect.segment(variables[i]->GetOffset(), variables[i]->GetDOF());
//        }
//    }
//}

//void ChConstraintTuple::AddJacobianTransposedTimesScalarInto(ChVectorRef result, double l) {
//    for (int i = 0; i < num_variables; i++) {
//        if (variables[i]->IsActive()) {
//            result.segment(variables[i]->GetOffset(), variables[i]->GetDOF()) += Cq(i).transpose() * l;
//        }
//    }
//}

//void ChConstraintTuple::PasteJacobianInto(ChSparseMatrix& mat, unsigned int start_row, unsigned int start_col) {
//    for (int i = 0; i < num_variables; i++) {
//        if (variables[i]->IsActive()) {
//            PasteMatrix(mat, Cq(i), start_row, variables[i]->GetOffset() + start_col);
//        }
//    }
//}

//void ChConstraintTuple::PasteJacobianTransposedInto(ChSparseMatrix& mat,
//                                                    unsigned int start_row,
//                                                    unsigned int start_col) {
//    for (int i = 0; i < num_variables; i++) {
//        if (variables[i]->IsActive()) {
//            PasteMatrix(mat, Cq(i).transpose(), variables[i]->GetOffset() + start_row, start_col);
//        }
//    }
//}

}  // end namespace chrono
