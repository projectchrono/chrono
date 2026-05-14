// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Handling of bilateral constraints for the system and Jacobian calculation
//
// =============================================================================

#include <algorithm>

#include "chrono_multicore/ChMulticoreDefines.h"
#include "chrono/multicore_math/thrust.h"

#include "chrono/solver/ChConstraintTwoBodies.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChConstraintThreeGeneric.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_multicore/constraints/ChConstraintBilateral.h"

using namespace chrono;

void ChConstraintBilateral::Build_b() {
    std::vector<ChConstraint*>& mconstraints = data_manager->system_descriptor->GetConstraints();

    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        ChConstraintTwoBodies* mbilateral = (ChConstraintTwoBodies*)(mconstraints[cntr]);
        data_manager->host_data.b[index + data_manager->num_unilaterals] = mbilateral->GetRightHandSide();
    }
}

void ChConstraintBilateral::Build_E() {
    data_manager->host_data.E.segment(data_manager->num_unilaterals, data_manager->num_bilaterals).setZero();
}

void ChConstraintBilateral::Build_D() {
    // Grab the list of all bilateral constraints present in the system
    // (note that this includes possibly inactive constraints)
    std::vector<ChConstraint*>& mconstraints = data_manager->system_descriptor->GetConstraints();

    // Loop over the active constraints and fill in the rows of the Jacobian,
    // taking into account the type of each constraint.
    // SparseMatrixType D_b_T = _DBT_;
    SparseMatrixType& D_b_T = data_manager->host_data.D_T;
    int off = data_manager->num_unilaterals;

    // #pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        int type = data_manager->host_data.bilateral_type[cntr];
        int row = off + index;

        switch (type) {
            case BilateralType::BODY_BODY: {
                ChConstraintTwoBodies* mbilateral = (ChConstraintTwoBodies*)(mconstraints[cntr]);

                int idA = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetIndex();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetIndex();
                int colA = idA * 6;
                int colB = idB * 6;

                D_b_T.coeffRef(row, colA + 0) = mbilateral->Get_Cq_a()(0);
                D_b_T.coeffRef(row, colA + 1) = mbilateral->Get_Cq_a()(1);
                D_b_T.coeffRef(row, colA + 2) = mbilateral->Get_Cq_a()(2);

                D_b_T.coeffRef(row, colA + 3) = mbilateral->Get_Cq_a()(3);
                D_b_T.coeffRef(row, colA + 4) = mbilateral->Get_Cq_a()(4);
                D_b_T.coeffRef(row, colA + 5) = mbilateral->Get_Cq_a()(5);

                D_b_T.coeffRef(row, colB + 0) = mbilateral->Get_Cq_b()(0);
                D_b_T.coeffRef(row, colB + 1) = mbilateral->Get_Cq_b()(1);
                D_b_T.coeffRef(row, colB + 2) = mbilateral->Get_Cq_b()(2);

                D_b_T.coeffRef(row, colB + 3) = mbilateral->Get_Cq_b()(3);
                D_b_T.coeffRef(row, colB + 4) = mbilateral->Get_Cq_b()(4);
                D_b_T.coeffRef(row, colB + 5) = mbilateral->Get_Cq_b()(5);
            } break;

            case BilateralType::SHAFT_SHAFT: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);

                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = data_manager->num_rigid_bodies * 6 + idB;

                D_b_T.coeffRef(row, colA) = mbilateral->Get_Cq_a()(0);
                D_b_T.coeffRef(row, colB) = mbilateral->Get_Cq_b()(0);
            } break;

            case BilateralType::SHAFT_BODY: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);

                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetIndex();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = idB * 6;

                D_b_T.coeffRef(row, colA) = mbilateral->Get_Cq_a()(0);

                D_b_T.coeffRef(row, colB + 0) = mbilateral->Get_Cq_b()(0);
                D_b_T.coeffRef(row, colB + 1) = mbilateral->Get_Cq_b()(1);
                D_b_T.coeffRef(row, colB + 2) = mbilateral->Get_Cq_b()(2);

                D_b_T.coeffRef(row, colB + 3) = mbilateral->Get_Cq_b()(3);
                D_b_T.coeffRef(row, colB + 4) = mbilateral->Get_Cq_b()(4);
                D_b_T.coeffRef(row, colB + 5) = mbilateral->Get_Cq_b()(5);
            } break;

            case BilateralType::SHAFT_SHAFT_SHAFT: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                int idC = ((ChVariablesShaft*)(mbilateral->GetVariables_c()))->GetShaft()->GetIndex();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = data_manager->num_rigid_bodies * 6 + idB;
                int colC = data_manager->num_rigid_bodies * 6 + idC;

                D_b_T.coeffRef(row, colA) = mbilateral->Get_Cq_a()(0);
                D_b_T.coeffRef(row, colB) = mbilateral->Get_Cq_b()(0);
                D_b_T.coeffRef(row, colC) = mbilateral->Get_Cq_c()(0);
            } break;

            case BilateralType::SHAFT_SHAFT_BODY: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                int idC = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_c()))->GetUserData())->GetIndex();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = data_manager->num_rigid_bodies * 6 + idB;
                int colC = idC * 6;

                D_b_T.coeffRef(row, colA) = mbilateral->Get_Cq_a()(0);
                D_b_T.coeffRef(row, colB) = mbilateral->Get_Cq_b()(0);

                D_b_T.coeffRef(row, colC + 0) = mbilateral->Get_Cq_c()(0);
                D_b_T.coeffRef(row, colC + 1) = mbilateral->Get_Cq_c()(1);
                D_b_T.coeffRef(row, colC + 2) = mbilateral->Get_Cq_c()(2);

                D_b_T.coeffRef(row, colC + 3) = mbilateral->Get_Cq_c()(3);
                D_b_T.coeffRef(row, colC + 4) = mbilateral->Get_Cq_c()(4);
                D_b_T.coeffRef(row, colC + 5) = mbilateral->Get_Cq_c()(5);
            } break;
        }
    }
}

void ChConstraintBilateral::GenerateSparsity() {
    // pre-insert zeros at each nonzero column position so subsequent Build_D()
    // coeffRef() calls hit pre-allocated slots rather than triggering internal
    // reallocations

    std::vector<ChConstraint*>& mconstraints = data_manager->system_descriptor->GetConstraints();

    SparseMatrixType& D_b_T = data_manager->host_data.D_T;
    const int rigid_dof_base = data_manager->num_rigid_bodies * 6;
    int off = data_manager->num_unilaterals;

    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        int type = data_manager->host_data.bilateral_type[cntr];
        int row = off + index;

        switch (type) {
            case BilateralType::BODY_BODY: {
                ChConstraintTwoBodies* mbilateral = (ChConstraintTwoBodies*)(mconstraints[cntr]);
                int idA = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetIndex();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetIndex();
                // eigen RowMajor insert() reqs ascending column order within each row.
                int col_lo = std::min(idA, idB) * 6;
                int col_hi = std::max(idA, idB) * 6;
                for (int k = 0; k < 6; ++k) D_b_T.insert(row, col_lo + k) = 0.0;
                for (int k = 0; k < 6; ++k) D_b_T.insert(row, col_hi + k) = 0.0;
            } break;

            case BilateralType::SHAFT_SHAFT: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                if (idA > idB) std::swap(idA, idB);
                D_b_T.insert(row, rigid_dof_base + idA) = 0.0;
                D_b_T.insert(row, rigid_dof_base + idB) = 0.0;
            } break;

            case BilateralType::SHAFT_BODY: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetIndex();
                int colB = idB * 6;
                for (int k = 0; k < 6; ++k) D_b_T.insert(row, colB + k) = 0.0;
                D_b_T.insert(row, rigid_dof_base + idA) = 0.0;
            } break;

            case BilateralType::SHAFT_SHAFT_SHAFT: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                std::vector<int> ids(3);
                ids[0] = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                ids[1] = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                ids[2] = ((ChVariablesShaft*)(mbilateral->GetVariables_c()))->GetShaft()->GetIndex();
                std::sort(ids.begin(), ids.end());
                D_b_T.insert(row, rigid_dof_base + ids[0]) = 0.0;
                D_b_T.insert(row, rigid_dof_base + ids[1]) = 0.0;
                D_b_T.insert(row, rigid_dof_base + ids[2]) = 0.0;
            } break;

            case BilateralType::SHAFT_SHAFT_BODY: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                int idC = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_c()))->GetUserData())->GetIndex();
                int colC = idC * 6;
                for (int k = 0; k < 6; ++k) D_b_T.insert(row, colC + k) = 0.0;
                if (idA > idB) std::swap(idA, idB);
                D_b_T.insert(row, rigid_dof_base + idA) = 0.0;
                D_b_T.insert(row, rigid_dof_base + idB) = 0.0;
            } break;
        }
    }
}
