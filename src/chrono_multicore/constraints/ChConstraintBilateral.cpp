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
    SparseMatrixType& D_b_T = data_manager->host_data.D_T;
    int off = data_manager->num_unilaterals;

    double* vals = D_b_T.valuePtr();
    const SparseMatrixType::StorageIndex* outer = D_b_T.outerIndexPtr();

    // #pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        int type = data_manager->host_data.bilateral_type[cntr];
        double* v = vals + outer[off + index];

        switch (type) {
            case BilateralType::BODY_BODY: {
                ChConstraintTwoBodies* mbilateral = (ChConstraintTwoBodies*)(mconstraints[cntr]);
                int idA = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetIndex();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetIndex();
                const auto& cq_lo = (idA <= idB) ? mbilateral->Get_Cq_a() : mbilateral->Get_Cq_b();
                const auto& cq_hi = (idA <= idB) ? mbilateral->Get_Cq_b() : mbilateral->Get_Cq_a();
                for (int k = 0; k < 6; ++k) {
                    v[k] = cq_lo(k); v[6 + k] = cq_hi(k);
                }
            } break;

            case BilateralType::SHAFT_SHAFT: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                if (idA <= idB) {
                    v[0] = mbilateral->Get_Cq_a()(0); v[1] = mbilateral->Get_Cq_b()(0);
                } else {
                    v[0] = mbilateral->Get_Cq_b()(0); v[1] = mbilateral->Get_Cq_a()(0);
                }
            } break;

            case BilateralType::SHAFT_BODY: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);
                for (int k = 0; k < 6; ++k) v[k] = mbilateral->Get_Cq_b()(k);
                v[6] = mbilateral->Get_Cq_a()(0);
            } break;

            case BilateralType::SHAFT_SHAFT_SHAFT: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                int idC = ((ChVariablesShaft*)(mbilateral->GetVariables_c()))->GetShaft()->GetIndex();
                std::pair<int, double> entries[3] = {
                    {idA, mbilateral->Get_Cq_a()(0)},
                    {idB, mbilateral->Get_Cq_b()(0)},
                    {idC, mbilateral->Get_Cq_c()(0)}
                };
                std::sort(entries, entries + 3, [](const auto& x, const auto& y) { return x.first < y.first; });
                v[0] = entries[0].second;
                v[1] = entries[1].second;
                v[2] = entries[2].second;
            } break;

            case BilateralType::SHAFT_SHAFT_BODY: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetIndex();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetIndex();
                for (int k = 0; k < 6; ++k) v[k] = mbilateral->Get_Cq_c()(k);
                if (idA <= idB) {
                    v[6] = mbilateral->Get_Cq_a()(0); v[7] = mbilateral->Get_Cq_b()(0);
                } else {
                    v[6] = mbilateral->Get_Cq_b()(0); v[7] = mbilateral->Get_Cq_a()(0);
                }
            } break;
        }
    }
}

void ChConstraintBilateral::GenerateSparsity() {
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
