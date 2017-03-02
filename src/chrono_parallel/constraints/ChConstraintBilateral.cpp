#include <algorithm>

#include "chrono_parallel/constraints/ChConstraintBilateral.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"

#include "chrono/solver/ChConstraintTwoBodies.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChConstraintThreeGeneric.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChShaft.h"

using namespace chrono;

void ChConstraintBilateral::Build_b() {
    std::vector<ChConstraint*>& mconstraints = data_manager->system_descriptor->GetConstraintsList();

#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        ChConstraintTwoBodies* mbilateral = (ChConstraintTwoBodies*)(mconstraints[cntr]);
        data_manager->host_data.b[index + data_manager->num_unilaterals] = mbilateral->Get_b_i();
    }
}

void ChConstraintBilateral::Build_E() {
#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        data_manager->host_data.E[index + data_manager->num_unilaterals] = 0;
    }
}

void ChConstraintBilateral::Build_D() {
    LOG(INFO) << "ChConstraintBilateral::Build_D";
    // Grab the list of all bilateral constraints present in the system
    // (note that this includes possibly inactive constraints)
    std::vector<ChConstraint*>& mconstraints = data_manager->system_descriptor->GetConstraintsList();

    // Loop over the active constraints and fill in the rows of the Jacobian,
    // taking into account the type of each constraint.
    SubMatrixType D_b_T = _DBT_;

    const CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;

    //#pragma omp parallel for
    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        int type = data_manager->host_data.bilateral_type[cntr];
        int row = index;

        switch (type) {
            case BilateralType::BODY_BODY: {
                ChConstraintTwoBodies* mbilateral = (ChConstraintTwoBodies*)(mconstraints[cntr]);

                int idA = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetId();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();
                int colA = idA * 6;
                int colB = idB * 6;

                D_b_T(row, colA + 0) = mbilateral->Get_Cq_a()->GetElementN(0);
                D_b_T(row, colA + 1) = mbilateral->Get_Cq_a()->GetElementN(1);
                D_b_T(row, colA + 2) = mbilateral->Get_Cq_a()->GetElementN(2);

                D_b_T(row, colA + 3) = mbilateral->Get_Cq_a()->GetElementN(3);
                D_b_T(row, colA + 4) = mbilateral->Get_Cq_a()->GetElementN(4);
                D_b_T(row, colA + 5) = mbilateral->Get_Cq_a()->GetElementN(5);

                D_b_T(row, colB + 0) = mbilateral->Get_Cq_b()->GetElementN(0);
                D_b_T(row, colB + 1) = mbilateral->Get_Cq_b()->GetElementN(1);
                D_b_T(row, colB + 2) = mbilateral->Get_Cq_b()->GetElementN(2);

                D_b_T(row, colB + 3) = mbilateral->Get_Cq_b()->GetElementN(3);
                D_b_T(row, colB + 4) = mbilateral->Get_Cq_b()->GetElementN(4);
                D_b_T(row, colB + 5) = mbilateral->Get_Cq_b()->GetElementN(5);
            } break;

            case BilateralType::SHAFT_SHAFT: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);

                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = data_manager->num_rigid_bodies * 6 + idB;

                D_b_T(row, colA) = mbilateral->Get_Cq_a()->GetElementN(0);
                D_b_T(row, colB) = mbilateral->Get_Cq_b()->GetElementN(0);
            } break;

            case BilateralType::SHAFT_BODY: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);

                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = idB * 6;

                D_b_T(row, colA) = mbilateral->Get_Cq_a()->GetElementN(0);

                D_b_T(row, colB + 0) = mbilateral->Get_Cq_b()->GetElementN(0);
                D_b_T(row, colB + 1) = mbilateral->Get_Cq_b()->GetElementN(1);
                D_b_T(row, colB + 2) = mbilateral->Get_Cq_b()->GetElementN(2);

                D_b_T(row, colB + 3) = mbilateral->Get_Cq_b()->GetElementN(3);
                D_b_T(row, colB + 4) = mbilateral->Get_Cq_b()->GetElementN(4);
                D_b_T(row, colB + 5) = mbilateral->Get_Cq_b()->GetElementN(5);
            } break;

            case BilateralType::SHAFT_SHAFT_SHAFT: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
                int idC = ((ChVariablesShaft*)(mbilateral->GetVariables_c()))->GetShaft()->GetId();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = data_manager->num_rigid_bodies * 6 + idB;
                int colC = data_manager->num_rigid_bodies * 6 + idC;

                D_b_T(row, colA) = mbilateral->Get_Cq_a()->GetElementN(0);
                D_b_T(row, colB) = mbilateral->Get_Cq_b()->GetElementN(0);
                D_b_T(row, colC) = mbilateral->Get_Cq_c()->GetElementN(0);
            } break;

            case BilateralType::SHAFT_SHAFT_BODY: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
                int idC = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_c()))->GetUserData())->GetId();

                int colA = data_manager->num_rigid_bodies * 6 + idA;
                int colB = data_manager->num_rigid_bodies * 6 + idB;
                int colC = idC * 6;

                D_b_T(row, colA) = mbilateral->Get_Cq_a()->GetElementN(0);
                D_b_T(row, colB) = mbilateral->Get_Cq_b()->GetElementN(0);

                D_b_T(row, colC + 0) = mbilateral->Get_Cq_c()->GetElementN(0);
                D_b_T(row, colC + 1) = mbilateral->Get_Cq_c()->GetElementN(1);
                D_b_T(row, colC + 2) = mbilateral->Get_Cq_c()->GetElementN(2);

                D_b_T(row, colC + 3) = mbilateral->Get_Cq_c()->GetElementN(3);
                D_b_T(row, colC + 4) = mbilateral->Get_Cq_c()->GetElementN(4);
                D_b_T(row, colC + 5) = mbilateral->Get_Cq_c()->GetElementN(5);
            } break;
        }
    }
}

void ChConstraintBilateral::GenerateSparsity() {
    LOG(INFO) << "ChConstraintBilateral::GenerateSparsity";
    // Grab the list of all bilateral constraints present in the system
    // (note that this includes possibly inactive constraints)

    std::vector<ChConstraint*>& mconstraints = data_manager->system_descriptor->GetConstraintsList();

    // Loop over the active constraints and fill in the sparsity pattern of the
    // Jacobian, taking into account the type of each constraint.
    // Note that the data for a Blaze compressed matrix must be filled in increasing
    // order of the column index for each row. Recall that body states are always
    // before shaft states.

    CompressedMatrix<real>& D_b_T = data_manager->host_data.D_T;
    int off = data_manager->num_unilaterals;
    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        int type = data_manager->host_data.bilateral_type[cntr];
        int row = off + index;

        int col1;
        int col2;
        int col3;

        switch (type) {
            case BilateralType::BODY_BODY: {
                ChConstraintTwoBodies* mbilateral = (ChConstraintTwoBodies*)(mconstraints[cntr]);

                int idA = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetId();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();

                if (idA < idB) {
                    col1 = idA * 6;
                    col2 = idB * 6;
                } else {
                    col1 = idB * 6;
                    col2 = idA * 6;
                }

                D_b_T.append(row, col1 + 0, 1);
                D_b_T.append(row, col1 + 1, 1);
                D_b_T.append(row, col1 + 2, 1);
                D_b_T.append(row, col1 + 3, 1);
                D_b_T.append(row, col1 + 4, 1);
                D_b_T.append(row, col1 + 5, 1);

                D_b_T.append(row, col2 + 0, 1);
                D_b_T.append(row, col2 + 1, 1);
                D_b_T.append(row, col2 + 2, 1);
                D_b_T.append(row, col2 + 3, 1);
                D_b_T.append(row, col2 + 4, 1);
                D_b_T.append(row, col2 + 5, 1);
            } break;

            case BilateralType::SHAFT_SHAFT: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);

                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();

                if (idA < idB) {
                    col1 = data_manager->num_rigid_bodies * 6 + idA;
                    col2 = data_manager->num_rigid_bodies * 6 + idB;
                } else {
                    col1 = data_manager->num_rigid_bodies * 6 + idB;
                    col2 = data_manager->num_rigid_bodies * 6 + idA;
                }

                D_b_T.append(row, col1, 1);
                D_b_T.append(row, col2, 1);
            } break;

            case BilateralType::SHAFT_BODY: {
                ChConstraintTwoGeneric* mbilateral = (ChConstraintTwoGeneric*)(mconstraints[cntr]);

                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                int idB = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();

                col1 = idB * 6;
                col2 = data_manager->num_rigid_bodies * 6 + idA;

                D_b_T.append(row, col1 + 0, 1);
                D_b_T.append(row, col1 + 1, 1);
                D_b_T.append(row, col1 + 2, 1);
                D_b_T.append(row, col1 + 3, 1);
                D_b_T.append(row, col1 + 4, 1);
                D_b_T.append(row, col1 + 5, 1);

                D_b_T.append(row, col2, 1);
            } break;

            case BilateralType::SHAFT_SHAFT_SHAFT: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                std::vector<int> ids(3);
                ids[0] = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                ids[1] = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
                ids[2] = ((ChVariablesShaft*)(mbilateral->GetVariables_c()))->GetShaft()->GetId();

                std::sort(ids.begin(), ids.end());
                col1 = data_manager->num_rigid_bodies * 6 + ids[0];
                col2 = data_manager->num_rigid_bodies * 6 + ids[1];
                col3 = data_manager->num_rigid_bodies * 6 + ids[2];

                D_b_T.append(row, col1, 1);
                D_b_T.append(row, col2, 1);
                D_b_T.append(row, col3, 1);
            } break;

            case BilateralType::SHAFT_SHAFT_BODY: {
                ChConstraintThreeGeneric* mbilateral = (ChConstraintThreeGeneric*)(mconstraints[cntr]);
                int idA = ((ChVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
                int idB = ((ChVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
                int idC = ((ChBody*)((ChVariablesBody*)(mbilateral->GetVariables_c()))->GetUserData())->GetId();

                col1 = idC * 6;
                if (idA < idB) {
                    col2 = data_manager->num_rigid_bodies * 6 + idA;
                    col3 = data_manager->num_rigid_bodies * 6 + idB;
                } else {
                    col2 = data_manager->num_rigid_bodies * 6 + idB;
                    col3 = data_manager->num_rigid_bodies * 6 + idA;
                }

                D_b_T.append(row, col1 + 0, 1);
                D_b_T.append(row, col1 + 1, 1);
                D_b_T.append(row, col1 + 2, 1);
                D_b_T.append(row, col1 + 3, 1);
                D_b_T.append(row, col1 + 4, 1);
                D_b_T.append(row, col1 + 5, 1);

                D_b_T.append(row, col2, 1);
                D_b_T.append(row, col3, 1);
            } break;
        }

        D_b_T.finalize(row);
    }
}
