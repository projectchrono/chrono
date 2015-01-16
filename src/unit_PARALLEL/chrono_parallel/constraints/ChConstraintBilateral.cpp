#include <algorithm>

#include "chrono_parallel/constraints/ChConstraintBilateral.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

#include "lcp/ChLcpConstraintTwoBodies.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpConstraintThreeGeneric.h"
#include "physics/ChBody.h"
#include "physics/ChShaft.h"

using namespace chrono;

using blaze::DenseSubvector;
using blaze::subvector;

void ChConstraintBilateral::Build_b()
{
  std::vector<ChLcpConstraint*>& mconstraints = data_container->lcp_system_descriptor->GetConstraintsList();

#pragma omp parallel for
  for (int index = 0; index < data_container->num_bilaterals; index++) {
    int cntr = data_container->host_data.bilateral_mapping[index];
    ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*)(mconstraints[cntr]);
    data_container->host_data.b[index + data_container->num_unilaterals] = mbilateral->Get_b_i();
  }
}

void ChConstraintBilateral::Build_E()
{
#pragma omp parallel for
  for (int index = 0; index < data_container->num_bilaterals; index++) {
    data_container->host_data.E[index + data_container->num_unilaterals] = 0;
  }
}

void ChConstraintBilateral::Build_D()
{
  data_container->host_data.gamma_bilateral.resize(data_container->num_bilaterals);

  // Grab the list of all bilateral constraints present in the system
  // (note that this includes possibly inactive constraints)
  std::vector<ChLcpConstraint*>& mconstraints = data_container->lcp_system_descriptor->GetConstraintsList();

  // Loop over the active constraints and fill in the rows of the Jacobian,
  // taking into account the type of each constraint.
  CompressedMatrix<real>& D_T = data_container->host_data.D_T;

//#pragma omp parallel for
  for (int index = 0; index < data_container->num_bilaterals; index++) {
    int cntr = data_container->host_data.bilateral_mapping[index];
    int type = data_container->host_data.bilateral_type[cntr];
    int row = index + data_container->num_unilaterals;

    switch (type) {

    case BODY_BODY:
    {
      ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*)(mconstraints[cntr]);

      int idA = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetId();
      int idB = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();
      int colA = idA * 6;
      int colB = idB * 6;

      D_T(row, colA + 0) = mbilateral->Get_Cq_a()->GetElementN(0);
      D_T(row, colA + 1) = mbilateral->Get_Cq_a()->GetElementN(1);
      D_T(row, colA + 2) = mbilateral->Get_Cq_a()->GetElementN(2);

      D_T(row, colA + 3) = mbilateral->Get_Cq_a()->GetElementN(3);
      D_T(row, colA + 4) = mbilateral->Get_Cq_a()->GetElementN(4);
      D_T(row, colA + 5) = mbilateral->Get_Cq_a()->GetElementN(5);

      D_T(row, colB + 0) = mbilateral->Get_Cq_b()->GetElementN(0);
      D_T(row, colB + 1) = mbilateral->Get_Cq_b()->GetElementN(1);
      D_T(row, colB + 2) = mbilateral->Get_Cq_b()->GetElementN(2);

      D_T(row, colB + 3) = mbilateral->Get_Cq_b()->GetElementN(3);
      D_T(row, colB + 4) = mbilateral->Get_Cq_b()->GetElementN(4);
      D_T(row, colB + 5) = mbilateral->Get_Cq_b()->GetElementN(5);
    }
      break;

    case SHAFT_SHAFT:
    {
      ChLcpConstraintTwoGeneric* mbilateral = (ChLcpConstraintTwoGeneric*)(mconstraints[cntr]);

      int idA = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
      int idB = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
      int colA = data_container->num_bodies * 6 + idA;
      int colB = data_container->num_bodies * 6 + idB;

      D_T(row, colA) = mbilateral->Get_Cq_a()->GetElementN(0);
      D_T(row, colB) = mbilateral->Get_Cq_b()->GetElementN(0);
    }
      break;

    case SHAFT_BODY:
    {
      ChLcpConstraintTwoGeneric* mbilateral = (ChLcpConstraintTwoGeneric*)(mconstraints[cntr]);

      int idA = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
      int idB = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();
      int colA = data_container->num_bodies * 6 + idA;
      int colB = idB * 6;

      D_T(row, colA) = mbilateral->Get_Cq_a()->GetElementN(0);

      D_T(row, colB + 0) = mbilateral->Get_Cq_b()->GetElementN(0);
      D_T(row, colB + 1) = mbilateral->Get_Cq_b()->GetElementN(1);
      D_T(row, colB + 2) = mbilateral->Get_Cq_b()->GetElementN(2);

      D_T(row, colB + 3) = mbilateral->Get_Cq_b()->GetElementN(3);
      D_T(row, colB + 4) = mbilateral->Get_Cq_b()->GetElementN(4);
      D_T(row, colB + 5) = mbilateral->Get_Cq_b()->GetElementN(5);
    }
      break;

    case SHAFT_SHAFT_SHAFT:
    {
      ChLcpConstraintThreeGeneric* mbilateral = (ChLcpConstraintThreeGeneric*)(mconstraints[cntr]);
      int idA = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
      int idB = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
      int idC = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_c()))->GetShaft()->GetId();
      int colA = data_container->num_bodies * 6 + idA;
      int colB = data_container->num_bodies * 6 + idB;
      int colC = data_container->num_bodies * 6 + idC;

      D_T(row, colA) = mbilateral->Get_Cq_a()->GetElementN(0);
      D_T(row, colB) = mbilateral->Get_Cq_b()->GetElementN(0);
      D_T(row, colC) = mbilateral->Get_Cq_c()->GetElementN(0);
    }
      break;

    }
  }
}

void ChConstraintBilateral::GenerateSparsity()
{
  // Grab the list of all bilateral constraints present in the system
  // (note that this includes possibly inactive constraints)
  std::vector<ChLcpConstraint*>& mconstraints = data_container->lcp_system_descriptor->GetConstraintsList();

  // Loop over the active constraints and fill in the sparsity pattern of the
  // Jacobian, taking into account the type of each constraint.
  // Note that the data for a Blaze compressed matrix must be filled in increasing
  // order of the column index for each row.
  CompressedMatrix<real>& D_T = data_container->host_data.D_T;

  for (int index = 0; index < data_container->num_bilaterals; index++) {
    int cntr = data_container->host_data.bilateral_mapping[index];
    int type = data_container->host_data.bilateral_type[cntr];
    int row = index + data_container->num_unilaterals;
    int col1;
    int col2;

    switch (type) {

    case BODY_BODY:
    {
      ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*)(mconstraints[cntr]);

      int idA = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetId();
      int idB = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();
      if (idA < idB) {
        col1 = idA * 6;
        col2 = idB * 6;
      } else {
        col1 = idB * 6;
        col2 = idA * 6;
      }

      D_T.append(row, col1 + 0, 1); D_T.append(row, col1 + 1, 1); D_T.append(row, col1 + 2, 1);
      D_T.append(row, col1 + 3, 1); D_T.append(row, col1 + 4, 1); D_T.append(row, col1 + 5, 1);

      D_T.append(row, col2 + 0, 1); D_T.append(row, col2 + 1, 1); D_T.append(row, col2 + 2, 1);
      D_T.append(row, col2 + 3, 1); D_T.append(row, col2 + 4, 1); D_T.append(row, col2 + 5, 1);
    }
      break;

    case SHAFT_SHAFT:
    {
      ChLcpConstraintTwoGeneric* mbilateral = (ChLcpConstraintTwoGeneric*)(mconstraints[cntr]);

      int idA = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
      int idB = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
      if (idA < idB) {
        col1 = data_container->num_bodies * 6 + idA;
        col2 = data_container->num_bodies * 6 + idB;
      } else {
        col1 = data_container->num_bodies * 6 + idB;
        col2 = data_container->num_bodies * 6 + idA;
      }

      D_T.append(row, col1, 1);
      D_T.append(row, col2, 1);
    }
      break;

    case SHAFT_BODY:
    {
      ChLcpConstraintTwoGeneric* mbilateral = (ChLcpConstraintTwoGeneric*)(mconstraints[cntr]);

      int idA = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
      int idB = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();
      col1 = idB * 6;
      col2 = data_container->num_bodies * 6 + idA;

      D_T.append(row, col1 + 0, 1); D_T.append(row, col1 + 1, 1); D_T.append(row, col1 + 2, 1);
      D_T.append(row, col1 + 3, 1); D_T.append(row, col1 + 4, 1); D_T.append(row, col1 + 5, 1);

      D_T.append(row, col2, 1);
    }
      break;

    case SHAFT_SHAFT_SHAFT:
    {
      ChLcpConstraintThreeGeneric* mbilateral = (ChLcpConstraintThreeGeneric*)(mconstraints[cntr]);
      std::vector<int> ids(3);
      ids[0] = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_a()))->GetShaft()->GetId();
      ids[1] = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_b()))->GetShaft()->GetId();
      ids[2] = ((ChLcpVariablesShaft*)(mbilateral->GetVariables_c()))->GetShaft()->GetId();
      std::sort(ids.begin(), ids.end());

      D_T.append(row, data_container->num_bodies * 6 + ids[0], 1);
      D_T.append(row, data_container->num_bodies * 6 + ids[1], 1);
      D_T.append(row, data_container->num_bodies * 6 + ids[2], 1);
    }
      break;

    }

    D_T.finalize(row);
  }
}


