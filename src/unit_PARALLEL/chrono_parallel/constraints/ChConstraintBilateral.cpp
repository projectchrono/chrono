#include "chrono_parallel/constraints/ChConstraintBilateral.h"
#include "lcp/ChLcpConstraintTwoBodies.h"
#include "physics/ChBody.h"
using namespace chrono;


void ChConstraintBilateral::Build_b() {
  blaze::DenseSubvector<DynamicVector<real> > b = blaze::subvector(data_container->host_data.b, data_container->num_unilaterals, data_container->num_bilaterals);
  std::vector<ChLcpConstraint*>& mconstraints = (*data_container->lcp_system_descriptor).GetConstraintsList();
#pragma omp parallel for
  for (int index = 0; index < num_bilaterals; index++) {
    int cntr = data_container->host_data.bilateral_mapping[index];
    ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*)(mconstraints[cntr]);
    b[index] = mbilateral->Get_b_i();
  }
}

void ChConstraintBilateral::Build_E() {
#pragma omp parallel for
  for (int index = 0; index < num_bilaterals; index++) {
    data_container->host_data.E[index + num_unilaterals] = 0;
  }
}

void ChConstraintBilateral::Build_D() {
  data_container->host_data.gamma_bilateral.resize(num_bilaterals);

  ChLcpSystemDescriptor* lcp_sys = data_container->lcp_system_descriptor;
  std::vector<ChLcpConstraint*>& mconstraints = (*lcp_sys).GetConstraintsList();

  CompressedMatrix<real>& D_T = data_container->host_data.D_T;
#pragma omp parallel for
  for (int index = 0; index < num_bilaterals; index++) {
    int cntr = data_container->host_data.bilateral_mapping[index];
    ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*)(mconstraints[cntr]);

    int idA = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetId();
    int idB = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();

    mconstraints[cntr]->Update_auxiliary();

    int2 body_id = I2(idA, idB);

    D_T(index + num_unilaterals, body_id.x* 6 + 0) = mbilateral->Get_Cq_a()->GetElementN(0);
    D_T(index + num_unilaterals, body_id.x* 6 + 1) = mbilateral->Get_Cq_a()->GetElementN(1);
    D_T(index + num_unilaterals, body_id.x* 6 + 2) = mbilateral->Get_Cq_a()->GetElementN(2);

    D_T(index + num_unilaterals, body_id.x* 6 + 3) = mbilateral->Get_Cq_a()->GetElementN(3);
    D_T(index + num_unilaterals, body_id.x* 6 + 4) = mbilateral->Get_Cq_a()->GetElementN(4);
    D_T(index + num_unilaterals, body_id.x* 6 + 5) = mbilateral->Get_Cq_a()->GetElementN(5);

    D_T(index + num_unilaterals, body_id.y* 6 + 0) = mbilateral->Get_Cq_b()->GetElementN(0);
    D_T(index + num_unilaterals, body_id.y* 6 + 1) = mbilateral->Get_Cq_b()->GetElementN(1);
    D_T(index + num_unilaterals, body_id.y* 6 + 2) = mbilateral->Get_Cq_b()->GetElementN(2);

    D_T(index + num_unilaterals, body_id.y* 6 + 3) = mbilateral->Get_Cq_b()->GetElementN(3);
    D_T(index + num_unilaterals, body_id.y* 6 + 4) = mbilateral->Get_Cq_b()->GetElementN(4);
    D_T(index + num_unilaterals, body_id.y* 6 + 5) = mbilateral->Get_Cq_b()->GetElementN(5);
  }
}

void ChConstraintBilateral::GenerateSparsity(SOLVERMODE solver_mode) {
  CompressedMatrix<real>& D_T = data_container->host_data.D_T;
  ChLcpSystemDescriptor* lcp_sys = data_container->lcp_system_descriptor;
  std::vector<ChLcpConstraint*>& mconstraints = (*lcp_sys).GetConstraintsList();

  for (int index = 0; index < num_bilaterals; index++) {
    int cntr = data_container->host_data.bilateral_mapping[index];

    ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*)(mconstraints[cntr]);

    int idA = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_a()))->GetUserData())->GetId();
    int idB = ((ChBody*)((ChLcpVariablesBody*)(mbilateral->GetVariables_b()))->GetUserData())->GetId();
    int2 body_id = I2(idA, idB);

    // std::cout<<" "<<index + num_unilaterals<<" "<<body_id.x * 6 +
    // 0<<std::endl;
    if (idA > idB) {
      D_T.append(index + num_unilaterals, body_id.y * 6 + 0, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 1, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 2, 1);

      D_T.append(index + num_unilaterals, body_id.y * 6 + 3, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 4, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 5, 1);

      D_T.append(index + num_unilaterals, body_id.x * 6 + 0, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 1, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 2, 1);

      D_T.append(index + num_unilaterals, body_id.x * 6 + 3, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 4, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 5, 1);
    } else {
      D_T.append(index + num_unilaterals, body_id.x * 6 + 0, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 1, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 2, 1);

      D_T.append(index + num_unilaterals, body_id.x * 6 + 3, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 4, 1);
      D_T.append(index + num_unilaterals, body_id.x * 6 + 5, 1);

      D_T.append(index + num_unilaterals, body_id.y * 6 + 0, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 1, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 2, 1);

      D_T.append(index + num_unilaterals, body_id.y * 6 + 3, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 4, 1);
      D_T.append(index + num_unilaterals, body_id.y * 6 + 5, 1);
    }
    D_T.finalize(index + num_unilaterals);
  }
}


