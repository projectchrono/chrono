#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

using namespace chrono;

void ChLcpSolverParallelDVI::RunTimeStep(real step) {
  // Setup constants and other values for system

  data_container->settings.step_size = step;

  // Compute the offsets and number of constrains depending on the solver
  // mode
  if (data_container->settings.solver.solver_mode == NORMAL) {
    rigid_rigid.offset = 1;
    data_container->num_unilaterals = 1 * data_container->num_contacts;
  } else if (data_container->settings.solver.solver_mode == SLIDING) {
    rigid_rigid.offset = 3;
    data_container->num_unilaterals = 3 * data_container->num_contacts;
  } else if (data_container->settings.solver.solver_mode == SPINNING) {
    rigid_rigid.offset = 6;
    data_container->num_unilaterals = 6 * data_container->num_contacts;
  }
  // This is the total number of constraints
  data_container->num_constraints = data_container->num_unilaterals + data_container->num_bilaterals;
  // Generate the mass matrix and compute M_inv_k
  ComputeMassMatrix();

  data_container->host_data.gamma.resize(data_container->num_constraints);
  data_container->host_data.gamma.reset();

  // Perform any setup tasks for all constraint types
  rigid_rigid.Setup(data_container);
  bilateral.Setup(data_container);
  // Clear and reset solver history data and counters
  solver->current_iteration = 0;
  solver->total_iteration = 0;
  solver->maxd_hist.clear();
  solver->maxdeltalambda_hist.clear();
  solver->iter_hist.clear();
  // Set pointers to constraint objects and perform setup actions for solver
  solver->rigid_rigid = &rigid_rigid;
  solver->bilateral = &bilateral;
  solver->Setup(data_container);

//  // This will be used as storage to keep gammas between time steps
//  data_container->host_data.gamma_bilateral.resize(data_container->num_bilaterals);
//
//#pragma omp parallel for
//  for (int i = 0; i < data_container->num_bilaterals; i++) {
//    data_container->host_data.gamma[i + data_container->num_unilaterals] = data_container->host_data.gamma_bilateral[i];
//  }

}

void ChLcpSolverParallelDVI::ComputeD() {

  CompressedMatrix<real>& D_T = data_container->host_data.D_T;

  uint& num_constraints = data_container->num_constraints;
  uint& num_bodies = data_container->num_bodies;
  uint& num_contacts = data_container->num_contacts;
  uint& num_bilaterals = data_container->num_bilaterals;
  if (num_constraints <= 0) {
    return;
  }
  clear(D_T);

  int unilateral_reserve = 0;

  if (data_container->settings.solver.solver_mode == NORMAL) {
    unilateral_reserve = 6 * 2 * data_container->num_contacts;
  } else if (data_container->settings.solver.solver_mode == SLIDING) {
    unilateral_reserve = 6 * 6 * data_container->num_contacts;
  } else if (data_container->settings.solver.solver_mode == SPINNING) {
    unilateral_reserve = 6 * 9 * data_container->num_contacts;
  }

  int constraint_reserve = 0;

  if (D_T.capacity() < constraint_reserve) {
    D_T.reserve(constraint_reserve * 1.2);
  }

  D_T.resize(num_constraints, num_bodies * 6, false);
  rigid_rigid.GenerateSparsity(data_container->settings.solver.solver_mode);
  bilateral.GenerateSparsity(data_container->settings.solver.solver_mode);
  rigid_rigid.Build_D(data_container->settings.solver.solver_mode);
  bilateral.Build_D();

  data_container->host_data.D = trans(data_container->host_data.D_T);
  data_container->host_data.M_invD = data_container->host_data.M_inv * data_container->host_data.D;
}

void ChLcpSolverParallelDVI::ComputeE() {

  uint& num_constraints = data_container->num_constraints;
  if (num_constraints <= 0) {
    return;
  }
  DynamicVector<real>& E = data_container->host_data.E;
  E.resize(num_constraints);
  reset(E);

  rigid_rigid.Build_E(data_container->settings.solver.solver_mode);
  bilateral.Build_E();
}

void ChLcpSolverParallelDVI::ComputeR(SOLVERMODE mode) {
  if (data_container->num_constraints <= 0) {
    return;
  }
  data_container->host_data.b.resize(data_container->num_constraints);
  reset(data_container->host_data.b);

  rigid_rigid.Build_b(data_container->settings.solver.solver_mode);
  bilateral.Build_b();

  data_container->host_data.R = -data_container->host_data.b - data_container->host_data.D_T * data_container->host_data.M_invk;
}


