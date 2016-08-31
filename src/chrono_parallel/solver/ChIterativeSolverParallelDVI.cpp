#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

#include "chrono_parallel/solver/ChSolverParallelAPGD.h"
#include "chrono_parallel/solver/ChSolverParallelAPGDREF.h"
#include "chrono_parallel/solver/ChSolverParallelBiCG.h"
#include "chrono_parallel/solver/ChSolverParallelBiCGStab.h"
#include "chrono_parallel/solver/ChSolverParallelCG.h"
#include "chrono_parallel/solver/ChSolverParallelCGS.h"
#include "chrono_parallel/solver/ChSolverParallelMinRes.h"
#include "chrono_parallel/solver/ChSolverParallelSD.h"
#include "chrono_parallel/solver/ChSolverParallelGD.h"
#include "chrono_parallel/solver/ChSolverParallelPGS.h"
#include "chrono_parallel/solver/ChSolverParallelJacobi.h"
#include "chrono_parallel/solver/ChSolverParallelPDIP.h"
using namespace chrono;

#define CLEAR_RESERVE_RESIZE(M, nnz, rows, cols) \
  if (M.capacity() > 0) clear(M);                \
  M.reserve(nnz);                                \
  M.resize(rows, cols, false);

void ChIterativeSolverParallelDVI::RunTimeStep() {
  LOG(INFO) << "ChIterativeSolverParallelDVI::RunTimeStep";
  // Compute the offsets and number of constrains depending on the solver mode
  if (data_manager->settings.solver.solver_mode == NORMAL) {
    rigid_rigid.offset = 1;
    data_manager->num_unilaterals = 1 * data_manager->num_rigid_contacts;
  } else if (data_manager->settings.solver.solver_mode == SLIDING) {
    rigid_rigid.offset = 3;
    data_manager->num_unilaterals = 3 * data_manager->num_rigid_contacts;
  } else if (data_manager->settings.solver.solver_mode == SPINNING) {
    rigid_rigid.offset = 6;
    data_manager->num_unilaterals = 6 * data_manager->num_rigid_contacts;
  }
  // This is the total number of constraints
  data_manager->num_constraints = data_manager->num_unilaterals + data_manager->num_bilaterals;

  // Generate the mass matrix and compute M_inv_k
  ComputeMassMatrix();

  data_manager->host_data.gamma.resize(data_manager->num_constraints);
  data_manager->host_data.gamma.reset();

  // Perform any setup tasks for all constraint types
  rigid_rigid.Setup(data_manager);
  bilateral.Setup(data_manager);
  // Clear and reset solver history data and counters
  solver->current_iteration = 0;
  data_manager->measures.solver.total_iteration = 0;
  data_manager->measures.solver.maxd_hist.clear();
  data_manager->measures.solver.maxdeltalambda_hist.clear();
  // Set pointers to constraint objects and perform setup actions for solver
  solver->rigid_rigid = &rigid_rigid;
  solver->bilateral = &bilateral;
  solver->Setup(data_manager);

  ComputeD();
  ComputeE();
  ComputeR();
  //ComputeN();

  //PreSolve();

  data_manager->system_timer.start("ChIterativeSolverParallel_Solve");

  //  if (data_manager->settings.solver.max_iteration_bilateral > 0) {
  //    solver->SetMaxIterations(data_manager->settings.solver.max_iteration_bilateral);
  //    data_manager->settings.solver.local_solver_mode = BILATERAL;
  //    SetR();
  //    solver->Solve();
  //  }

  PerformStabilization();

  if (data_manager->settings.solver.solver_mode == NORMAL || data_manager->settings.solver.solver_mode == SLIDING ||
      data_manager->settings.solver.solver_mode == SPINNING) {
    if (data_manager->settings.solver.max_iteration_normal > 0) {
      solver->SetMaxIterations(data_manager->settings.solver.max_iteration_normal);
      data_manager->settings.solver.local_solver_mode = NORMAL;
      SetR();
      LOG(INFO) << "ChIterativeSolverParallelDVI::RunTimeStep - Solve Normal";
      solver->Solve();
    }
  }
  if (data_manager->settings.solver.solver_mode == SLIDING ||
      data_manager->settings.solver.solver_mode == SPINNING) {
    if (data_manager->settings.solver.max_iteration_sliding > 0) {
      solver->SetMaxIterations(data_manager->settings.solver.max_iteration_sliding);
      data_manager->settings.solver.local_solver_mode = SLIDING;
      SetR();
      LOG(INFO) << "ChIterativeSolverParallelDVI::RunTimeStep - Solve Sliding";
      solver->Solve();
    }
  }
  if (data_manager->settings.solver.solver_mode == SPINNING) {
    if (data_manager->settings.solver.max_iteration_spinning > 0) {
      solver->SetMaxIterations(data_manager->settings.solver.max_iteration_spinning);
      data_manager->settings.solver.local_solver_mode = SPINNING;
      SetR();
      LOG(INFO) << "ChIterativeSolverParallelDVI::RunTimeStep - Solve Spinning";
      solver->Solve();
    }
  }

  data_manager->Fc_current = false;
  data_manager->system_timer.stop("ChIterativeSolverParallel_Solve");

  ComputeImpulses();

  for (int i = 0; i < data_manager->measures.solver.maxd_hist.size(); i++) {
    AtIterationEnd(data_manager->measures.solver.maxd_hist[i], data_manager->measures.solver.maxdeltalambda_hist[i],
                   i);
  }
  tot_iterations = data_manager->measures.solver.maxd_hist.size();

  LOG(TRACE) << "Solve Done: " << residual;
}

void ChIterativeSolverParallelDVI::ComputeD() {
  LOG(INFO) << "ChIterativeSolverParallelDVI::ComputeD()";
  data_manager->system_timer.start("ChIterativeSolverParallel_D");
  uint num_constraints = data_manager->num_constraints;
  if (num_constraints <= 0) {
    return;
  }

  uint num_bodies = data_manager->num_rigid_bodies;
  uint num_shafts = data_manager->num_shafts;
  uint num_dof = data_manager->num_dof;
  uint num_contacts = data_manager->num_rigid_contacts;
  uint num_bilaterals = data_manager->num_bilaterals;
  uint nnz_bilaterals = data_manager->nnz_bilaterals;

  int nnz_normal = 6 * 2 * data_manager->num_rigid_contacts;
  int nnz_tangential = 6 * 4 * data_manager->num_rigid_contacts;
  int nnz_spinning = 6 * 3 * data_manager->num_rigid_contacts;

  int num_normal = 1 * data_manager->num_rigid_contacts;
  int num_tangential = 2 * data_manager->num_rigid_contacts;
  int num_spinning = 3 * data_manager->num_rigid_contacts;

  CompressedMatrix<real>& D_n_T = data_manager->host_data.D_n_T;
  CompressedMatrix<real>& D_t_T = data_manager->host_data.D_t_T;
  CompressedMatrix<real>& D_s_T = data_manager->host_data.D_s_T;
  CompressedMatrix<real>& D_b_T = data_manager->host_data.D_b_T;

  CompressedMatrix<real>& D_n = data_manager->host_data.D_n;
  CompressedMatrix<real>& D_t = data_manager->host_data.D_t;
  CompressedMatrix<real>& D_s = data_manager->host_data.D_s;
  CompressedMatrix<real>& D_b = data_manager->host_data.D_b;

  CompressedMatrix<real>& M_invD_n = data_manager->host_data.M_invD_n;
  CompressedMatrix<real>& M_invD_t = data_manager->host_data.M_invD_t;
  CompressedMatrix<real>& M_invD_s = data_manager->host_data.M_invD_s;
  CompressedMatrix<real>& M_invD_b = data_manager->host_data.M_invD_b;

  const CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;

  switch (data_manager->settings.solver.solver_mode) {
    case NORMAL:
      CLEAR_RESERVE_RESIZE(D_n_T, nnz_normal, num_normal, num_dof)
      CLEAR_RESERVE_RESIZE(D_n, nnz_normal, num_dof, num_normal)
      CLEAR_RESERVE_RESIZE(M_invD_n, nnz_normal, num_dof, num_normal)
      break;
    case SLIDING:

      CLEAR_RESERVE_RESIZE(D_n_T, nnz_normal, num_normal, num_dof)
      CLEAR_RESERVE_RESIZE(D_n, nnz_normal, num_dof, num_normal)
      CLEAR_RESERVE_RESIZE(M_invD_n, nnz_normal, num_dof, num_normal)

      CLEAR_RESERVE_RESIZE(D_t_T, nnz_tangential, num_tangential, num_dof)
      CLEAR_RESERVE_RESIZE(D_t, nnz_tangential, num_dof, num_tangential)
      CLEAR_RESERVE_RESIZE(M_invD_t, nnz_tangential, num_dof, num_tangential)

      break;
    case SPINNING:

      CLEAR_RESERVE_RESIZE(D_n_T, nnz_normal, num_normal, num_dof)
      CLEAR_RESERVE_RESIZE(D_n, nnz_normal, num_dof, num_normal)
      CLEAR_RESERVE_RESIZE(M_invD_n, nnz_normal, num_dof, num_normal)

      CLEAR_RESERVE_RESIZE(D_t_T, nnz_tangential, num_tangential, num_dof)
      CLEAR_RESERVE_RESIZE(D_t, nnz_tangential, num_dof, num_tangential)
      CLEAR_RESERVE_RESIZE(M_invD_t, nnz_tangential, num_dof, num_tangential)

      CLEAR_RESERVE_RESIZE(D_s_T, nnz_spinning, num_spinning, num_dof)
      CLEAR_RESERVE_RESIZE(D_s, nnz_spinning, num_dof, num_spinning)
      CLEAR_RESERVE_RESIZE(M_invD_s, nnz_spinning, num_dof, num_spinning)

      break;
  }
  CLEAR_RESERVE_RESIZE(D_b_T, nnz_bilaterals, num_bilaterals, num_dof)

  rigid_rigid.GenerateSparsity();
  bilateral.GenerateSparsity();
  rigid_rigid.Build_D();
  bilateral.Build_D();

  data_manager->system_timer.stop("ChIterativeSolverParallel_D");
}

void ChIterativeSolverParallelDVI::ComputeE() {
  LOG(INFO) << "ChIterativeSolverParallelDVI::ComputeE()";
  data_manager->system_timer.start("ChIterativeSolverParallel_E");
  if (data_manager->num_constraints <= 0) {
    return;
  }

  data_manager->host_data.E.resize(data_manager->num_constraints);
  reset(data_manager->host_data.E);

  rigid_rigid.Build_E();
  bilateral.Build_E();
  data_manager->system_timer.stop("ChIterativeSolverParallel_E");
}

void ChIterativeSolverParallelDVI::ComputeR() {
  LOG(INFO) << "ChIterativeSolverParallelDVI::ComputeR()";
  data_manager->system_timer.start("ChIterativeSolverParallel_R");
  if (data_manager->num_constraints <= 0) {
    return;
  }

  const CompressedMatrix<real>& D_n_T = data_manager->host_data.D_n_T;
  const CompressedMatrix<real>& D_t_T = data_manager->host_data.D_t_T;
  const CompressedMatrix<real>& D_s_T = data_manager->host_data.D_s_T;
  const CompressedMatrix<real>& D_b_T = data_manager->host_data.D_b_T;

  const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;

  DynamicVector<real>& R = data_manager->host_data.R_full;
  DynamicVector<real>& b = data_manager->host_data.b;

  uint num_contacts = data_manager->num_rigid_contacts;
  uint num_unilaterals = data_manager->num_unilaterals;
  uint num_bilaterals = data_manager->num_bilaterals;

  b.resize(data_manager->num_constraints);
  reset(b);

  R.resize(data_manager->num_constraints);
  reset(R);

  rigid_rigid.Build_b();
  bilateral.Build_b();

  SubVectorType b_n = blaze::subvector(b, 0, num_contacts);
  SubVectorType R_n = blaze::subvector(R, 0, num_contacts);

  SubVectorType b_b = blaze::subvector(b, num_unilaterals, num_bilaterals);
  SubVectorType R_b = blaze::subvector(R, num_unilaterals, num_bilaterals);

  R_b = -b_b - D_b_T * M_invk;
  switch (data_manager->settings.solver.solver_mode) {
    case NORMAL: {
      R_n = -b_n - D_n_T * M_invk;
    } break;

    case SLIDING: {
      // SubVectorType b_t = blaze::subvector(b, num_contacts, num_contacts * 2);
      SubVectorType R_t = blaze::subvector(R, num_contacts, num_contacts * 2);

      R_n = -b_n - D_n_T * M_invk;
      R_t = -D_t_T * M_invk;
    } break;

    case SPINNING: {
      // SubVectorType b_t = blaze::subvector(b, num_contacts, num_contacts * 2);
      SubVectorType R_t = blaze::subvector(R, num_contacts, num_contacts * 2);

      // SubVectorType b_s = blaze::subvector(b, num_contacts * 3, num_contacts * 3);
      SubVectorType R_s = blaze::subvector(R, num_contacts * 3, num_contacts * 3);

      R_n = -b_n - D_n_T * M_invk;
      R_t = -D_t_T * M_invk;
      R_s = -D_s_T * M_invk;
    } break;
  }
  data_manager->system_timer.stop("ChIterativeSolverParallel_R");
}

void ChIterativeSolverParallelDVI::ComputeN() {
  LOG(INFO) << "ChIterativeSolverParallelDVI::ComputeN()";
  if (!data_manager->settings.solver.compute_N) {
    return;
  }

  data_manager->system_timer.start("ChIterativeSolverParallel_N");

  data_manager->system_timer.stop("ChIterativeSolverParallel_N");
}

void ChIterativeSolverParallelDVI::SetR() {
  LOG(INFO) << "ChIterativeSolverParallelDVI::SetR()";
  if (data_manager->num_constraints <= 0) {
    return;
  }

  DynamicVector<real>& R = data_manager->host_data.R;
  const DynamicVector<real>& R_full = data_manager->host_data.R_full;

  uint num_contacts = data_manager->num_rigid_contacts;
  uint num_unilaterals = data_manager->num_unilaterals;
  uint num_bilaterals = data_manager->num_bilaterals;
  R.resize(data_manager->num_constraints);
  reset(R);

  switch (data_manager->settings.solver.local_solver_mode) {
    case BILATERAL: {
      blaze::subvector(R, num_unilaterals, num_bilaterals) = blaze::subvector(R_full, num_unilaterals, num_bilaterals);
    } break;

    case NORMAL: {
      blaze::subvector(R, num_unilaterals, num_bilaterals) = blaze::subvector(R_full, num_unilaterals, num_bilaterals);
      blaze::subvector(R, 0, num_contacts) = blaze::subvector(R_full, 0, num_contacts);
    } break;
    case SLIDING: {
      blaze::subvector(R, num_unilaterals, num_bilaterals) = blaze::subvector(R_full, num_unilaterals, num_bilaterals);
      blaze::subvector(R, 0, num_contacts) = blaze::subvector(R_full, 0, num_contacts);
      blaze::subvector(R, num_contacts, num_contacts * 2) = blaze::subvector(R_full, num_contacts, num_contacts * 2);
    } break;

    case SPINNING: {
      blaze::subvector(R, num_unilaterals, num_bilaterals) = blaze::subvector(R_full, num_unilaterals, num_bilaterals);
      blaze::subvector(R, 0, num_contacts) = blaze::subvector(R_full, 0, num_contacts);
      blaze::subvector(R, num_contacts, num_contacts * 2) = blaze::subvector(R_full, num_contacts, num_contacts * 2);
      blaze::subvector(R, num_contacts * 3, num_contacts * 3) =
          blaze::subvector(R_full, num_contacts * 3, num_contacts * 3);
    } break;
  }
}

void ChIterativeSolverParallelDVI::ComputeImpulses() {
  LOG(INFO) << "ChIterativeSolverParallelDVI::ComputeImpulses()";
  DynamicVector<real>& v = data_manager->host_data.v;

  const DynamicVector<real>& M_invk = data_manager->host_data.M_invk;
  const DynamicVector<real>& gamma = data_manager->host_data.gamma;

  const CompressedMatrix<real>& M_invD_n = data_manager->host_data.M_invD_n;
  const CompressedMatrix<real>& M_invD_t = data_manager->host_data.M_invD_t;
  const CompressedMatrix<real>& M_invD_s = data_manager->host_data.M_invD_s;
  const CompressedMatrix<real>& M_invD_b = data_manager->host_data.M_invD_b;

  uint num_contacts = data_manager->num_rigid_contacts;
  uint num_unilaterals = data_manager->num_unilaterals;
  uint num_bilaterals = data_manager->num_bilaterals;

  if (data_manager->num_constraints > 0) {
    ConstSubVectorType gamma_b =
        blaze::subvector(gamma, num_unilaterals, num_bilaterals);
    ConstSubVectorType gamma_n = blaze::subvector(gamma, 0, num_contacts);

    // Compute new velocity based on the lagrange multipliers
    switch (data_manager->settings.solver.solver_mode) {
      case NORMAL: {
        v = M_invk + M_invD_n * gamma_n + M_invD_b * gamma_b;
      } break;

      case SLIDING: {
        ConstSubVectorType gamma_t =
            blaze::subvector(gamma, num_contacts, num_contacts * 2);

        v = M_invk + M_invD_n * gamma_n + M_invD_t * gamma_t + M_invD_b * gamma_b;
        // printf("-gamma: %f %f %f \n", gamma_n[0],gamma_t[0],gamma_t[1]);
      } break;

      case SPINNING: {
        ConstSubVectorType gamma_t =
            blaze::subvector(gamma, num_contacts, num_contacts * 2);
        ConstSubVectorType gamma_s =
            blaze::subvector(gamma, num_contacts * 3, num_contacts * 3);

        v = M_invk + M_invD_n * gamma_n + M_invD_t * gamma_t + M_invD_s * gamma_s + M_invD_b * gamma_b;

      } break;
    }
  } else {
    // When there are no constraints we need to still apply gravity and other
    // body forces!
    v = M_invk;
  }
}

void ChIterativeSolverParallelDVI::PreSolve() {
//Currently not supported, might be added back in the future
}

void ChIterativeSolverParallelDVI::ChangeSolverType(SOLVERTYPE type) {
  data_manager->settings.solver.solver_type = type;

  if (this->solver) {
    delete (this->solver);
  }
  switch (type) {
    case STEEPEST_DESCENT:
      solver = new ChSolverParallelSD();
      break;
    case GRADIENT_DESCENT:
      solver = new ChSolverParallelGD();
      break;
    case CONJUGATE_GRADIENT:
      solver = new ChSolverParallelCG();
      break;
    case CONJUGATE_GRADIENT_SQUARED:
      solver = new ChSolverParallelCGS();
      break;
    case BICONJUGATE_GRADIENT:
      solver = new ChSolverParallelBiCG();
      break;
    case BICONJUGATE_GRADIENT_STAB:
      solver = new ChSolverParallelBiCGStab();
      break;
    case MINIMUM_RESIDUAL:
      solver = new ChSolverParallelMinRes();
      break;
    case QUASI_MINIMUM_RESIDUAL:
      // This solver has not been implemented yet
      break;
    case APGD:
      solver = new ChSolverParallelAPGD();
      break;
    case APGDREF:
      solver = new ChSolverParallelAPGDREF();
      break;
    case JACOBI:
      solver = new ChSolverParallelJacobi();
      break;
    case GAUSS_SEIDEL:
      solver = new ChSolverParallelPGS();
      break;
    case PDIP:
      solver = new ChSolverParallelPDIP();
      break;
  }
}
