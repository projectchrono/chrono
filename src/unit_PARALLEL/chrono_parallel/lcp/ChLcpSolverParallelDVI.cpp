#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

#include "chrono_parallel/solver/ChSolverAPGD.h"
#include "chrono_parallel/solver/ChSolverAPGDREF.h"
#include "chrono_parallel/solver/ChSolverBiCG.h"
#include "chrono_parallel/solver/ChSolverBiCGStab.h"
#include "chrono_parallel/solver/ChSolverCG.h"
#include "chrono_parallel/solver/ChSolverCGS.h"
#include "chrono_parallel/solver/ChSolverMinRes.h"
#include "chrono_parallel/solver/ChSolverSD.h"
#include "chrono_parallel/solver/ChSolverGD.h"
#include "chrono_parallel/solver/ChSolverPGS.h"
#include "chrono_parallel/solver/ChSolverJacobi.h"
#include "chrono_parallel/solver/ChSolverPDIP.h"
using namespace chrono;

void ChLcpSolverParallelDVI::RunTimeStep(real step)
{
  // Setup constants and other values for system
  data_container->settings.step_size = step;
  data_container->settings.solver.tol_speed = step * data_container->settings.solver.tolerance;

  // Compute the offsets and number of constrains depending on the solver mode
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
  // This is the total number of degrees of freedom in the system
  data_container->num_dof = data_container->num_bodies * 6 + data_container->num_shafts;

  // Generate the mass matrix and compute M_inv_k
  ComputeMassMatrix();

  data_container->host_data.gamma.resize(data_container->num_constraints);
  data_container->host_data.gamma.reset();

  // Perform any setup tasks for all constraint types
  rigid_rigid.Setup(data_container);
  bilateral.Setup(data_container);
  // Clear and reset solver history data and counters
  solver->current_iteration = 0;
  data_container->measures.solver.total_iteration = 0;
  data_container->measures.solver.maxd_hist.clear();
  data_container->measures.solver.maxdeltalambda_hist.clear();
  data_container->measures.solver.iter_hist.clear();
  // Set pointers to constraint objects and perform setup actions for solver
  solver->rigid_rigid = &rigid_rigid;
  solver->bilateral = &bilateral;
  solver->Setup(data_container);

  ComputeD();
  ComputeE();
  ComputeR();

  // solve for the normal
  if (data_container->settings.solver.solver_mode == NORMAL || data_container->settings.solver.solver_mode == SLIDING || data_container->settings.solver.solver_mode == SPINNING) {
    if (data_container->settings.solver.max_iteration_normal > 0) {
      solver->SetMaxIterations(data_container->settings.solver.max_iteration_normal);
      data_container->settings.solver.local_solver_mode = NORMAL;
      data_container->system_timer.start("ChLcpSolverParallel_Solve");
      PerformStabilization();
      solver->Solve();
      data_container->system_timer.stop("ChLcpSolverParallel_Solve");
    }
  }
  if (data_container->settings.solver.solver_mode != NORMAL) {
    if (data_container->settings.solver.max_iteration_sliding > 0) {
      solver->SetMaxIterations(data_container->settings.solver.max_iteration_sliding);
      data_container->settings.solver.local_solver_mode = SLIDING;
      data_container->system_timer.start("ChLcpSolverParallel_Solve");
      PerformStabilization();
      solver->Solve();
      data_container->system_timer.stop("ChLcpSolverParallel_Solve");
    }
  }
  if (data_container->settings.solver.solver_mode == SPINNING) {
    if (data_container->settings.solver.max_iteration_spinning > 0) {
      solver->SetMaxIterations(data_container->settings.solver.max_iteration_spinning);
      data_container->settings.solver.local_solver_mode = SPINNING;
      data_container->system_timer.start("ChLcpSolverParallel_Solve");
      PerformStabilization();
      solver->Solve();
      data_container->system_timer.stop("ChLcpSolverParallel_Solve");
    }
  }

  ComputeImpulses();

  for (int i = 0; i < data_container->measures.solver.iter_hist.size(); i++) {
    AtIterationEnd(data_container->measures.solver.maxd_hist[i], data_container->measures.solver.maxdeltalambda_hist[i], data_container->measures.solver.iter_hist[i]);
  }
  tot_iterations = data_container->measures.solver.iter_hist.size();

#if PRINT_LEVEL == 2
  std::cout << "Solve Done: " << residual << std::endl;
#endif
}

void ChLcpSolverParallelDVI::ComputeD()
{
  uint num_constraints = data_container->num_constraints;
  if (num_constraints <= 0) {
    return;
  }

  uint num_bodies = data_container->num_bodies;
  uint num_shafts = data_container->num_shafts;
  uint num_dof = data_container->num_dof;
  uint num_contacts = data_container->num_contacts;
  uint num_bilaterals = data_container->num_bilaterals;
  uint nnz_bilaterals = data_container->nnz_bilaterals;

  int nnz_normal = 6 * 2 * data_container->num_contacts;
  int nnz_tangential = 6 * 4 * data_container->num_contacts;
  int nnz_spinning = 6 * 3 * data_container->num_contacts;

  int num_normal = 1 * data_container->num_contacts;
  int num_tangential = 2 * data_container->num_contacts;
  int num_spinning = 3 * data_container->num_contacts;

  CompressedMatrix<real>& D_n_T = data_container->host_data.D_n_T;
  CompressedMatrix<real>& D_t_T = data_container->host_data.D_t_T;
  CompressedMatrix<real>& D_s_T = data_container->host_data.D_s_T;
  CompressedMatrix<real>& D_b_T = data_container->host_data.D_b_T;

  CompressedMatrix<real>& D_n = data_container->host_data.D_n;
  CompressedMatrix<real>& D_t = data_container->host_data.D_t;
  CompressedMatrix<real>& D_s = data_container->host_data.D_s;
  CompressedMatrix<real>& D_b = data_container->host_data.D_b;

  CompressedMatrix<real>& M_invD_n = data_container->host_data.M_invD_n;
  CompressedMatrix<real>& M_invD_t = data_container->host_data.M_invD_t;
  CompressedMatrix<real>& M_invD_s = data_container->host_data.M_invD_s;
  CompressedMatrix<real>& M_invD_b = data_container->host_data.M_invD_b;

  const CompressedMatrix<real>& M_inv = data_container->host_data.M_inv;

  switch (data_container->settings.solver.solver_mode) {
    case NORMAL:
      clear(D_n_T);

      D_n_T.reserve(nnz_normal);

      D_n_T.resize(num_normal, num_dof, false);
      break;
    case SLIDING:
      clear(D_n_T);
      clear(D_t_T);

      D_n_T.reserve(nnz_normal);
      D_t_T.reserve(nnz_tangential);

      D_n_T.resize(num_normal, num_dof, false);
      D_t_T.resize(num_tangential, num_dof, false);
      break;
    case SPINNING:
      clear(D_n_T);
      clear(D_t_T);
      clear(D_s_T);

      D_n_T.reserve(nnz_normal);
      D_t_T.reserve(nnz_tangential);
      D_s_T.reserve(nnz_spinning);

      D_n_T.resize(num_normal, num_dof, false);
      D_t_T.resize(num_tangential, num_dof, false);
      D_s_T.resize(num_spinning, num_dof, false);
      break;
  }

  clear(D_b_T);
  D_b_T.reserve(nnz_bilaterals);

  rigid_rigid.GenerateSparsity();
  bilateral.GenerateSparsity();
  rigid_rigid.Build_D();
  bilateral.Build_D();

  switch (data_container->settings.solver.solver_mode) {
    case NORMAL:
      D_n = trans(D_n_T);
      M_invD_n = M_inv * D_n;
      break;
    case SLIDING:
      D_n = trans(D_n_T);
      D_t = trans(D_t_T);
      M_invD_n = M_inv * D_n;
      M_invD_t = M_inv * D_t;
      break;
    case SPINNING:
      D_n = trans(D_n_T);
      D_t = trans(D_t_T);
      D_s = trans(D_s_T);
      M_invD_n = M_inv * D_n;
      M_invD_t = M_inv * D_t;
      M_invD_s = M_inv * D_s;
      break;
  }

  D_b = trans(D_b_T);
  M_invD_b = M_inv * D_b;
}

void ChLcpSolverParallelDVI::ComputeE()
{
  if (data_container->num_constraints <= 0) {
    return;
  }

  data_container->host_data.E.resize(data_container->num_constraints);
  reset(data_container->host_data.E);

  rigid_rigid.Build_E();
  bilateral.Build_E();
}

void ChLcpSolverParallelDVI::ComputeR()
{
  if (data_container->num_constraints <= 0) {
    return;
  }

  const CompressedMatrix<real>& D_n_T = data_container->host_data.D_n_T;
  const CompressedMatrix<real>& D_t_T = data_container->host_data.D_t_T;
  const CompressedMatrix<real>& D_s_T = data_container->host_data.D_s_T;
  const CompressedMatrix<real>& D_b_T = data_container->host_data.D_b_T;

  const DynamicVector<real>& M_invk = data_container->host_data.M_invk;

  DynamicVector<real>& R = data_container->host_data.R;
  DynamicVector<real>& b = data_container->host_data.b;

  b.resize(data_container->num_constraints);
  reset(b);

  rigid_rigid.Build_b();
  bilateral.Build_b();

  switch (data_container->settings.solver.solver_mode) {
    case NORMAL: {
      R = -b - D_n_T * M_invk;
    } break;

    case SLIDING: {
      R = -b - D_n_T * M_invk - D_t_T * M_invk;
    } break;

    case SPINNING: {
      R = -b - D_n_T * M_invk - D_t_T * M_invk - D_s_T * M_invk;
    } break;
  }
}

void ChLcpSolverParallelDVI::ChangeSolverType(SOLVERTYPE type) {
  data_container->settings.solver.solver_type = type;

  if (this->solver) {
    delete (this->solver);
  }
  switch (type) {
    case STEEPEST_DESCENT:
      solver = new ChSolverSD();
      break;
    case GRADIENT_DESCENT:
      solver = new ChSolverGD();
      break;
    case CONJUGATE_GRADIENT:
      solver = new ChSolverCG();
      break;
    case CONJUGATE_GRADIENT_SQUARED:
      solver = new ChSolverCGS();
      break;
    case BICONJUGATE_GRADIENT:
      solver = new ChSolverBiCG();
      break;
    case BICONJUGATE_GRADIENT_STAB:
      solver = new ChSolverBiCGStab();
      break;
    case MINIMUM_RESIDUAL:
      solver = new ChSolverMinRes();
      break;
    case QUASAI_MINIMUM_RESIDUAL:
      // This solver has not been implemented yet
      break;
    case APGD:
      solver = new ChSolverAPGD();
      break;
    case APGDREF:
      solver = new ChSolverAPGDREF();
      break;
    case JACOBI:
      solver = new ChSolverJacobi();
      break;
    case GAUSS_SEIDEL:
      solver = new ChSolverPGS();
      break;
    case PDIP:
      solver = new ChSolverPDIP();
      break;
  }

}
