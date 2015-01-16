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

  // solve for the normal
  if (data_container->settings.solver.solver_mode == NORMAL || data_container->settings.solver.solver_mode == SLIDING || data_container->settings.solver.solver_mode == SPINNING) {
    if (data_container->settings.solver.max_iteration_normal > 0) {
      solver->SetMaxIterations(data_container->settings.solver.max_iteration_normal);
      rigid_rigid.solve_sliding = false;
      rigid_rigid.solve_spinning = false;
      ComputeR(NORMAL);
      data_container->system_timer.start("ChLcpSolverParallel_Solve");
      PerformStabilization();
      solver->Solve();
      data_container->system_timer.stop("ChLcpSolverParallel_Solve");
    }
  }
  if (data_container->settings.solver.solver_mode != NORMAL) {
    if (data_container->settings.solver.max_iteration_sliding > 0) {
      solver->SetMaxIterations(data_container->settings.solver.max_iteration_sliding);
      rigid_rigid.solve_sliding = true;
      rigid_rigid.solve_spinning = false;
      ComputeR(SLIDING);
      data_container->system_timer.start("ChLcpSolverParallel_Solve");
      PerformStabilization();
      solver->Solve();
      data_container->system_timer.stop("ChLcpSolverParallel_Solve");
    }
  }
  if (data_container->settings.solver.solver_mode == SPINNING) {
    if (data_container->settings.solver.max_iteration_spinning > 0) {
      solver->SetMaxIterations(data_container->settings.solver.max_iteration_spinning);
      rigid_rigid.solve_sliding = true;
      rigid_rigid.solve_spinning = true;
      ComputeR(SPINNING);
      data_container->system_timer.start("ChLcpSolverParallel_Solve");
      PerformStabilization();
      solver->Solve();
      data_container->system_timer.stop("ChLcpSolverParallel_Solve");
    }
  }

  ComputeImpulses();

    for (int i = 0; i <  data_container->measures.solver.iter_hist.size(); i++) {
      AtIterationEnd( data_container->measures.solver.maxd_hist[i],  data_container->measures.solver.maxdeltalambda_hist[i],  data_container->measures.solver.iter_hist[i]);
  }

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

  int nnz_unilaterals = 0;

  switch (data_container->settings.solver.solver_mode) {
  case NORMAL:
    nnz_unilaterals = 6 * 2 * data_container->num_contacts;
    break;
  case SLIDING:
    nnz_unilaterals = 6 * 6 * data_container->num_contacts;
    break;
  case SPINNING:
    nnz_unilaterals = 6 * 9 * data_container->num_contacts;
    break;
  }

  int nnz_total = nnz_unilaterals + nnz_bilaterals;

  CompressedMatrix<real>& D_T = data_container->host_data.D_T;
  clear(D_T);
  if (D_T.capacity() < nnz_total) {
    D_T.reserve(nnz_total * 1.2);
  }
  D_T.resize(num_constraints, num_dof, false);

  rigid_rigid.GenerateSparsity(data_container->settings.solver.solver_mode);
  bilateral.GenerateSparsity();
  rigid_rigid.Build_D(data_container->settings.solver.solver_mode);
  bilateral.Build_D();

  data_container->host_data.D = trans(data_container->host_data.D_T);
  data_container->host_data.M_invD = data_container->host_data.M_inv * data_container->host_data.D;
}

void ChLcpSolverParallelDVI::ComputeE()
{
  if (data_container->num_constraints <= 0) {
    return;
  }

  data_container->host_data.E.resize(data_container->num_constraints);
  reset(data_container->host_data.E);

  rigid_rigid.Build_E(data_container->settings.solver.solver_mode);
  bilateral.Build_E();
}

void ChLcpSolverParallelDVI::ComputeR(SOLVERMODE mode)
{
  if (data_container->num_constraints <= 0) {
    return;
  }

  data_container->host_data.b.resize(data_container->num_constraints);
  reset(data_container->host_data.b);

  rigid_rigid.Build_b(mode);
  bilateral.Build_b();

  data_container->host_data.R = -data_container->host_data.b - data_container->host_data.D_T * data_container->host_data.M_invk;
}



void ChLcpSolverParallelDVI::ChangeSolverType(SOLVERTYPE type) {
  data_container->settings.solver.solver_type = type;

  if (this->solver) {
    delete (this->solver);
  }
  if (type == STEEPEST_DESCENT) {
    solver = new ChSolverSD();
  } else if (type == GRADIENT_DESCENT) {
    solver = new ChSolverGD();
  } else if (type == CONJUGATE_GRADIENT) {
    solver = new ChSolverCG();
  } else if (type == CONJUGATE_GRADIENT_SQUARED) {
    solver = new ChSolverCGS();
  } else if (type == BICONJUGATE_GRADIENT) {
    solver = new ChSolverBiCG();
  } else if (type == BICONJUGATE_GRADIENT_STAB) {
    solver = new ChSolverBiCGStab();
  } else if (type == MINIMUM_RESIDUAL) {
    solver = new ChSolverMinRes();
  } else if (type == QUASAI_MINIMUM_RESIDUAL) {
    //         // This solver has not been implemented yet
    //         //SolveQMR(data_container->gpu_data.device_gam_data, rhs, max_iteration);
  } else if (type == APGD) {
    solver = new ChSolverAPGD();
  } else if (type == APGDREF) {
    solver = new ChSolverAPGDREF();
  } else if (type == JACOBI) {
    solver = new ChSolverJacobi();
  } else if (type == GAUSS_SEIDEL) {
    solver = new ChSolverPGS();
  } else if (type == PDIP) {
    solver = new ChSolverPDIP();
  }
}
