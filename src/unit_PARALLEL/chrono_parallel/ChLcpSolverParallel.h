#ifndef CHC_LCPITERATIVESOLVERGPU_H
#define CHC_LCPITERATIVESOLVERGPU_H

//////////////////////////////////////////////////
//
//   ChIterativeGPU.h
//
//   GPU LCP Solver
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "lcp/ChLcpIterativeSolver.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/ChIntegratorParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/constraints/ChConstraintBilateral.h"
#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {

class CH_PARALLEL_API ChLcpSolverParallel : public ChLcpIterativeSolver {
 public:
  ChLcpSolverParallel() {
    tolerance = 1e-7;

    max_iter_bilateral = 100;
    record_violation_history = true;
    warm_start = false;

  }

  virtual ~ChLcpSolverParallel() {
  }

  virtual double Solve(ChLcpSystemDescriptor &sysd) {
    return 0;
  }
  void host_addForces(bool* active, real *mass, real3 *inertia, real3 *forces, real3 *torques, real3 *vel, real3 *omega);

  void host_ComputeGyro(real3 *omega, real3 *inertia, real3 *gyro, real3 *torque);

  void host_Integrate_Timestep(bool *active, real3 *acc, real4 *rot, real3 *vel, real3 *omega, real3 *pos, real3 *lim);

  virtual void RunTimeStep(real step) = 0;
  void Preprocess();

  void SetTolerance(real tol) {
    tolerance = tol;
  }

  void SetMaxIterationBilateral(uint max_iter) {
    max_iter_bilateral = max_iter;
  }
  real GetResidual() {
    return residual;
  }
  void Dump_Rhs(std::vector<double> &temp) {
    for (int i = 0; i < rhs.size(); i++) {
      temp.push_back(rhs[i]);
    }
  }
  void Dump_Shur(std::vector<double> &temp) {
    for (int i = 0; i < debug.size(); i++) {
      temp.push_back(debug[i]);
    }
  }
  void Dump_Lambda(std::vector<double> &temp) {
    for (int i = 0; i < lambda.size(); i++) {
      temp.push_back(lambda[i]);
    }
  }

  ChParallelDataManager *data_container;
  int3 num_bins_per_axis;
  real3 origin;
  real3 bin_size_vec;

 protected:
  real tolerance;

  uint max_iter_bilateral;

  real residual;

  custom_vector<real> rhs, debug, lambda;

  ChConstraintBilateral bilateral;
};

class CH_PARALLEL_API ChLcpSolverParallelDVI : public ChLcpSolverParallel {
 public:
  ChLcpSolverParallelDVI() {
    alpha = .2;
    contact_recovery_speed = .6;
    solver_type = APGD;
    do_stab = false;
    collision_inside = false;
    update_rhs=false;

    max_iteration = 1000;
    max_iter_normal = max_iter_sliding = max_iter_spinning = 100;
  }

  virtual void RunTimeStep(real step);
  void RunWarmStartPostProcess();
  void RunWarmStartPreprocess();

  void SetCompliance(real a) {
    data_container->alpha = a;
  }
  void SetSolverType(GPUSOLVERTYPE type) {
    solver_type = type;
  }
  void SetMaxIterationNormal(uint max_iter) {
    max_iter_normal = max_iter;
  }
  void SetMaxIterationSliding(uint max_iter) {
    max_iter_sliding = max_iter;
  }
  void SetMaxIterationSpinning(uint max_iter) {
    max_iter_spinning = max_iter;
  }
  void SetMaxIteration(uint max_iter) {
    max_iteration = max_iter;
    max_iter_normal = max_iter_sliding = max_iter_spinning = max_iter_bilateral = max_iter;
  }
  void SetContactRecoverySpeed(real recovery_speed) {
    data_container->contact_recovery_speed = fabs(recovery_speed);
  }
  void DoStabilization(bool stab) {
    do_stab = stab;
  }
  void DoCollision(bool do_collision) {
    collision_inside = do_collision;
  }
  void DoUpdateRHS(bool do_update_rhs) {
    update_rhs = do_update_rhs;
  }
  ChSolverParallel solver;

 private:
  GPUSOLVERTYPE solver_type;

  real alpha;

  real contact_recovery_speed;
  bool do_stab;
  bool collision_inside;
  bool update_rhs;

  uint max_iteration;
  uint max_iter_normal;
  uint max_iter_sliding;
  uint max_iter_spinning;

  ChConstraintRigidRigid rigid_rigid;
};

class CH_PARALLEL_API ChLcpSolverParallelDEM : public ChLcpSolverParallel {
 public:
  virtual void RunTimeStep(real step);

  void SetMaxIteration(uint max_iter) {
    max_iter_bilateral = max_iter;
  }

  void ProcessContacts();

 private:
  void host_CalcContactForces(custom_vector<int>& ext_body_id,
  custom_vector<real3>& ext_body_force,
  custom_vector<real3>& ext_body_torque);

  void host_AddContactForces(uint ct_body_count,
  const custom_vector<int>& ct_body_id,
  const custom_vector<real3>& ct_body_force,
  const custom_vector<real3>& ct_body_torque);

  ChSolverParallel solver;
};

}
  // end namespace chrono

#endif

