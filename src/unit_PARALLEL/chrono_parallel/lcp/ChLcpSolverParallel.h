// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: This class calls the parallel solver, used as an intermediate
// between chrono's solver interface and the parallel solver interface.
// =============================================================================

#ifndef CHLCPSOLVERPARALLEL_H
#define CHLCPSOLVERPARALLEL_H

#include "lcp/ChLcpIterativeSolver.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/ChIntegratorParallel.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/constraints/ChConstraintBilateral.h"
#include "chrono_parallel/math/ChParallelMath.h"

#include "chrono_parallel/solver/ChSolverAPGD.h"
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
namespace chrono {

class CH_PARALLEL_API ChLcpSolverParallel : public ChLcpIterativeSolver {
 public:
   ChLcpSolverParallel();

   virtual ~ChLcpSolverParallel() {
   }

   virtual double Solve(ChLcpSystemDescriptor &sysd) {
      return 0;
   }
   void host_addForces(bool* active,
                       real *mass,
                       real3 *inertia,
                       real3 *forces,
                       real3 *torques,
                       real3 *vel,
                       real3 *omega);

   void host_addForces_shafts(bool* active,
                              real* inertia,
                              real* torques,
                              real* omega);

   void host_ComputeGyro(real3 *omega,
                         real3 *inertia,
                         real3 *gyro,
                         real3 *torque);

   virtual void RunTimeStep(real step) = 0;
   void Preprocess();

   real GetResidual() {
      return residual;
   }
   ChParallelDataManager *data_container;

 protected:
   real residual;

   ChConstraintBilateral bilateral;
};

class CH_PARALLEL_API ChLcpSolverParallelDVI : public ChLcpSolverParallel {
 public:
   ChLcpSolverParallelDVI() {
      solver = new ChSolverAPGD();
   }

   ~ChLcpSolverParallelDVI() {
      delete solver;
   }

   virtual void RunTimeStep(real step);
   void ComputeN();

   void ChangeSolverType(SOLVERTYPE type) {
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
      }  else if (type == JACOBI) {
         solver = new ChSolverJacobi();
      } else if (type == GAUSS_SEIDEL) {
         solver = new ChSolverPGS();
      } else if (type == PDIP) {
         solver = new ChSolverPDIP();
      }
   }
   ChSolverParallel *solver;
 private:
   ChConstraintRigidRigid rigid_rigid;
};

class CH_PARALLEL_API ChLcpSolverParallelDEM : public ChLcpSolverParallel {
 public:

   ChLcpSolverParallelDEM() {
      solver = new ChSolverAPGD();
   }

   virtual void RunTimeStep(real step);

   void ProcessContacts();

 private:
   void host_CalcContactForces(custom_vector<int>& ext_body_id,
                               custom_vector<real3>& ext_body_force,
                               custom_vector<real3>& ext_body_torque);

   void host_AddContactForces(uint ct_body_count,
                              const custom_vector<int>& ct_body_id,
                              const custom_vector<real3>& ct_body_force,
                              const custom_vector<real3>& ct_body_torque);

   ChSolverParallel * solver;
};

}
// end namespace chrono

#endif

