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
#include "chrono_parallel/solver/ChSolverParallel.h"
#include "chrono_parallel/solver/ChSolverAPGD.h"
namespace chrono {

class CH_PARALLEL_API ChLcpSolverParallel : public ChLcpIterativeSolver {
 public:
   ChLcpSolverParallel();
   virtual ~ChLcpSolverParallel() {}

   //Each child class must define its own solve method
   virtual double Solve(ChLcpSystemDescriptor &sysd) {return 0;}
   //Similarly, the run timestep function needs to be defined
   virtual void RunTimeStep(real step) = 0;

   //Compute the inverse mass matrix and the term v+M_inv*hf
   void ComputeMassMatrix();
   //This function computes the new velocities based on the lagrange multipliers
   void ComputeImpulses();
   //Solves just the bilaterals so that they can be warm started
   void PerformStabilization();

   real GetResidual() { return residual; }
   ChParallelDataManager *data_container;
   ChSolverParallel *solver;

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

   // Compute the constraint Jacobian matrix.
   void ComputeD();
   // Compute the compliance matrix.
   void ComputeE();
   // Compute the RHS vector. Depending on what type of solve is being performed
   // the RHS vector will have different non zero entries.
   void ComputeR(SOLVERMODE mode);
   // This function is used to change the solver algorithm.
   void ChangeSolverType(SOLVERTYPE type);

 private:
   ChConstraintRigidRigid rigid_rigid;
};

class CH_PARALLEL_API ChLcpSolverParallelDEM : public ChLcpSolverParallel {
 public:
   ChLcpSolverParallelDEM() {
      solver = new ChSolverAPGD();
   }

   ~ChLcpSolverParallelDEM() {
     delete solver;
   }

  virtual void RunTimeStep(real step);

  // Compute the constraint Jacobian matrix.
  void ComputeD();
  // Compute the compliance matrix.
  void ComputeE();
  // Compute the RHS vector.
  void ComputeR();

  void ProcessContacts();

 private:
   void host_CalcContactForces(custom_vector<int>& ext_body_id,
                               custom_vector<real3>& ext_body_force,
                               custom_vector<real3>& ext_body_torque);

   void host_AddContactForces(uint ct_body_count,
                              const custom_vector<int>& ct_body_id,
                              const custom_vector<real3>& ct_body_force,
                              const custom_vector<real3>& ct_body_torque);
};

}
// end namespace chrono

#endif

