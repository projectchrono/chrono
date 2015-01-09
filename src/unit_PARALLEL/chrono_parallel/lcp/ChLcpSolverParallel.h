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

   virtual ~ChLcpSolverParallel() {}
   //Each child class must define its own solve method
   virtual double Solve(ChLcpSystemDescriptor &sysd) {return 0;}
   //Similarly, the run timestep function needs to be defined
   virtual void RunTimeStep(real step) = 0;
   //Compute the inverse mass matrix and the term v+M_inv*hf
   void ComputeMassMatrix();
   //Compute the jacobian matrix
   virtual void ComputeD() = 0;
   //Compute the compliance matrix
   virtual void ComputeE() = 0;
   //Compute the rhs matrix, depending on what type of solve is being performed
   //the RHS vector will have different non zero entries
   virtual void ComputeR(SOLVERMODE mode) = 0;
   //This function computes the new velocities based on the lagrange multipliers
   void ComputeImpulses();

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

   virtual void ComputeD();
   virtual void ComputeE();
   virtual void ComputeR(SOLVERMODE mode);

   void ChangeSolverType(SOLVERTYPE type);

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

};

}
// end namespace chrono

#endif

