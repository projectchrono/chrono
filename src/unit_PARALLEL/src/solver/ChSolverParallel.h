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
// This file contains the base class used for all parallel iterative solvers.
// All of the functions are defined here, with the implementation of each solver
// in it's specific cpp file.
// =============================================================================
#ifndef CHSOLVERPARALLEL_H
#define CHSOLVERPARALLEL_H

#include "ChParallelDefines.h"
#include "ChBaseParallel.h"
#include "ChDataManager.h"
#include "math/ChParallelMath.h"
#include "math/ChThrustLinearAlgebra.h"
#include "core/ChTimer.h"
#include "constraints/ChConstraintRigidRigid.h"
#include "constraints/ChConstraintBilateral.h"
#include "collision/ChCNarrowphaseMPR.h"

namespace chrono {
class ChApiGPU ChSolverParallel : public ChBaseParallel {
 public:
   ChSolverParallel() {
      tolerance = 1e-6;
      epsilon = 1e-3;
      alpha = .2;
      max_iteration = 100;
      total_iteration = 0;
      current_iteration = 0;
      collision_inside = false;
      rigid_rigid = NULL;
      bilateral = NULL;
   }

   // Set the data container and step size for the solver, run the setup function.
   // Depending on how the solver was created, this needs to be run every step.
   void Initial(
         real step,
         ChParallelDataManager *data_container_  //
         );

   // At the beginning of the step reset the size/indexing variables,
   // resize for new contact list and clear temporary accumulation variables
   void Setup();

   // Project the lagrange multipliers (gamma) onto the friciton cone.
   void Project(
         real* gamma  //Lagrange Multipliers
         );

   // Compute the first half of the shur matrix vector multiplication (N*x)
   // Perform M_invDx=M^-1*D*x
   void shurA(
         real*x  //Vector that N is multiplied by
         );

   // Compute second half of shur matrix vector multiplication Nx=D*M_invDx
   void shurB(
         real* x,   //Vector that contains M_invDx
         real* out  //N*x
         );

   // Perform M^-1*D*gamma and increment the linear and rotational velocity vectors
   void ComputeImpulses();

   // Function that performs time integration to get the new positions
   // Used when contacts need to be updated within the solver
   // Function is similar to compute impulses
   void UpdatePosition(
         custom_vector<real>& x   //Lagrange multipliers
         );

   // Rerun the narrowphase to get the new contact list, broadphase is not run again here
   // This assumes that the positions did not drastically change.
   void UpdateContacts();

   // Compute the full shur matrix vector product (N*x) where N=D^T*M^-1*D
   void ShurProduct(
         custom_vector<real>& x,  //Vector that will be multiplied by N
         custom_vector<real>& AX  //Output Result
         );

   // Compute the shur matrix vector product only for the bilaterals (N*x)
   // where N=D^T*M^-1*D
   void ShurBilaterals(
         custom_vector<real> &x,   //Vector that will be multiplied by N
         custom_vector<real> & AX  //Output Result
         );

   // Call this function with an associated solver type to solve the system
   void Solve(
         GPUSOLVERTYPE solver_type
         );

   // Perform velocity stabilization on bilateral constraints
   uint SolveStab(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using the steepest descent method
   uint SolveSD(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using the gradient descent method
   uint SolveGD(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using the conjugate gradient method
   uint SolveCG(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using the conjugate gradient squared method
   uint SolveCGS(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using the bi-conjugate gradient method
   uint SolveBiCG(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter);

   // Solve using the bi-conjugate gradient method with stabilization
   uint SolveBiCGStab(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using the minimum residual method
   uint SolveMinRes(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using the Accelerated Projected Gradient Descent Method
   uint SolveAPGD(
         custom_vector<real> &x,
         const custom_vector<real> &b,
         const uint max_iter
         );

   // Solve using a more streamlined but harder to read version of the APGD method
   uint SolveAPGDRS(
         custom_vector<real> &x,
         custom_vector<real> &b,
         const uint max_iter,
         const int SIZE
         );

   // Initialize the APGD method so that variables are not recreated
   void InitAPGD(
         custom_vector<real> &x);

   // Compute the residual for the solver
   // TODO: What is the best way to explain this...
   real Res4(
         const int SIZE,
         real* mg_tmp,
         const real* b,
         real*x,
         real* mb_tmp
         );

   // Set parameters for growing and shrinking the step size
   void SetAPGDParams(
         real theta_k,
         real shrink,
         real grow
         );
   // Get the number of iterations perfomed by the solver
   int GetIteration() {
      return current_iteration;
   }
   // Get the current residual
   real GetResidual() {
      return residual;
   }

   void AtIterationEnd(
         real maxd,
         real maxdeltalambda,
         int iter) {
      maxd_hist.push_back(maxd);
      maxdeltalambda_hist.push_back(maxdeltalambda);
      iter_hist.push_back(iter);
   }

   // Set the tolerance for all solvers
   void SetTolerance(
         const real tolerance_value) {
      tolerance = tolerance_value;
   }

   // Set the maximum number of iterations for all solvers
   void SetMaxIterations(
         const int max_iteration_value) {
      max_iteration = max_iteration_value;
   }

   // Set the compliance alpha value
   void SetComplianceAlpha(
         const real alpha_value) {
      alpha = alpha_value;
   }
   //Set the maximum contact recovery speed, used by RHS computation
   void SetContactRecoverySpeed(
         const real & recovery_speed) {
      contact_recovery_speed = recovery_speed;
   }

   int current_iteration, max_iteration, total_iteration;
   real residual, epsilon, tolerance;

   thrust::host_vector<real> maxd_hist, maxdeltalambda_hist, iter_hist;

   custom_vector<real> temp1, temp2;
   custom_vector<real> obj2_temp, obj1_temp, lm;
   custom_vector<real> ms, mg_tmp2, mb_tmp, mg_tmp, mg_tmp1;
   custom_vector<real> mg, ml, mx, my, ml_candidate;

   real init_theta_k;
   real step_shrink;
   real step_grow;

   ChConstraintRigidRigid *rigid_rigid;
   ChConstraintBilateral *bilateral;

   bool do_stab;
   bool collision_inside;
};
}

#endif
