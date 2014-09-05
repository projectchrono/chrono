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
// This file contains the base class used for all parallel iterative solvers.
// All of the functions are defined here, with the implementation of each solver
// in it's specific cpp file.
// =============================================================================
#ifndef CHSOLVERPARALLEL_H
#define CHSOLVERPARALLEL_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChBaseParallel.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/constraints/ChConstraintBilateral.h"
#include "chrono_parallel/collision/ChCNarrowphaseMPR.h"
namespace chrono {
class CH_PARALLEL_API ChSolverParallel : public ChBaseParallel {
 public:
   ChSolverParallel();
   ~ChSolverParallel();

   // At the beginning of the step reset the size/indexing variables,
   // resize for new contact list and clear temporary accumulation variables
   void Setup(ChParallelDataManager *data_container_  //pointer to data container
   );

   // Project the lagrange multipliers (gamma) onto the friction cone.
   void Project(real* gamma  //Lagrange Multipliers
   );

   // Project a single contact
   void Project_Single(int index,
                       real* gamma);

   // Compute the first half of the shur matrix vector multiplication (N*x)
   // Perform M_invDx=M^-1*D*x
   void shurA(real*x  //Vector that N is multiplied by
   );

   // Compute second half of shur matrix vector multiplication Nx=D*M_invDx
   void shurB(real* x,   //Vector that contains M_invDx
              real* out  //N*x
              );

   // Compute rhs value with relaxation term
   void ComputeSRhs(
                    custom_vector<real>& gamma,
                    const custom_vector<real>& rhs,
                    custom_vector<real3>& vel_data,
                    custom_vector<real3>& omg_data,
                    custom_vector<real>& b);

   // Perform M^-1*D*gamma and increment the linear and rotational velocity vectors
   void ComputeImpulses();

   // Perform M^-1*D*gamma and compute the linear and rotational velocity vectors
   void ComputeImpulses(
                        custom_vector<real>& gamma,
                        custom_vector<real3>& vel_data,
                        custom_vector<real3>& omg_data);

   // Function that performs time integration to get the new positions
   // Used when contacts need to be updated within the solver
   // Function is similar to compute impulses
   void UpdatePosition(custom_vector<real>& x   //Lagrange multipliers
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
   virtual void Solve()=0;

   // Perform velocity stabilization on bilateral constraints
   uint SolveStab(const uint max_iter,           // Maximum number of iterations
                  const uint size,               // Number of unknowns
                  const custom_vector<real> &b,  // Rhs vector
                  custom_vector<real> &x         // The vector of unknowns
                  );

   // Get the number of iterations perfomed by the solver
   int GetIteration() {
      return current_iteration;
   }
   // Get the current residual
   real GetResidual() {
      return residual;
   }
   real GetObjective(custom_vector<real> & x,
                     custom_vector<real> & b) {
      custom_vector<real> Nl(x.size());
      // f_p = 0.5*l_candidate'*N*l_candidate - l_candidate'*b  = l_candidate'*(0.5*Nl_candidate - b);
      ShurProduct(x, Nl);     // 1)  g_tmp = N*l_candidate ...        #### MATR.MULTIPLICATION!!!###
      SEAXMY(0.5, Nl, b, Nl);  // 2) 0.5*N*l_candidate-b_shur
      return Dot(x, Nl);      // 3)  mf_p  = l_candidate'*(0.5*N*l_candidate-b_shur)

   }
   real GetObjectiveBlaze(blaze::DynamicVector<real> & x,
                          blaze::DynamicVector<real> & b) {
      blaze::DynamicVector<real> Nl(x.size());
      // f_p = 0.5*l_candidate'*N*l_candidate - l_candidate'*b  = l_candidate'*(0.5*Nl_candidate - b);
      Nl = data_container->host_data.Nshur * x;  // 1)  g_tmp = N*l_candidate ...        #### MATR.MULTIPLICATION!!!###
      Nl = 0.5 * Nl - b;          // 2) 0.5*N*l_candidate-b_shur
      return (x, Nl);            // 3)  mf_p  = l_candidate'*(0.5*N*l_candidate-b_shur)
   }
   real GetObjective() {
      custom_vector<real> Nl(data_container->host_data.gamma_data.size());
      // f_p = 0.5*l_candidate'*N*l_candidate - l_candidate'*b  = l_candidate'*(0.5*Nl_candidate - b);
      ShurProduct(data_container->host_data.gamma_data, Nl);    // 1)  g_tmp = N*l_candidate ...        #### MATR.MULTIPLICATION!!!###
      SEAXMY(0.5, Nl, data_container->host_data.rhs_data, Nl);  // 2) 0.5*N*l_candidate-b_shur
      return Dot(data_container->host_data.gamma_data, Nl);     // 3)  obj  = l_candidate'*(0.5*N*l_candidate-b_shur)

   }
   real Res4Blaze(blaze::DynamicVector<real> & x,
                  blaze::DynamicVector<real> & b) {
      real gdiff = .1;
      blaze::DynamicVector<real> inside = x - gdiff * (data_container->host_data.Nshur * x - b);
      Project(inside.data());
      blaze::DynamicVector<real> temp = (x - inside) / (x.size() * gdiff);
      return sqrt((temp, temp));
   }

   real GetKE() {
      real kinetic_energy = 0;

      for (int i = 0; i < data_container->num_bodies; i++) {
         real v = length(data_container->host_data.vel_data[i]);
         kinetic_energy += 0.5 * 1.0 / data_container->host_data.mass_data[i] * v * v;
      }
      return kinetic_energy;

   }

   void AtIterationEnd(real maxd,
                       real maxdeltalambda,
                       int iter) {
      maxd_hist.push_back(maxd);
      maxdeltalambda_hist.push_back(maxdeltalambda);
      iter_hist.push_back(iter);
   }

   // Set the tolerance for all solvers
   void SetTolerance(const real tolerance_value) {
      tolerance = tolerance_value;
   }

   // Set the maximum number of iterations for all solvers
   void SetMaxIterations(const int max_iteration_value) {
      max_iteration = max_iteration_value;
   }

   int current_iteration;  // The current iteration number of the solver
   int max_iteration;     // The maximum number of iterations that the solver will perform
   int total_iteration;   // The total number of iterations performed, this variable accumulates
   real residual;         // Current residual for the solver
   real tolerance;        // Solver tolerance
   real objective_value;
   //These three variables are used to store the convergence history of the solver
   thrust::host_vector<real> maxd_hist, maxdeltalambda_hist, iter_hist;

   bool do_stab;           //There is an alternative velocity stab that can be performed in APGD, this enables that.
   bool collision_inside;
   bool update_rhs;        //Updates the tilting term within the solve
   bool verbose;
   bool tol_objective;
   ChConstraintRigidRigid *rigid_rigid;
   ChConstraintBilateral *bilateral;

};
}

#endif
