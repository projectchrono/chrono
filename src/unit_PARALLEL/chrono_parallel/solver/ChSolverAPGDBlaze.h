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
// This file contains an implementation of APGD that is more optimized.
// =============================================================================

#ifndef CHSOLVERAPGDBLAZE_H
#define CHSOLVERAPGDBLAZE_H

#include "chrono_parallel/ChConfigParallel.h"
#include "ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverAPGDBlaze : public ChSolverParallel {
 public:

   ChSolverAPGDBlaze()
         : ChSolverParallel() {

      //APGD specific
      step_shrink = .9;
      step_grow = 2.0;
      init_theta_k = 1.0;

   }
   ~ChSolverAPGDBlaze() {

   }

   void Solve() {
      if (num_constraints == 0) {
         return;
      }

      total_iteration += SolveAPGDBlaze(max_iteration, num_constraints, data_container->host_data.rhs_data, data_container->host_data.gamma_data);

      current_iteration = total_iteration;
   }

   // Solve using a more streamlined but harder to read version of the APGD method
   uint SolveAPGDBlaze(const uint max_iter,           // Maximum number of iterations
                       const uint size,               // Number of unknowns
                       custom_vector<real> &b,        // Rhs vector
                       custom_vector<real> &x         // The vector of unknowns
                       );

   // Compute the updated velocities
   // The solution for gamma is already here, so use it rather than having to copy it
   void ComputeImpulses();

   // Compute the residual for the solver
   // TODO: What is the best way to explain this...
   real Res4(const int SIZE,
             blaze::DynamicVector<real> & mg_tmp2,
             blaze::DynamicVector<real> & x,
             blaze::DynamicVector<real> & mb_tmp);

   // Set parameters for growing and shrinking the step size
   void SetAPGDParams(real theta_k,
                      real shrink,
                      real grow);

   //APGD specific vectors
   blaze::DynamicVector<real> obj2_temp, obj1_temp, ms, mg_tmp2, mb_tmp, mg_tmp, mg_tmp1, mg, ml, mx, my, ml_candidate, mb, mso;
   real L_k, t_k;
   real init_theta_k;
   real step_shrink;
   real step_grow;
   real old_objective;
   real lastgoodres;
   real theta_k;
   real theta_k1;
   real beta_k1;
   real mb_tmp_norm, mg_tmp_norm;
   real obj1, obj2;
   real dot_mg_ms, norm_ms;
   real delta_obj;

};
}
#endif
