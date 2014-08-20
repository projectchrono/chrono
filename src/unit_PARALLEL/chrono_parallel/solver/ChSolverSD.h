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
// This file contains an implementation of an iterative Steepest Descent solver.
// =============================================================================


#ifndef CHSOLVERSD_H
#define CHSOLVERSD_H

#include "chrono_parallel/ChConfigParallel.h"
#include "ChSolverParallel.h"

namespace chrono {
class CH_PARALLEL_API ChSolverSD : public ChSolverParallel {
 public:

   ChSolverSD()
         :
           ChSolverParallel() {

   }
   ~ChSolverSD() {

   }

   void Solve() {
      if (num_constraints == 0) {return;}
      data_container->system_timer.start("ChSolverParallel_Solve");
      total_iteration += SolveSD(max_iteration, num_constraints, data_container->host_data.rhs_data, data_container->host_data.gamma_data);
      data_container->system_timer.stop("ChSolverParallel_Solve");
      current_iteration = total_iteration;
   }
   // Solve using the Accelerated Projected Gradient Descent Method
   uint SolveSD(
                const uint max_iter,           // Maximum number of iterations
                const uint size,               // Number of unknowns
                const custom_vector<real> &b,  // Rhs vector
                custom_vector<real> &x         // The vector of unknowns
                );

   custom_vector<real> r,temp;
};
}

#endif
