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
#pragma once

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/constraints/ChConstraintRigidFluid.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluid.h"
#include "chrono_parallel/constraints/ChConstraintBilateral.h"

namespace chrono {

/// @addtogroup parallel_module
/// @{

class CH_PARALLEL_API ChSolverParallel {
 public:
  ChSolverParallel();
  virtual ~ChSolverParallel() {}

  void Setup(ChParallelDataManager* data_container_) { data_manager = data_container_; }

  // Project the Lagrange multipliers
  void Project(real* gamma);  // Lagrange Multipliers

  // Project a single lagrange multiplier
  void Project_Single(int index,     // index
                      real* gamma);  // Lagrange Multipliers

  // Compute the first half of the shur matrix vector multiplication (N*x)
  // Perform M_invDx=M^-1*D*x
  void shurA(DynamicVector<real>& x, DynamicVector<real>& out);  // Vector that N is multiplied by

  // Compute rhs value with relaxation term
  void ComputeSRhs(custom_vector<real>& gamma,
                   const custom_vector<real>& rhs,
                   custom_vector<real3>& vel_data,
                   custom_vector<real3>& omg_data,
                   custom_vector<real>& b);

  // Function that performs time integration to get the new positions
  // Used when contacts need to be updated within the solver
  // Function is similar to compute impulses
  // Currently not supported
  void UpdatePosition(custom_vector<real>& x);  // Lagrange multipliers

  // Rerun the narrowphase to get the new contact list, broadphase is not run
  // again here. This assumes that the positions did not drastically change.
  void UpdateContacts();

  // Compute the full shur matrix vector product (N*x) where N=D^T*M^-1*D
  void ShurProduct(const DynamicVector<real>& x,  // Vector that will be multiplied by N
                   DynamicVector<real>& AX);      // Output Result

  // Compute the shur matrix vector product only for the bilaterals (N*x)
  // where N=D^T*M^-1*D
  void ShurBilaterals(const DynamicVector<real>& x, DynamicVector<real>& output);

  // Call this function with an associated solver type to solve the system
  virtual void Solve() = 0;

  // Perform velocity stabilization on bilateral constraints
  uint SolveStab(const uint max_iter,                                         // Maximum number of iterations
                 const uint size,                                             // Number of unknowns
                 const ConstSubVectorType& b,  // Rhs vector
                 SubVectorType& x);             // The vector of unknowns

  real GetObjective(const DynamicVector<real>& x, const DynamicVector<real>& b) {
    DynamicVector<real> Nl(x.size());
    ShurProduct(x, Nl);  // 1)  g_tmp = N*l_candidate
    Nl = 0.5 * Nl - b;   // 2) 0.5*N*l_candidate-b_shur
    return (x, Nl);      // 3)  mf_p  = l_candidate'*(0.5*N*l_candidate-b_shur)
  }

  real Res4Blaze(DynamicVector<real>& x, DynamicVector<real>& b) {
    // The gdiff parameter should scale with the number of constraints
    real gdiff = 1.0 / pow(x.size(), 2.0);
    DynamicVector<real> temp;
    ShurProduct(x, temp);
    DynamicVector<real> inside = x - gdiff * (temp - b);
    Project(inside.data());
    temp = (1.0 / gdiff) * (x - inside);
    return Sqrt((real)(temp, temp));
  }

  void AtIterationEnd(real maxd, real maxdeltalambda) {
    data_manager->measures.solver.maxd_hist.push_back(maxd);
    data_manager->measures.solver.maxdeltalambda_hist.push_back(maxdeltalambda);
  }

  // Set the maximum number of iterations for all solvers
  void SetMaxIterations(const int max_iteration_value) { max_iteration = max_iteration_value; }

  // The maximum number of iterations that the solver will perform
  // This is local to a solver because it can be changed depending on what is
  // being solved
  int max_iteration;
  int current_iteration;  // The current iteration number of the solver

  ChConstraintRigidRigid* rigid_rigid;
  ChConstraintBilateral* bilateral;

  // Pointer to the system's data manager
  ChParallelDataManager* data_manager;
};

/// @} parallel_module
}
