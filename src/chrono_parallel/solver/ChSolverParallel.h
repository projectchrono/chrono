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
#include "chrono_parallel/physics/Ch3DOFContainer.h"
#include "chrono_parallel/constraints/ChConstraintBilateral.h"
#include "chrono_parallel/solver/ChShurProduct.h"

namespace chrono {

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

    // Compute rhs value with relaxation term
    void ComputeSRhs(custom_vector<real>& gamma,
                     const custom_vector<real>& rhs,
                     custom_vector<real3>& vel_data,
                     custom_vector<real3>& omg_data,
                     custom_vector<real>& b);

    // Call this function with an associated solver type to solve the system
    virtual uint Solve(ChShurProduct& ShurProduct,
                       const uint max_iter,           // Maximum number of iterations
                       const uint size,               // Number of unknowns
                       const DynamicVector<real>& b,  // Rhs vector
                       DynamicVector<real>& x         // The vector of unknowns
                       ) = 0;
    virtual void InnerSolve();

    real GetObjective(const DynamicVector<real>& x, const DynamicVector<real>& b) {}

    void AtIterationEnd(real maxd, real maxdeltalambda) {
        data_manager->measures.solver.maxd_hist.push_back(maxd);
        data_manager->measures.solver.maxdeltalambda_hist.push_back(maxdeltalambda);
    }

    real LargestEigenValue(ChShurProduct& ShurProduct, DynamicVector<real>& temp, real lambda = 0);

    int current_iteration;  // The current iteration number of the solver

    ChConstraintRigidRigid* rigid_rigid;
    ChConstraintBilateral* bilateral;
    Ch3DOFContainer* three_dof;
    Ch3DOFContainer* fem;
    Ch3DOFContainer* mpm;

    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;

    DynamicVector<real> eigen_vec;
};
}
