// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
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

namespace chrono {

/// @addtogroup parallel_solver
/// @{

/// Functor class for performing projection on the hyper-cone.
class CH_PARALLEL_API ChProjectConstraints {
  public:
    ChProjectConstraints() {}
    virtual ~ChProjectConstraints() {}

    virtual void Setup(ChParallelDataManager* data_container_) { data_manager = data_container_; }

    // Project the Lagrange multipliers
    virtual void operator()(real* data);

    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;
};

/// Functor class for performing a single cone projection.
class CH_PARALLEL_API ChProjectNone : public ChProjectConstraints {
  public:
    ChProjectNone() {}
    virtual ~ChProjectNone() {}

    // Project the Lagrange multipliers
    virtual void operator()(real* data) {}
};

/// Functor class for calculating the Shur product of the matrix of unilateral constraints.
class CH_PARALLEL_API ChShurProduct {
  public:
    ChShurProduct();
    virtual ~ChShurProduct() {}

    virtual void Setup(ChParallelDataManager* data_container_) { data_manager = data_container_; }

    // Perform the Shur Product
    virtual void operator()(const DynamicVector<real>& x, DynamicVector<real>& AX);

    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;
};

/// Functor class for performing the Shur product of the matrix of bilateral constraints.
class CH_PARALLEL_API ChShurProductBilateral : public ChShurProduct {
  public:
    ChShurProductBilateral() {}
    virtual ~ChShurProductBilateral() {}
    virtual void Setup(ChParallelDataManager* data_container_);

    // Perform the Shur Product
    virtual void operator()(const DynamicVector<real>& x, DynamicVector<real>& AX);

    CompressedMatrix<real> NshurB;
};

/// Functor class for performing the Shur product.
class CH_PARALLEL_API ChShurProductFEM : public ChShurProduct {
  public:
    ChShurProductFEM() {}
    virtual ~ChShurProductFEM() {}
    virtual void Setup(ChParallelDataManager* data_container_);

    // Perform the Shur Product
    virtual void operator()(const DynamicVector<real>& x, DynamicVector<real>& AX);

    CompressedMatrix<real> NshurB;
};

//========================================================================================================

/// Base class for all Chrono::Parallel solvers.
class CH_PARALLEL_API ChSolverParallel {
  public:
    ChSolverParallel();
    virtual ~ChSolverParallel() {}

    void Setup(ChParallelDataManager* data_container_) { data_manager = data_container_; }

    // Compute rhs value with relaxation term
    void ComputeSRhs(custom_vector<real>& gamma,
                     const custom_vector<real>& rhs,
                     custom_vector<real3>& vel_data,
                     custom_vector<real3>& omg_data,
                     custom_vector<real>& b);

    // Call this function with an associated solver type to solve the system
    virtual uint Solve(ChShurProduct& ShurProduct,
                       ChProjectConstraints& Project,
                       const uint max_iter,           // Maximum number of iterations
                       const uint size,               // Number of unknowns
                       const DynamicVector<real>& b,  // Rhs vector
                       DynamicVector<real>& x         // The vector of unknowns
                       ) = 0;

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

//========================================================================================================

/// Accelerated Projected Gradient Descent (APGD) solver. Reference implementation.
class CH_PARALLEL_API ChSolverParallelAPGDREF : public ChSolverParallel {
  public:
    ChSolverParallelAPGDREF() : ChSolverParallel() {}
    ~ChSolverParallelAPGDREF() {}

    // Solve using the APGD method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& r,  // Rhs vector
               DynamicVector<real>& gamma     // The vector of unknowns
               );

    // Compute the residual for the solver
    real Res4(ChShurProduct& ShurProduct,
              ChProjectConstraints& Project,
              DynamicVector<real>& gamma,
              const DynamicVector<real>& r,
              DynamicVector<real>& tmp);

    // APGD specific vectors
    DynamicVector<real> gamma_hat;
    DynamicVector<real> gammaNew, g, y, yNew, tmp;
};

/// Accelerated Projected Gradient Descent (APGD) solver.
class CH_PARALLEL_API ChSolverParallelAPGD : public ChSolverParallel {
  public:
    ChSolverParallelAPGD();
    ~ChSolverParallelAPGD() {}

    // Solve using a more streamlined but harder to read version of the APGD method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );

    void UpdateR();

    // APGD specific vectors
    DynamicVector<real> obj2_temp, obj1_temp, temp, g, gamma_new, y, gamma_hat, N_gamma_new, _t_g;
    real L, t;
    real g_diff;
    real theta, theta_new, beta_new;
    real mb_tmp_norm, mg_tmp_norm;
    real obj1, obj2;
    real dot_g_temp, norm_ms;
};

/// Barzilai-Borwein solver.
class CH_PARALLEL_API ChSolverParallelBB : public ChSolverParallel {
  public:
    ChSolverParallelBB();
    ~ChSolverParallelBB() {}

    // Solve using a more streamlined but harder to read version of the BB method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );

    void UpdateR();

    // BB specific vectors
    DynamicVector<real> temp, ml, mg, mg_p, ml_candidate, ms, my, mdir, ml_p;
    DynamicVector<real> mD, invmD;
};

/// MINRES solver.
class CH_PARALLEL_API ChSolverParallelMinRes : public ChSolverParallel {
  public:
    ChSolverParallelMinRes() : ChSolverParallel() {}
    ~ChSolverParallelMinRes() {}

    // Solve using the minimal residual method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );

    DynamicVector<real> v, v_hat, w, w_old, xMR, v_old, Av, w_oold;
};

/// Spectral Projected Gradient solver.
class CH_PARALLEL_API ChSolverParallelSPGQP : public ChSolverParallel {
  public:
    ChSolverParallelSPGQP();
    ~ChSolverParallelSPGQP() {}

    // Solve using a more streamlined but harder to read version of the BB method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );

    void UpdateR();

    // BB specific vectors
    real alpha, f_max, xi, beta_bar, beta_tilde, beta_k, gam;
    DynamicVector<real> g, d_k, x, temp, Ad_k, g_alpha, x_candidate;
    std::vector<real> f_hist;
};

/// Conjugate gradient solver.
class CH_PARALLEL_API ChSolverParallelCG : public ChSolverParallel {
  public:
    ChSolverParallelCG() : ChSolverParallel() {}
    ~ChSolverParallelCG() {}

    // Solve using the conjugate gradient method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );

    DynamicVector<real> r, q, s;
};

/// Jacobi solver.
class CH_PARALLEL_API ChSolverParallelJacobi : public ChSolverParallel {
  public:
    ChSolverParallelJacobi() {}
    ~ChSolverParallelJacobi() {}

    // Solve using a more streamlined but harder to read version of the BB method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );
    DynamicVector<real> ml_old, ml;
};

/// Gauss Seidel solver.
class CH_PARALLEL_API ChSolverParallelGS : public ChSolverParallel {
  public:
    ChSolverParallelGS() {}
    ~ChSolverParallelGS() {}

    // Solve using a more streamlined but harder to read version of the BB method
    uint Solve(ChShurProduct& ShurProduct,
               ChProjectConstraints& Project,
               const uint max_iter,           // Maximum number of iterations
               const uint size,               // Number of unknowns
               const DynamicVector<real>& b,  // Rhs vector
               DynamicVector<real>& x         // The vector of unknowns
               );
    DynamicVector<real> ml_old, ml;
};

/// @} parallel_solver

} // end namespace chrono
