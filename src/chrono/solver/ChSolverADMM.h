// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHSOLVERADMM_H
#define CHSOLVERADMM_H

#include "chrono/solver/ChIterativeSolverVI.h"
#include "chrono/solver/ChDirectSolverLS.h"

namespace chrono {

/// @addtogroup chrono_solver
/// @{

/// An iterative solver based on modified version of ADMM Alternating Direction Method of Multipliers.
///
/// See ChSystemDescriptor for more information about the problem formulation and the data structures passed to the
/// solver.
class ChApi ChSolverADMM : public ChIterativeSolverVI {
  public:

    /// Default constructor: uses the SparseQR direct solver from Eigine, slow and UNOPTIMAL.
    /// Prefer the other constructor where you can pass the MKL direct solver.
    ChSolverADMM();

    /// Constructor where you can pass a better direct solver than the
    /// default SparseQR solver. The custom direct solver will be
    /// used in the factorization and solves in the inner loop.
    ChSolverADMM(std::shared_ptr<ChDirectSolverLS> my_LS_engine);

    ~ChSolverADMM() {}

    virtual Type GetType() const override { return Type::ADDM; }

    /// Performs the solution of the problem.
    /// \return  the maximum constraint violation after termination, as  dual (speed) residual
    virtual double Solve(ChSystemDescriptor& sysd  ///< system description with constraints and variables
                         ) override;

    /// Enable the diagonal preconditioner
    void SetDiagonalPreconditioner(bool mp) { precond = mp; }
    bool GetDiagonalPreconditioner() { return precond; }

    /// Set the initial ADMM step. Could change later if adaptive step is used.
    void SetRho(double mr) { rho = mr; }
    double GetRho() { return rho; }

    /// Set the ADMM step for bilateral constraints only.
    void SetRhoBilaterals(double mr) { rho_b = mr; }
    double GetRhoBilaterals() { return rho_b; }

    /// Set the sigma regularization.
    void SetSigma(double mr) { sigma = mr; }
    double GetSigma() { return sigma; }

    /// Set how frequently the rho step is adjusted.
    void SetStepAdjustEach(int mr) { stepadjust_each = mr; }
    int GetStepAdjustEach() { return stepadjust_each; }

    /// Set the step adjust threshold T, ex. 1.5; if new step scaling is in
    /// the interval [1/T, T], no adjustment is done anyway, to save CPU effort.
    void SetStepAdjustThreshold(double mr) { stepadjust_threshold = mr; }
    double GetStepAdjustThreshold() { return stepadjust_threshold; }

    /// Set the step adjust max factor F, if new step scaling is higher than F
    /// it is clamped to F, if lower than 1/F it is clamped to F, to avoid exploding results.
    void SetStepAdjustMaxfactor(double mr) { stepadjust_maxfactor = mr; }
    double GetStepAdjustMaxfactor() { return stepadjust_maxfactor; }

    /// Types of step adaption policies for the ADMM solver
    enum  AdmmStepType {
        NONE = 0,            
        BALANCED_UNSCALED,
        BALANCED_FAST,
        BALANCED_RANGE
    };

    /// Set the step adjust max factor F, if new step scaling is higher than F
    /// it is clamped to F, if lower than 1/F it is clamped to F, to avoid exploding results.
    void SetStepAdjustPolicy(AdmmStepType mr) { stepadjust_type = mr; }
    AdmmStepType GetStepAdjustPolicy() { return stepadjust_type; }

    /// Types of ADMM enhancements
    enum  AdmmAcceleration {
        BASIC = 0,      // basic ADMM        
        NESTEROV,       // inspired to  FAST ALTERNATING DIRECTION OPTIMIZATION METHODS, 2015, T. Goldstein et al.
    };
    /// Set the type of ADMM iteration, enabling acceleration variants
    void   SetAcceleration(AdmmAcceleration mr) { acceleration = mr; }
    AdmmAcceleration GetAcceleration() { return acceleration; }

    /// Set the absolute tolerance for the primal residual (force impulses error), if 
    /// the iteration falls below this and the dual tolerance, it stops iterating.
    void SetTolerancePrimal(double mr) { tol_prim = mr; }
    double GetTolerancePrimal() { return tol_prim; }

    /// Set the absolute tolerance for the dual residual (constraint speed error), if
    /// the iteration falls below this and the primal tolerance, it stops iterating.
    void SetToleranceDual(double mr) { tol_dual = mr; }
    double GetToleranceDual() { return tol_dual; }

    /// Return the primal residual (force impulses error) reached during the last solve.
    double GetErrorPrimal() const { return r_prim; }

    /// Return the dual residual (constraint speed error) reached during the last solve.
    double GetErrorDual() const { return r_dual; }

    /// Return the number of iterations performed during the last solve.
    // virtual int GetIterations() const = 0;

    /// Return the tolerance error reached during the last solve (here refer to the ADMM dual residual)
    virtual double GetError() const override { return r_dual; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    double r_prim;
    double r_dual;
    bool precond;
    double rho;
    double rho_b;
    double sigma;
    int stepadjust_each;
    double stepadjust_threshold;
    double stepadjust_maxfactor;
    AdmmStepType stepadjust_type;
    double tol_prim;
    double tol_dual;
    AdmmAcceleration acceleration;

    std::shared_ptr<ChDirectSolverLS> LS_solver;
    //Eigen::SparseQR<ChSparseMatrix, Eigen::COLAMDOrdering<int>> m_engine;  ///< Eigen SparseQR solver (do not use SparseLU: it is broken!)
    // SparseLU: it is broken!)
    /// Performs basic ADMM, as in solve_kkt_ADMMbasic.m prototype 
    virtual double _SolveBasic(ChSystemDescriptor& sysd);

    /// Performs ADMM with Nesterov acceleration, as in solve_kkt_ADMMfast.m prototype 
    virtual double _SolveFast(ChSystemDescriptor& sysd);

};

/// @} chrono_solver

}  // end namespace chrono

#endif
