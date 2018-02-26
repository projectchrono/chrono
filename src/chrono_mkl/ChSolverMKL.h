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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#ifndef CHSOLVERMKL_H
#define CHSOLVERMKL_H

#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChSparseMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChSystemDescriptor.h"

#include "chrono/core/ChCSMatrix.h"
#include "chrono_mkl/ChMklEngine.h"

namespace chrono {

/// @addtogroup mkl_module
/// @{

/** \class ChSolverMKL
\brief Class that wraps the Intel MKL Pardiso parallel direct solver.

Sparse linear direct solver.
Cannot handle VI and complementarity problems, so it cannot be used with NSC formulations.

The solver is equipped with two main features:
- sparsity pattern lock
- sparsity pattern learning

The sparsity pattern \e lock enables the equivalent feature on the underlying matrix (if supported) and
is intended to be used when the sparsity pattern of the matrix does not undergo significant changes from call to call.\n
Is controlled by #SetSparsityPatternLock();

The sparsity pattern \e learning feature acquires the sparsity pattern in advance, in order to speed up
the first build of the matrix.\n
Is controlled by #ForceSparsityPatternUpdate();

A further option allows the user to manually set the number of non-zeros of the underlying matrix.<br>
This option will \e overrides the sparsity pattern lock.

<div class="ce-warning">
If the sparsity pattern \e learning is enabled, then is \e highly recommended to enable also the sparsity pattern \e
lock.
</div>

Minimal usage example, to be put anywhere in the code, before starting the main simulation loop:
\code{.cpp}
auto mkl_solver = std::make_shared<ChSolverMKL<>>();
application.GetSystem()->SetSolver(mkl_solver);
\endcode

See ChSystemDescriptor for more information about the problem formulation and the data structures
passed to the solver.

\tparam Matrix type of the matrix used by the solver;
*/
template <typename Matrix = ChCSMatrix>
class ChSolverMKL : public ChSolver {
  public:
    ChSolverMKL() { SetSparsityPatternLock(true); }

    ~ChSolverMKL() override {}

    /// Get a handle to the underlying MKL engine.
    ChMklEngine& GetMklEngine() { return m_engine; }

    /// Get a handle to the underlying matrix.
    Matrix& GetMatrix() { return m_mat; }

    /// Enable/disable locking the sparsity pattern (default: false).\n
    /// If \a val is set to true, then the sparsity pattern of the problem matrix is assumed
    /// to be unchanged from call to call.
    void SetSparsityPatternLock(bool val) {
        m_lock = val;
        m_mat.SetSparsityPatternLock(m_lock);
    }

    /// Call an update of the sparsity pattern on the underlying matrix.\n
    /// It is used to inform the solver (and the underlying matrices) that the sparsity pattern is changed.\n
    /// It is suggested to call this function just after the construction of the solver.
    /// \remark Turn on the sparsity pattern lock feature #SetSparsityPatternLock(); otherwise performance can be
    /// compromised.
    void ForceSparsityPatternUpdate(bool val = true) { m_force_sparsity_pattern_update = val; }

    /// Enable/disable use of permutation vector (default: false).
    void UsePermutationVector(bool val) { m_use_perm = val; }

    /// Enable/disable leveraging sparsity in right-hand side vector (default: false).
    void LeverageRhsSparsity(bool val) { m_use_rhs_sparsity = val; }

    /// Set the parameter that controls preconditioned CGS.
    void SetPreconditionedCGS(bool val, int L) { m_engine.SetPreconditionedCGS(val, L); }

    /// Set the number of non-zero entries in the problem matrix.
    void SetMatrixNNZ(int nnz) { m_nnz = nnz; }

    /// Reset timers for internal phases in Solve and Setup.
    void ResetTimers() {
        m_timer_setup_assembly.reset();
        m_timer_setup_solvercall.reset();
        m_timer_solve_assembly.reset();
        m_timer_solve_solvercall.reset();
    }

    /// Get cumulative time for assembly operations in Solve phase.
    double GetTimeSolve_Assembly() const { return m_timer_solve_assembly(); }
    /// Get cumulative time for Pardiso calls in Solve phase.
    double GetTimeSolve_SolverCall() const { return m_timer_solve_solvercall(); }
    /// Get cumulative time for assembly operations in Setup phase.
    double GetTimeSetup_Assembly() const { return m_timer_setup_assembly(); }
    /// Get cumulative time for Pardiso calls in Setup phase.
    double GetTimeSetup_SolverCall() const { return m_timer_setup_solvercall(); }
    /// Return the number of calls to the solver's Setup function.
    int GetNumSetupCalls() const { return m_setup_call; }
    /// Return the number of calls to the solver's Setup function.
    int GetNumSolveCalls() const { return m_solve_call; }

    /// Indicate whether or not the #Solve() phase requires an up-to-date problem matrix.
    /// As typical of direct solvers, the Pardiso solver only requires the matrix for its #Setup() phase.
    virtual bool SolveRequiresMatrix() const override { return false; }

    /// Perform the solver setup operations.
    /// For the MKL solver, this means assembling and factorizing the system matrix.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override {
        m_timer_setup_assembly.start();

        // Calculate problem size at first call.
        if (m_setup_call == 0) {
            m_dim = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
        }

        // Let the matrix acquire the information about ChSystem
        if (m_force_sparsity_pattern_update) {
            m_force_sparsity_pattern_update = false;

            ChSparsityPatternLearner sparsity_learner(m_dim, m_dim, true);
            sysd.ConvertToMatrixForm(&sparsity_learner, nullptr);
            m_mat.LoadSparsityPattern(sparsity_learner);
        } else {
            // If an NNZ value for the underlying matrix was specified, perform an initial resizing, *before*
            // a call to ChSystemDescriptor::ConvertToMatrixForm(), to allow for possible size optimizations.
            // Otherwise, do this only at the first call, using the default sparsity fill-in.

            if (m_nnz == 0 && !m_lock || m_setup_call == 0)
                m_mat.Reset(m_dim, m_dim, static_cast<int>(m_dim * (m_dim * SPM_DEF_FULLNESS)));
            else if (m_nnz > 0)
                m_mat.Reset(m_dim, m_dim, m_nnz);
        }

        // Please mind that Reset will be called again on m_mat, inside ConvertToMatrixForm
        sysd.ConvertToMatrixForm(&m_mat, nullptr);

        // Allow the matrix to be compressed.
        bool change = m_mat.Compress();

        // Set current matrix in the MKL engine.
        m_engine.SetMatrix(m_mat);

        // If compression made any change, flag for update of permutation vector.
        if (change && m_use_perm) {
            m_engine.UsePermutationVector(true);
        }

        // The sparsity of rhs must be updated at every cycle (is this true?)
        if (m_use_rhs_sparsity && !m_use_perm)
            m_engine.UsePartialSolution(2);

        m_timer_setup_assembly.stop();

        // Perform the factorization with the Pardiso sparse direct solver.
        m_timer_setup_solvercall.start();
        int pardiso_message_phase12 = m_engine.PardisoCall(ChMklEngine::phase_t::ANALYSIS_NUMFACTORIZATION, 0);
        m_timer_setup_solvercall.stop();

        m_setup_call++;

        if (verbose) {
            GetLog() << " MKL setup n = " << m_dim << "  nnz = " << m_mat.GetNNZ() << "\n";
            GetLog() << "  assembly: " << m_timer_setup_assembly.GetTimeSecondsIntermediate() << "s"
                     << "  solver_call: " << m_timer_setup_solvercall.GetTimeSecondsIntermediate() << "\n";
        }

        if (pardiso_message_phase12 != 0) {
            GetLog() << "Pardiso analyze+reorder+factorize error code = " << pardiso_message_phase12 << "\n";
            return false;
        }

        return true;
    }

    /// Solve using the MKL Pardiso sparse direct solver.
    /// It uses the matrix factorization obtained at the last call to Setup().
    virtual double Solve(ChSystemDescriptor& sysd) override {
        // Assemble the problem right-hand side vector.
        m_timer_solve_assembly.start();
        sysd.ConvertToMatrixForm(nullptr, &m_rhs);
        m_sol.Resize(m_rhs.GetRows(), 1);
        m_engine.SetRhsVector(m_rhs);
        m_engine.SetSolutionVector(m_sol);
        m_timer_solve_assembly.stop();

        // Solve the problem using Pardiso.
        m_timer_solve_solvercall.start();
        int pardiso_message_phase33 = m_engine.PardisoCall(ChMklEngine::phase_t::SOLVE, 0);
        m_timer_solve_solvercall.stop();

        m_solve_call++;

        if (pardiso_message_phase33) {
            GetLog() << "Pardiso solve+refine error code = " << pardiso_message_phase33 << "\n";
            return -1.0;
        }

        if (verbose) {
            double res_norm = m_engine.GetResidualNorm();
            GetLog() << " MKL solve call " << m_solve_call << "  |residual| = " << res_norm << "\n";
            GetLog() << "  assembly: " << m_timer_solve_assembly.GetTimeSecondsIntermediate() << "s\n"
                     << "  solver_call: " << m_timer_solve_solvercall.GetTimeSecondsIntermediate() << "\n";
        }

        // Scatter solution vector to the system descriptor.
        m_timer_solve_assembly.start();
        sysd.FromVectorToUnknowns(m_sol);
        m_timer_solve_assembly.stop();

        return 0.0f;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChSolverMKL>();
        // serialize parent class
        ChSolver::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(m_lock);
        marchive << CHNVP(m_use_perm);
        marchive << CHNVP(m_use_rhs_sparsity);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChSolverMKL>();
        // deserialize parent class
        ChSolver::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(m_lock);
        marchive >> CHNVP(m_use_perm);
        marchive >> CHNVP(m_use_rhs_sparsity);
    }

  private:
    ChMklEngine m_engine = {0, ChSparseMatrix::GENERAL};  ///< interface to MKL solver
    Matrix m_mat = {1, 1};                                ///< problem matrix
    ChMatrixDynamic<double> m_rhs;                        ///< right-hand side vector
    ChMatrixDynamic<double> m_sol;                        ///< solution vector

    int m_dim = 0;         ///< problem size
    int m_nnz = 0;         ///< user-supplied estimate of NNZ
    int m_solve_call = 0;  ///< counter for calls to Solve
    int m_setup_call = 0;  ///< counter for calls to Setup

    bool m_lock = false;                           ///< is the matrix sparsity pattern locked?
    bool m_force_sparsity_pattern_update = false;  ///< is the sparsity pattern changed compared to last call?
    bool m_use_perm = false;                       ///< enable use of the permutation vector?
    bool m_use_rhs_sparsity = false;               ///< leverage right-hand side sparsity?

    ChTimer<> m_timer_setup_assembly;    ///< timer for matrix assembly
    ChTimer<> m_timer_setup_solvercall;  ///< timer for factorization
    ChTimer<> m_timer_solve_assembly;    ///< timer for RHS assembly
    ChTimer<> m_timer_solve_solvercall;  ///< timer for solution
};

/// @} mkl_module

}  // end namespace chrono

#endif
