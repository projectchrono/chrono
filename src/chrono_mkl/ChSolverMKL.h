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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#ifndef CHSOLVERMKL_H
#define CHSOLVERMKL_H

#include "chrono/solver/ChSolver.h"
#include "chrono/solver/ChSystemDescriptor.h"
#include "chrono/core/ChSparseMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"
#include "chrono/core/ChTimer.h"

#include "chrono_mkl/ChMklEngine.h"
#include "chrono_mkl/ChCSR3Matrix.h"

namespace chrono {

/// @addtogroup mkl_module
/// @{

/// Class that wraps the Intel MKL Pardiso parallel direct solver.
/// It can solve linear systems, but not VI and complementarity problems.
template <typename Matrix = ChCSR3Matrix>
class ChSolverMKL : public ChSolver {
  public:
    ChSolverMKL()
        : m_engine(0, ChSparseMatrix::GENERAL),
          m_mat(1, 1),
          m_solver_call(0),
          m_dim(0),
          m_nnz(0),
          m_lock(false),
          m_use_perm(false),
          m_use_rhs_sparsity(false) {}

    ~ChSolverMKL() {}

    /// Get a handle to thenunderlying MKL engine.
    ChMklEngine& GetMklEngine() { return m_engine; }

    Matrix& GetMatrix() { return m_mat; }

    /// Enable/disable locking the sparsity pattern (default: false).
    /// If on_off is set to true, then the sparsity pattern of the problem matrix is assumed
    /// to be unchanged from call to call.
    void SetSparsityPatternLock(bool val) { m_lock = val; }

    /// Enable/disable use of permutation vector (default: false).
    void UsePermutationVector(bool val) { m_use_perm = val; }

    /// Enable/disable leveraging sparsity in right-hand side vector (default: false).
    void LeverageRhsSparsity(bool val) { m_use_rhs_sparsity = val; }

    void SetPreconditionedCGS(bool val, int L) { m_engine.SetPreconditionedCGS(val, L); }

    /// Set the number of non-zero entries in the problem matrix.
    void SetMatrixNNZ(int nnz) { m_nnz = nnz; }

    /// Reset timers for internal phases in Solve and Setup.
    void ResetTimers() {
        m_timer_setup_assembly.reset();
        m_timer_setup_pardiso.reset();
        m_timer_solve_assembly.reset();
        m_timer_solve_pardiso.reset();
    }

    /// Get cumulative time for assembly operations in Solve phase.
    double GetTimeSolveAssembly() { return m_timer_solve_assembly(); }
    /// Get cumulative time for Pardiso calls in Solve phase.
    double GetTimeSolvePardiso() { return m_timer_solve_pardiso(); }
    /// Get cumulative time for assembly operations in Setup phase.
    double GetTimeSetupAssembly() { return m_timer_setup_assembly(); }
    /// Get cumulative time for Pardiso calls in Setup phase.
    double GetTimeSetupPardiso() { return m_timer_setup_pardiso(); }

    /// Indicate whether or not the Solve() phase requires an up-to-date problem matrix.
    /// As typical of direct solvers, the Pardiso solver only requires the matrix for its Setup() phase.
    virtual bool SolveRequiresMatrix() const override { return false; }

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
        m_timer_solve_pardiso.start();
        int pardiso_message_phase33 = m_engine.PardisoCall(33, 0);
        m_timer_solve_pardiso.stop();

        m_solver_call++;

        if (pardiso_message_phase33) {
            GetLog() << "Pardiso solve+refine error code = " << pardiso_message_phase33 << "\n";
            return -1.0;
        }

        if (verbose) {
            double res_norm = m_engine.GetResidualNorm();
            GetLog() << " MKL solve call " << m_solver_call << "  |residual| = " << res_norm << "\n";
        }

        // Scatter solution vector to the system descriptor.
        m_timer_solve_assembly.start();
        sysd.FromVectorToUnknowns(m_sol);
        m_timer_solve_assembly.stop();

        return 0.0f;
    }

    /// Perform the solver setup operations.
    /// For the MKL solver, this means assembling and factorizing the system matrix.
    /// Returns true if successful and false otherwise.
    virtual bool Setup(ChSystemDescriptor& sysd) override {
        m_timer_setup_assembly.start();

        // Set the lock on the matrix sparsity pattern (if enabled).
        m_mat.SetSparsityPatternLock(m_lock);

        // Calculate problem size at first call.
        if (m_solver_call == 0) {
            m_dim = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
        }

        // If an NNZ value for the underlying matrix was specified, perform an initial resizing, *before*
        // a call to ChSystemDescriptor::ConvertToMatrixForm(), to allow for possible size optimizations.
        // Otherwise, do this only at the first call, using the default sparsity fill-in.
        if (m_nnz != 0) {
            m_mat.Reset(m_dim, m_dim, m_nnz);
        } else if (m_solver_call == 0) {
            m_mat.Reset(m_dim, m_dim, static_cast<int>(m_dim * (m_dim * SPM_DEF_FULLNESS)));
        }

        // Assemble the matrix.
        sysd.ConvertToMatrixForm(&m_mat, nullptr);
        m_dim = m_mat.GetNumRows();


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

        if (verbose) {
            GetLog() << " MKL setup n = " << m_dim << "  nnz = " << m_mat.GetNNZ() << "\n";
        }

        // Perform the factorization with the Pardiso sparse direct solver.
        m_timer_setup_pardiso.start();
        int pardiso_message_phase12 = m_engine.PardisoCall(12, 0);
        m_timer_setup_pardiso.stop();

        if (pardiso_message_phase12 != 0) {
            GetLog() << "Pardiso analyze+reorder+factorize error code = " << pardiso_message_phase12 << "\n";
            return false;
        }

        return true;
    }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite(1);
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
        int version = marchive.VersionRead();
        // deserialize parent class
        ChSolver::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(m_lock);
        marchive >> CHNVP(m_use_perm);
        marchive >> CHNVP(m_use_rhs_sparsity);
    }

  private:
    ChMklEngine m_engine;           ///< interface to MKL solver
    Matrix m_mat;                   ///< problem matrix
    ChMatrixDynamic<double> m_rhs;  ///< right-hand side vector
    ChMatrixDynamic<double> m_sol;  ///< solution vector
    int m_dim;                      ///< problem size
    int m_nnz;                      ///< user-supplied estimate of NNZ
    int m_solver_call;              ///< counter for calls to Solve

    bool m_lock;              ///< is the matrix sparsity pattern locked?
    bool m_use_perm;          ///< enable use of the permutation vector?
    bool m_use_rhs_sparsity;  ///< leverage right-hand side sparsity?

    ChTimer<> m_timer_setup_assembly;  ///< timer for matrix assembly
    ChTimer<> m_timer_setup_pardiso;   ///< timer for factorization
    ChTimer<> m_timer_solve_assembly;  ///< timer for RHS assembly
    ChTimer<> m_timer_solve_pardiso;   ///< timer for solution
};

/// @} mkl_module

}  // end namespace chrono

#endif
