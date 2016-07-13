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

#include "chrono_mkl/ChSolverMKL.h"

namespace chrono {
// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChSolverMKL> a_registration_ChSolverMKL;

/** \brief It calls Intel MKL Pardiso Sparse Direct Solver.
*
*	On the first call the routine resets the matrix and vectors involved to the size of the current problem.
*	Every time the system stored in \c sysd is converted in a CSR3 standard matrix that will be later used by Pardiso.
*	If the sparsity pattern lock is turned on then the matrix will try, as long as possible, to preserve not only the
*   arrays dimensions, but it also keeps column and row indexes through different calls.
*/

ChSolverMKL::ChSolverMKL()
    : mkl_engine(0, ChSparseMatrix::GENERAL),
      matCSR3(1, 1, 1),
      solver_call(0),
      n(0),
      sparsity_pattern_lock(false),
      use_perm(false),
      use_rhs_sparsity(false),
      nnz(0) {}

double ChSolverMKL::Solve(ChSystemDescriptor& sysd) {
    timer_solve_assembly.start();
    sysd.ConvertToMatrixForm(nullptr, &rhs);
    sol.Resize(rhs.GetRows(), 1);
    timer_solve_assembly.stop();

    timer_solve_pardiso.start();
    mkl_engine.SetProblem(matCSR3, rhs, sol);
    int pardiso_message_phase33 = mkl_engine.PardisoCall(33, 0);
    timer_solve_pardiso.stop();

    solver_call++;
    if (pardiso_message_phase33) {
        GetLog() << "Pardiso solve+refine error code = " << pardiso_message_phase33 << "\n";
        GetLog() << "Matrix verification code = " << matCSR3.VerifyMatrix() << "\n";
        GetLog() << "Matrix MKL verification code = " << matCSR3.VerifyMatrixByMKL() << "\n";
    }

    if (verbose) {
        ChMatrixDynamic<> res(rhs.GetRows(), 1);
        mkl_engine.GetResidual(res);
        double res_norm = mkl_engine.GetResidualNorm(res);

        GetLog() << " MKL call " << solver_call << "  |residual| = " << res_norm << "\n";
    }

    // Replicate the changes to vvariables and vconstraint into SystemDescriptor
    timer_solve_assembly.start();
    sysd.FromVectorToUnknowns(sol);
    timer_solve_assembly.stop();

    return 0.0f;
}

bool ChSolverMKL::Setup(ChSystemDescriptor& sysd) {
    timer_setup_assembly.start();

    // Initial resizing;
    if (solver_call == 0) {
        // not mandatory, but it speeds up the first build of the matrix, guessing its sparsity; needs to stay BEFORE
        // ConvertToMatrixForm()
        n = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
        nnz ? matCSR3.Reset(n, n, nnz) : matCSR3.Reset(n, n, static_cast<int>(n * (n * SPM_DEF_FULLNESS)));
    } else if (nnz) {
        matCSR3.Reset(n, n, nnz);
    }

    // Build matrix and rhs;
    // in case the matrix changes size (rows) then RowIndexLockBroken is turned on
    // in case the matrix has to insert a new element in the matrix then ColIndexLockBroken is turned on
    // in case the matrix inserts LESS elements than the previous cycle:
    //     - if sparsity_pattern_lock is ON the matrix will remain with unuseful filling zeros; they have to be
    //     eventually removed with a Purge();
    //     - if sparsity_pattern_lock is ON the matrix will have some uninitialized element flagged with -1 in the
    //     colIndex;
    //          those have to be removed by Compress().
    sysd.ConvertToMatrixForm(&matCSR3, nullptr);
    n = matCSR3.GetNumRows();

    // Set up the lock (if enabled).
    // This must be done only after a first call to ChSystemDescriptor::ConvertToMatrixForm();
    matCSR3.SetSparsityPatternLock(sparsity_pattern_lock);

    // If the sparsity pattern is unlocked or the lock is broken, compress the matrix.
    if (!sparsity_pattern_lock || matCSR3.IsSparsityPatternLockBroken()) {
        matCSR3.Compress();

        // If the sparsity pattern has changed, flag update of permutation vector.
        if (use_perm)
            mkl_engine.UsePermutationVector(true);
    }

    // the sparsity of rhs must be updated at every cycle (am I wrong?)
    if (use_rhs_sparsity && !use_perm)
        mkl_engine.UsePartialSolution(2);

    timer_setup_assembly.stop();

    // Solve with Pardiso Sparse Direct Solver
    // the problem size must be updated also in the Engine: this is done by SetProblem() itself.
    timer_setup_pardiso.start();
    mkl_engine.SetProblem(matCSR3, rhs, sol);
    int pardiso_message_phase12 = mkl_engine.PardisoCall(12, 0);
    timer_setup_pardiso.stop();

    if (verbose) {
        GetLog() << " MKL setup n = " << n << "  nnz = " << matCSR3.GetColIndexLength() << "\n";
    }

    if (pardiso_message_phase12 != 0) {
        // Factorization failed.
        GetLog() << "Pardiso analyze+reorder+factorize error code = " << pardiso_message_phase12 << "\n";
        GetLog() << "Matrix verification code = " << matCSR3.VerifyMatrix() << "\n";
        GetLog() << "Matrix MKL verification code = " << matCSR3.VerifyMatrixByMKL() << "\n";
        return false;
    }

    return true;
}

}  // end namespace chrono
