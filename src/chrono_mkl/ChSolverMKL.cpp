#include "ChSolverMKL.h"
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

/// Perform the solver setup operations.
/// For the MKL solver, this means assembling and factorizing the system matrix.
/// Returns true if successful and false otherwise.

bool chrono::ChSolverMKL::Setup(ChSystemDescriptor& sysd) {
    m_timer_setup_assembly.start();

    // Calculate problem size at first call.
    if (m_setup_call == 0) {
		sysd.UpdateCountsAndOffsets();
        m_dim = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
    }

    // Let the matrix acquire the information about ChSystem
    if (m_force_sparsity_pattern_update || (m_lock && m_setup_call == 0)) {
        m_force_sparsity_pattern_update = false;

        ChSparsityPatternLearner sparsity_pattern(m_dim, m_dim);
        sysd.ConvertToMatrixForm(&sparsity_pattern, nullptr);
        sparsity_pattern.Apply(m_mat);
    } else {
        // If an NNZ value for the underlying matrix was specified, perform an initial resizing, *before*
        // a call to ChSystemDescriptor::ConvertToMatrixForm(), to allow for possible size optimizations.
        // Otherwise, do this only at the first call, using the default sparsity fill-in.

        if (m_nnz == 0 && !m_lock || m_setup_call == 0) {
            m_mat.resize(m_dim, m_dim);
            m_mat.reserve(static_cast<int>(m_dim * (m_dim * SPM_DEF_FULLNESS)));
        } else if (m_nnz > 0) {
            m_mat.resize(m_dim, m_dim);
            m_mat.reserve(m_nnz);
        }
    }

    // Please mind that Reset will be called again on m_mat, inside ConvertToMatrixForm
    sysd.ConvertToMatrixForm(&m_mat, nullptr);


    // Allow the matrix to be compressed.
    m_mat.makeCompressed();

	{
        ChStreamOutAsciiFile mySparseFile("C:/mySparseMat.dat");
        StreamOUTsparseMatlabFormat(m_mat, mySparseFile);
    }
  

    m_timer_setup_assembly.stop();

    // Perform the factorization with the Pardiso sparse direct solver.
    m_timer_setup_solvercall.start();
    m_engine.analyzePattern(m_mat);
    m_timer_setup_solvercall.stop();

    m_setup_call++;

    if (verbose) {
        GetLog() << " MKL setup n = " << m_dim << "  nnz = " << (int)m_mat.nonZeros() << "\n";
        GetLog() << "  assembly (matrix): " << m_timer_setup_assembly.GetTimeSecondsIntermediate() << "s\n"
                 << "  analyzePattern: " << m_timer_setup_solvercall.GetTimeSecondsIntermediate() << "s\n";
    }


    if (m_engine.info() != Eigen::Success) {
        GetLog() << "PardisoLU compute command exited with errors\n";
        return false;
    }

    return true;
}

double chrono::ChSolverMKL::Solve(ChSystemDescriptor& sysd) {
    // Assemble the problem right-hand side vector.
    m_timer_solve_assembly.start();
    sysd.ConvertToMatrixForm(nullptr, &m_rhs);
    m_sol.resize(m_rhs.size());
    m_timer_solve_assembly.stop();

    // Solve the problem using Pardiso.
    m_timer_solve_solvercall.start();
    m_engine.factorize(m_mat);


    if (m_engine.info() != Eigen::Success) {
        GetLog() << "PardisoLU factorize exited with errors\n";
        return -1.0;
    }

    m_sol = m_engine.solve(m_rhs);
	m_solve_call++;
    m_timer_solve_solvercall.stop();

    // Scatter solution vector to the system descriptor.
    m_timer_solve_assembly.start();
    sysd.FromVectorToUnknowns(m_sol);
    m_timer_solve_assembly.stop();

	
    if (verbose) {
        GetLog() << "  assembly rhs+sol: " << m_timer_solve_assembly.GetTimeSecondsIntermediate() << "s\n"
                 << "  factorize+solve: " << m_timer_solve_solvercall.GetTimeSecondsIntermediate() << "\n";
		double res_norm = (m_rhs - m_mat*m_sol).norm();
		GetLog() << "residual norm = " << res_norm << "\n";
    }

    return 0.0f;
}
