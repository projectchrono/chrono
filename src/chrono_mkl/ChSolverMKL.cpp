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

#define SPM_DEF_SPARSITY 0.99  ///< default predicted sparsity (in [0,1])

namespace chrono {

ChSolverMKL::ChSolverMKL()
    : m_lock(true),
      m_force_sparsity_pattern_update(false),
      m_use_perm(false),
      m_use_rhs_sparsity(false),
      m_dim(0),
      m_sparsity(-1),
      m_solve_call(0),
      m_setup_call(0) {}

bool ChSolverMKL::Setup(ChSystemDescriptor& sysd) {
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
        // If an sparsity estimate for the underlying matrix was specified, perform an initial resizing, *before*
        // a call to ChSystemDescriptor::ConvertToMatrixForm(), to allow for possible size optimizations.
        // Otherwise, do this only at the first call, using the default sparsity fill-in.
        if ((m_sparsity <= 0 && !m_lock) || m_setup_call == 0) {
            m_mat.resize(m_dim, m_dim);
            m_mat.reserve(Eigen::VectorXi::Constant(m_dim, static_cast<int>(m_dim * (1 - SPM_DEF_SPARSITY))));
        } else if (m_sparsity > 0) {
            m_mat.resize(m_dim, m_dim);
            m_mat.reserve(Eigen::VectorXi::Constant(m_dim, static_cast<int>(m_dim * (1 - m_sparsity))));
        }
    }

    // Please mind that resize will be called again on m_mat, inside ConvertToMatrixForm
    sysd.ConvertToMatrixForm(&m_mat, nullptr);

    // Allow the matrix to be compressed.
    m_mat.makeCompressed();

    m_timer_setup_assembly.stop();

    // Perform the factorization with the Pardiso sparse direct solver.
    m_timer_setup_solvercall.start();
    m_engine.compute(m_mat);
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

double ChSolverMKL::Solve(ChSystemDescriptor& sysd) {
    // Assemble the problem right-hand side vector.
    m_timer_solve_assembly.start();
    sysd.ConvertToMatrixForm(nullptr, &m_rhs);
    m_sol.resize(m_rhs.size());
    m_timer_solve_assembly.stop();

    // Solve the problem using Pardiso.
    m_timer_solve_solvercall.start();

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
        double res_norm = (m_rhs - m_mat * m_sol).norm();
        GetLog() << "residual norm = " << res_norm << "\n";
    }

    if (m_engine.info() != Eigen::Success) {
        GetLog() << "PardisoLU factorize exited with errors\n";
        return -1.0;
    }

    return 0.0f;
}

void ChSolverMKL::ResetTimers() {
    m_timer_setup_assembly.reset();
    m_timer_setup_solvercall.reset();
    m_timer_solve_assembly.reset();
    m_timer_solve_solvercall.reset();
}

void ChSolverMKL::GetErrorMessage(Eigen::ComputationInfo error) const {
    switch (error) {
        case Eigen::Success:
            GetLog() << "computation was successful";
            break;
        case Eigen::NumericalIssue:
            GetLog() << "provided data did not satisfy the prerequisites";
            break;
        case Eigen::NoConvergence:
            GetLog() << "inputs are invalid, or the algorithm has been improperly called";
            break;
    }
}

void ChSolverMKL::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSolverMKL>();
    // serialize parent class
    ChSolver::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(m_lock);
    marchive << CHNVP(m_use_perm);
    marchive << CHNVP(m_use_rhs_sparsity);
}

void ChSolverMKL::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSolverMKL>();
    // deserialize parent class
    ChSolver::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_lock);
    marchive >> CHNVP(m_use_perm);
    marchive >> CHNVP(m_use_rhs_sparsity);
}

}  // end of namespace chrono
