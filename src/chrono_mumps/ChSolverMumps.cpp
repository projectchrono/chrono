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

#include "chrono_mumps/ChSolverMumps.h"
#include "chrono/core/ChSparsityPatternLearner.h"

#define SPM_DEF_SPARSITY 0.9  ///< default predicted sparsity (in [0,1])

namespace chrono {

ChSolverMumps::ChSolverMumps()
    : m_lock(false),
      m_use_learner(true),
      m_force_update(true),
      m_null_pivot_detection(false),
      m_use_rhs_sparsity(false),
      m_symmetry(MatrixSymmetryType::GENERAL),
      m_dim(0),
      m_sparsity(-1),
      m_solve_call(0),
      m_setup_call(0) {}

void ChSolverMumps::EnableNullPivotDetection(bool val, double threshold) {
    m_null_pivot_detection = val;
    m_engine.EnableNullPivotDetection(val, threshold);
}

bool ChSolverMumps::Setup(ChSystemDescriptor& sysd) {
    m_timer_setup_assembly.start();

    // Calculate problem size.
    //// RADU: revisit this; what happens when a problem size does actually change?
    if (m_setup_call == 0) {
        sysd.UpdateCountsAndOffsets();
        m_dim = sysd.CountActiveVariables() + sysd.CountActiveConstraints();
    }

    // If use of the sparsity pattern learner is enabled, call it if:
    // (a) an explicit update was requested (by default this is true at the first call), or
    // (b) the sparsity pattern is not locked and so has to be re-evaluated at each call
    bool call_learner = m_use_learner && (m_force_update || !m_lock);

    // If use of the sparsity pattern learner is disabled, reserve space for nonzeros,
    // using the current sparsity level estimate, if:
    // (a) this is the first call to setup, or
    // (b) the sparsity pattern is not locked and so has to be re-evaluated at each call
    bool call_reserve = !m_use_learner && (m_setup_call == 0 || !m_lock);

    if (verbose) {
        GetLog() << "MUMPS setup\n";
        GetLog() << "  call number:    " << m_setup_call << "\n";
        GetLog() << "  use learner?    " << m_use_learner << "\n";
        GetLog() << "  pattern locked? " << m_lock << "\n";
        GetLog() << "  CALL learner:   " << call_learner << "\n";
        GetLog() << "  CALL reserve:   " << call_reserve << "\n";
    }

    if (call_learner) {
        ChSparsityPatternLearner sparsity_pattern(m_dim, m_dim);
        sysd.ConvertToMatrixForm(&sparsity_pattern, nullptr);
        sparsity_pattern.Apply(m_mat);
        m_force_update = false;
    } else if (call_reserve) {
        double density = (m_sparsity > 0) ? 1 - m_sparsity : 1 - SPM_DEF_SPARSITY;
        m_mat.resize(m_dim, m_dim);
        m_mat.reserve(Eigen::VectorXi::Constant(m_dim, static_cast<int>(m_dim * density)));
    }

    sysd.ConvertToMatrixForm(&m_mat, nullptr);

    m_mat.makeCompressed();

    m_dim = m_mat.rows();

    // Set current matrix in the MUMPS engine.
    m_engine.SetMatrix(m_mat);

    switch (m_symmetry) {
        case MatrixSymmetryType::GENERAL:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::UNSYMMETRIC);
            break;
        case MatrixSymmetryType::SYMMETRIC_POSDEF:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::SYMMETRIC_POSDEF);
            break;
        case MatrixSymmetryType::SYMMETRIC_INDEF:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::SYMMETRIC_GENERAL);
            break;
        default:
            m_engine.SetMatrixSymmetry(ChMumpsEngine::UNSYMMETRIC);
            break;
    }

    m_timer_setup_assembly.stop();

    // Perform the factorization with the MUMPS sparse direct solver.
    m_timer_setup_solvercall.start();
    auto mumps_message = m_engine.MumpsCall(ChMumpsEngine::mumps_JOB::ANALYZE_FACTORIZE);
    m_timer_setup_solvercall.stop();

    m_setup_call++;

    if (verbose) {
        GetLog() << " Mumps setup [" << m_setup_call << "] n = " << m_dim << "  nnz = " << (int)m_mat.nonZeros()
                 << "\n";
        if (m_null_pivot_detection && m_engine.GetINFOG(28) != 0)
            GetLog() << "  Encountered " << m_engine.GetINFOG(28) << " null pivots\n";
        GetLog() << "  assembly:      " << m_timer_setup_assembly.GetTimeSecondsIntermediate() << "s"
                 << "  factorization: " << m_timer_setup_solvercall.GetTimeSecondsIntermediate() << "\n";
    }

    if (mumps_message != 0) {
        m_engine.PrintINFOG();
        return false;
    }

    return true;
}

double ChSolverMumps::Solve(ChSystemDescriptor& sysd) {
    m_timer_solve_assembly.start();
    sysd.ConvertToMatrixForm(nullptr, &m_rhs_sol);
    if (verbose)
        m_rhs_bkp = m_rhs_sol;
    m_timer_solve_assembly.stop();

    m_timer_solve_solvercall.start();
    m_engine.SetRhsVector(m_rhs_sol);
    m_engine.MumpsCall(ChMumpsEngine::mumps_JOB::SOLVE);
    m_timer_solve_solvercall.stop();

    m_solve_call++;

    if (verbose) {
        double res_norm = (m_rhs_bkp - m_mat * m_rhs_sol).norm();
        GetLog() << " Mumps solve [" << m_solve_call << "]  |residual| = " << res_norm << "\n\n";
        GetLog() << "  assembly: " << m_timer_solve_assembly.GetTimeSecondsIntermediate() << "s"
                 << "  solve:    " << m_timer_solve_solvercall.GetTimeSecondsIntermediate() << "\n";
    }

    sysd.FromVectorToUnknowns(m_rhs_sol);

    return 0.0;
}

void ChSolverMumps::ResetTimers() {
    m_timer_setup_assembly.reset();
    m_timer_setup_solvercall.reset();
    m_timer_solve_assembly.reset();
    m_timer_solve_solvercall.reset();
}

}  // namespace chrono
