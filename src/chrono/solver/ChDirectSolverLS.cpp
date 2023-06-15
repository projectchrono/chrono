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
// Authors: Radu Serban
// =============================================================================

#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/core/ChSparsityPatternLearner.h"

#define SPM_DEF_SPARSITY 0.9  ///< default predicted sparsity (in [0,1])

namespace chrono {

ChDirectSolverLS::ChDirectSolverLS()
    : m_lock(false),
      m_use_learner(true),
      m_force_update(true),
      m_null_pivot_detection(false),
      m_use_rhs_sparsity(false),
      m_use_perm(false),
      m_symmetry(MatrixSymmetryType::GENERAL),
      m_dim(0),
      m_sparsity(-1),
      m_solve_call(0),
      m_setup_call(0) {}

void ChDirectSolverLS::ResetTimers() {
    m_timer_setup_assembly.reset();
    m_timer_setup_solvercall.reset();
    m_timer_solve_assembly.reset();
    m_timer_solve_solvercall.reset();
}

bool ChDirectSolverLS::Setup(ChSystemDescriptor& sysd) {
    m_timer_setup_assembly.start();

    // Calculate problem size.
    // Note that ChSystemDescriptor::UpdateCountsAndOffsets was already called at the beginning of the step.
    m_dim = sysd.CountActiveVariables() + sysd.CountActiveConstraints();

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
        GetLog() << "Solver setup\n";
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

    // Let the system descriptor load the current matrix
    sysd.ConvertToMatrixForm(&m_mat, nullptr);

    // Allow the matrix to be compressed
    m_mat.makeCompressed();

    m_timer_setup_assembly.stop();

    if (write_matrix)
        WriteMatrix("LS_" + frame_id + "_A.dat", m_mat);

    // Let the concrete solver perform the facorization
    m_timer_setup_solvercall.start();
    bool result = FactorizeMatrix();
    m_timer_setup_solvercall.stop();

    if (write_matrix)
        WriteMatrix("LS_" + frame_id + "_F.dat", m_mat);

    if (verbose) {
        GetLog() << " Solver setup [" << m_setup_call << "] n = " << m_dim << "  nnz = " << (int)m_mat.nonZeros()
                 << "\n";
        GetLog() << "  assembly matrix:   " << m_timer_setup_assembly.GetTimeSecondsIntermediate() << "s\n"
                 << "  analyze+factorize: " << m_timer_setup_solvercall.GetTimeSecondsIntermediate() << "s\n";
    }

    m_setup_call++;

    if (!result) {
        // If the factorization failed, let the concrete solver display an error message.
        GetLog() << "Solver setup failed\n";
        PrintErrorMessage();
    }

    return result;
}

double ChDirectSolverLS::Solve(ChSystemDescriptor& sysd) {
    // Assemble the problem right-hand side vector
    m_timer_solve_assembly.start();
    sysd.ConvertToMatrixForm(nullptr, &m_rhs);
    m_sol.resize(m_rhs.size());
    m_timer_solve_assembly.stop();

    if (write_matrix)
        WriteVector("LS_" + frame_id + "_b.dat", m_rhs);

    // Let the concrete solver compute the solution
    m_timer_solve_solvercall.start();
    bool result = SolveSystem();
    m_timer_solve_solvercall.stop();

    if (write_matrix)
        WriteVector("LS_" + frame_id + "_x.dat", m_sol);

    // Scatter solution vector to the system descriptor
    m_timer_solve_assembly.start();
    sysd.FromVectorToUnknowns(m_sol);
    m_timer_solve_assembly.stop();

    if (verbose) {
        double res_norm = (m_rhs - m_mat * m_sol).norm();
        GetLog() << " Solver solve [" << m_solve_call << "]  |residual| = " << res_norm << "\n\n";
        GetLog() << "  assembly rhs+sol:  " << m_timer_solve_assembly.GetTimeSecondsIntermediate() << "s\n"
                 << "  solve:             " << m_timer_solve_solvercall.GetTimeSecondsIntermediate() << "\n";
    }

    if (!result) {
        // If the solution failed, let the concrete solver display an error message.
        GetLog() << "Solver solve failed\n";
        PrintErrorMessage();
    }

    return result;
}

bool ChDirectSolverLS::SetupCurrent() {
    m_timer_setup_assembly.start();

    // Allow the matrix to be compressed, if not yet compressed
    m_mat.makeCompressed();

    m_timer_setup_assembly.stop();

    // Let the concrete solver perform the factorization
    m_timer_setup_solvercall.start();
    bool result = FactorizeMatrix();
    m_timer_setup_solvercall.stop();

    if (verbose) {
        GetLog() << " Solver SetupCurrent() [" << m_setup_call << "] n = " << m_dim
                 << "  nnz = " << (int)m_mat.nonZeros() << "\n";
        GetLog() << "  assembly matrix:   " << m_timer_setup_assembly.GetTimeSecondsIntermediate() << "s\n"
                 << "  analyze+factorize: " << m_timer_setup_solvercall.GetTimeSecondsIntermediate() << "s\n";
    }

    m_setup_call++;

    if (!result) {
        // If the factorization failed, let the concrete solver display an error message.
        GetLog() << "Solver SetupCurrent() failed\n";
        PrintErrorMessage();
    }

    return result;
}

double ChDirectSolverLS::SolveCurrent() {
    m_timer_solve_assembly.start();
    m_sol.resize(m_rhs.size());
    m_timer_solve_assembly.stop();

    // Let the concrete solver compute the solution
    m_timer_solve_solvercall.start();
    bool result = SolveSystem();
    m_timer_solve_solvercall.stop();

    if (verbose) {
        double res_norm = (m_rhs - m_mat * m_sol).norm();
        GetLog() << " Solver SolveCurrent() [" << m_solve_call << "]  |residual| = " << res_norm << "\n\n";
        GetLog() << "  assembly rhs+sol:  " << m_timer_solve_assembly.GetTimeSecondsIntermediate() << "s\n"
                 << "  solve:             " << m_timer_solve_solvercall.GetTimeSecondsIntermediate() << "\n";
    }

    if (!result) {
        // If the solution failed, let the concrete solver display an error message.
        GetLog() << "Solver SolveCurrent() failed\n";
        PrintErrorMessage();
    }

    return result;
}

// ---------------------------------------------------------------------------

void ChDirectSolverLS::WriteMatrix(const std::string& filename, const ChSparseMatrix& M) {
    ChStreamOutAsciiFile file(filename.c_str());
    file.SetNumFormat("%.12g");
    for (int i = 0; i < M.rows(); i++) {
        for (int j = 0; j < M.cols(); j++) {
            double elVal = M.coeff(i, j);
            if (elVal || (i == M.rows() - 1 && j == M.cols() - 1)) {
                file << i + 1 << " " << j + 1 << " " << elVal << "\n";
            }
        }
    }
}

void ChDirectSolverLS::WriteVector(const std::string& filename, const ChVectorDynamic<double>& v) {
    ChStreamOutAsciiFile file(filename.c_str());
    file.SetNumFormat("%.12g");
    for (int i = 0; i < v.size(); i++)
        file << v(i) << "\n";
}

// ---------------------------------------------------------------------------

void ChDirectSolverLS::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChDirectSolverLS>();

    // serialize parent class
    ChSolver::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(m_lock);
    marchive << CHNVP(m_use_learner);
    marchive << CHNVP(m_use_perm);
    marchive << CHNVP(m_use_rhs_sparsity);
}

void ChDirectSolverLS::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChDirectSolverLS>();

    // deserialize parent class
    ChSolver::ArchiveIn(marchive);

    // stream in all member data:
    marchive >> CHNVP(m_lock);
    marchive >> CHNVP(m_use_learner);
    marchive >> CHNVP(m_use_perm);
    marchive >> CHNVP(m_use_rhs_sparsity);
}

// ---------------------------------------------------------------------------

bool ChSolverSparseLU::FactorizeMatrix() {
    m_engine.compute(m_mat);
    return (m_engine.info() == Eigen::Success);
}

bool ChSolverSparseLU::SolveSystem() {
    m_sol = m_engine.solve(m_rhs);
    return (m_engine.info() == Eigen::Success);
}

void ChSolverSparseLU::PrintErrorMessage() {
    // There are only three possible return codes (see Eigen SparseLU.h)
    switch (m_engine.info()) {
        case Eigen::Success:
            GetLog() << "computation was successful\n";
            break;
        case Eigen::NumericalIssue:
            GetLog() << "LU factorization reported a problem, zero diagonal for instance\n";
            break;
        case Eigen::InvalidInput:
            GetLog() << "inputs are invalid, or the algorithm has been improperly called\n";
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------

bool ChSolverSparseQR::FactorizeMatrix() {
    m_engine.compute(m_mat);
    return (m_engine.info() == Eigen::Success);
}

bool ChSolverSparseQR::SolveSystem() {
    m_sol = m_engine.solve(m_rhs);
    return (m_engine.info() == Eigen::Success);
}

void ChSolverSparseQR::PrintErrorMessage() {
    // There are only three possible return codes (see Eigen SparseLU.h)
    switch (m_engine.info()) {
        case Eigen::Success:
            GetLog() << "computation was successful\n";
            break;
        case Eigen::NumericalIssue:
            GetLog() << "QR factorization reported a problem\n";
            break;
        case Eigen::InvalidInput:
            GetLog() << "inputs are invalid, or the algorithm has been improperly called\n";
            break;
        default:
            break;
    }
}

}  // end namespace chrono
