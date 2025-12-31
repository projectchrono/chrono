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
// Authors: Alessandro Tasora, Dario Mangoni
// =============================================================================

#ifndef CHMODALSOLVERUNDAMPED_H
#define CHMODALSOLVERUNDAMPED_H

#include "chrono_modal/ChApiModal.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChAssembly.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChSparsityPatternLearner.h"
#include "chrono_modal/ChGeneralizedEigenvalueSolver.h"
#include "chrono_modal/ChModalSolver.h"

namespace chrono {
namespace modal {

/// @addtogroup modal
/// @{

/// Utility function to build the A and B matrices for the undamped eigenvalue problem.
void ChApiModal BuildUndampedEigenProblemMatrices(ChAssembly& assembly,
                                                  ChSystemDescriptor& temp_descriptor,
                                                  ChSparseMatrix& A,
                                                  ChSparseMatrix& B,
                                                  int n_vars);

/// Modal solver for undamped systems (-w^2*M+K)*x = 0 s.t. Cq*x = 0
/// Contrary to eigensolvers, these classes deals directly with Chrono data structures, creating the appropriate setup
/// for running the inner eigensolver.
/// Multiple requests spanning different frequency ranges can be specified.
template <typename EigenvalueSolverType>
class ChModalSolverUndamped : public ChModalSolver {
  public:
    using ScalarType = typename EigenvalueSolverType::ScalarType;

    /// Creates a modal solver for the undamped case.
    /// \a n_lower_modes number of lowest modes to be found.
    /// \a base_freq frequency around which the modes will be found; higher values can help finding eigenvalues for
    /// ill-conditioned problems.
    /// \a scaleCq if true, the Cq matrix is scaled to improve conditioning.
    /// \a verbose if true, additional information is printed during the solution process.
    /// \a solver the inner eigensolver to be used.
    ChModalSolverUndamped(
        int n_lower_modes,
        double base_freq = 1e-5,
        bool scaleCq = true,
        bool verbose = false,
        std::shared_ptr<EigenvalueSolverType> solver = chrono_types::make_shared<EigenvalueSolverType>())
        : ChModalSolver(n_lower_modes, base_freq, scaleCq, verbose), m_solver(solver){};

    /// Creates a modal solver for the undamped case.
    /// \a freq_spans pair of number of modes and frequency around which the modes will be found.
    /// ill-conditioned problems.
    /// \a scaleCq if true, the Cq matrix is scaled to improve conditioning.
    /// \a verbose if true, additional information is printed during the solution process.
    /// \a solver the inner eigensolver to be used.
    ChModalSolverUndamped(
        std::vector<ChFreqSpan> freq_spans,
        bool scaleCq = true,
        bool verbose = false,
        std::shared_ptr<EigenvalueSolverType> solver = chrono_types::make_shared<EigenvalueSolverType>())
        : ChModalSolver(freq_spans, scaleCq, verbose), m_solver(solver){};

    virtual ~ChModalSolverUndamped(){};

    /// Solve the constrained eigenvalue problem retrieving it from the ChAssembly.
    /// Only the position part of the eigenvectors is returned, unless SetClipPositionCoords(false) is called.
    int Solve(const ChAssembly& assembly,
              ChMatrixDynamic<ScalarType>& eigvects,
              ChVectorDynamic<ScalarType>& eigvals,
              ChVectorDynamic<double>& freq) const;

    /// Solve the constrained eigenvalue problem setting it up from individual matrices.
    /// Only the position part of the eigenvectors is returned, unless SetClipPositionCoords(false) is called.
    int Solve(const ChSparseMatrix& K,
              const ChSparseMatrix& M,
              const ChSparseMatrix& Cq,
              ChMatrixDynamic<ScalarType>& eigvects,
              ChVectorDynamic<ScalarType>& eigvals,
              ChVectorDynamic<double>& freq) const;

    /// Get the inner eigensolver.
    std::shared_ptr<EigenvalueSolverType> GetEigenSolver() const { return m_solver; }

  protected:
    std::shared_ptr<EigenvalueSolverType> m_solver;
};

template <typename EigenvalueSolverType>
int ChModalSolverUndamped<EigenvalueSolverType>::Solve(const ChAssembly& assembly,
                                                       ChMatrixDynamic<ScalarType>& eigvects,
                                                       ChVectorDynamic<ScalarType>& eigvals,
                                                       ChVectorDynamic<double>& freq) const {
    ChAssembly* assembly_nonconst = const_cast<ChAssembly*>(&assembly);

    std::shared_ptr<ChSystemDescriptor> descriptor_bkp;
    if (assembly_nonconst->GetSystem()->GetSystemDescriptor())
        descriptor_bkp = assembly_nonconst->GetSystem()->GetSystemDescriptor();

    m_timer_matrix_assembly.start();

    ChSystemDescriptor temp_descriptor;

    temp_descriptor.BeginInsertion();
    assembly_nonconst->InjectVariables(temp_descriptor);
    assembly_nonconst->InjectKRMMatrices(temp_descriptor);
    assembly_nonconst->InjectConstraints(temp_descriptor);
    temp_descriptor.EndInsertion();

    // Generate the A and B in state space
    int n_vars = temp_descriptor.CountActiveVariables();
    int n_constr = temp_descriptor.CountActiveConstraints();

    // A  =  [ -K   -Cq' ]
    //       [ -Cq    0  ]

    // B  =  [  M     0  ]
    //       [  0     0  ]

    ChSparsityPatternLearner A_spl(n_vars + n_constr, n_vars + n_constr);
    ChSparsityPatternLearner B_spl(n_vars + n_constr, n_vars + n_constr);
    BuildUndampedEigenProblemMatrices(*assembly_nonconst, temp_descriptor, A_spl, B_spl, n_vars);

    ChSparseMatrix A(n_vars + n_constr, n_vars + n_constr);
    ChSparseMatrix B(n_vars + n_constr, n_vars + n_constr);
    A_spl.Apply(A);
    B_spl.Apply(B);

    A.setZeroValues();
    B.setZeroValues();
    BuildUndampedEigenProblemMatrices(*assembly_nonconst, temp_descriptor, A, B, n_vars);

    m_timer_matrix_assembly.stop();

    m_timer_matrix_assembly.start();

    // Find scaling factor for Cq
    // Be aware that A contains -K, not K
    double scaling = 1.0;
    if (m_scaleCq) {
        scaling = 0.0;
        for (int k = 0; k < A.outerSize(); ++k) {
            for (ChSparseMatrix::InnerIterator it(A, k); it; ++it) {
                if (it.row() < n_vars && it.col() == it.row()) {
                    scaling += it.valueRef();
                }
            }
        }
        scaling = -scaling / n_vars;
    }

    // Cq scaling and sign change
    for (auto row_i = n_vars; row_i < n_vars + n_constr; row_i++) {
        for (auto nnz_i = A.outerIndexPtr()[row_i];
             nnz_i <
             (A.isCompressed() ? A.outerIndexPtr()[row_i + 1] : A.outerIndexPtr()[row_i] + A.innerNonZeroPtr()[row_i]);
             ++nnz_i) {
            A.valuePtr()[nnz_i] *= -scaling;
        }
    }

    // CqT scaling and sign change
    for (auto k = 0; k < n_vars; ++k) {
        for (ChSparseMatrix::InnerIterator it(A, k); it; ++it) {
            if (it.col() >= n_vars) {
                it.valueRef() *= -scaling;
            }
        }
    }

    A.makeCompressed();
    B.makeCompressed();

    std::list<std::pair<int, ScalarType>> eig_requests;
    for (int i = 0; i < m_freq_spans.size(); i++) {
        eig_requests.push_back(std::make_pair(m_freq_spans[i].nmodes, m_solver->GetOptimalShift(m_freq_spans[i].freq)));
    }

    m_timer_matrix_assembly.stop();

    m_timer_eigen_solver.start();
    int found_eigs = modal::Solve<>(*m_solver, A, B, eigvects, eigvals, eig_requests, true, m_clip_position_coords ? n_vars : 0);

    // the scaling does not affect the eigenvalues
    // but affects the constraint part of the eigenvectors
    if (!m_clip_position_coords) {
        eigvects.bottomRows(n_constr) *= scaling;
    }

    m_timer_eigen_solver.stop();

    m_timer_solution_postprocessing.start();
    m_solver->GetNaturalFrequencies(eigvals, freq);
    m_timer_solution_postprocessing.stop();

    if (descriptor_bkp) {
        assembly_nonconst->GetSystem()->SetSystemDescriptor(descriptor_bkp);
        assembly_nonconst->Setup();
    }

    return found_eigs;
}

template <typename EigenvalueSolverType>
int ChModalSolverUndamped<EigenvalueSolverType>::Solve(const ChSparseMatrix& K,
                                                       const ChSparseMatrix& M,
                                                       const ChSparseMatrix& Cq,
                                                       ChMatrixDynamic<ScalarType>& eigvects,
                                                       ChVectorDynamic<ScalarType>& eigvals,
                                                       ChVectorDynamic<double>& freq) const {
    m_timer_matrix_assembly.start();

    // Generate the A and B in state space
    int n_vars = K.rows();
    int n_constr = Cq.rows();

    ChSparseMatrix A(n_vars + n_constr, n_vars + n_constr);
    ChSparseMatrix B(n_vars + n_constr, n_vars + n_constr);
    double scaling = m_solver->BuildUndampedSystem(M, K, Cq, A, B, m_scaleCq);

    std::list<std::pair<int, ScalarType>> eig_requests;
    for (int i = 0; i < m_freq_spans.size(); i++) {
        eig_requests.push_back(std::make_pair(m_freq_spans[i].nmodes, m_solver->GetOptimalShift(m_freq_spans[i].freq)));
    }

    m_timer_matrix_assembly.stop();

    m_timer_eigen_solver.start();
    int found_eigs = modal::Solve<>(*m_solver, A, B, eigvects, eigvals, eig_requests, true, m_clip_position_coords ? n_vars : 0);

    // the scaling does not affect the eigenvalues
    // but affects the constraint part of the eigenvectors
    if (!m_clip_position_coords) {
        eigvects.bottomRows(n_constr) *= scaling;
    }
    m_timer_eigen_solver.stop();

    m_timer_solution_postprocessing.start();
    m_solver->GetNaturalFrequencies(eigvals, freq);
    m_timer_solution_postprocessing.stop();

    return found_eigs;
}

/// @} modal

}  // end namespace modal
}  // end namespace chrono

#endif
