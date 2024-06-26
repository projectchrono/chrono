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

#include <numeric>
#include <iomanip>

#include "chrono_modal/ChModalSolverDamped.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChDirectSolverLScomplex.h"
#include "chrono/utils/ChConstants.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChSparsityPatternLearner.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>

using namespace Eigen;

namespace chrono {

namespace modal {

/// Partially build the damped eigenvalue problem matrices A and B from a given ChAssembly.
/// WARNING: Cq and Cq' signs are not flipped here: the user is expected to flip it during scaling (if any)
/// The final shape of the matrices is:
///
/// A  =  [  0     I     0 ]
///       [ -K    -R  -Cq' ]
///       [ -Cq    0     0 ]
///
/// B  =  [  I     0     0 ]
///       [  0     M     0 ]
///       [  0     0     0 ]
///
void BuildDampedEigenProblemMatrices(ChAssembly& assembly,
                                     ChSystemDescriptor& temp_descriptor,
                                     ChSparseMatrix& A,
                                     ChSparseMatrix& B,
                                     int n_vars) {
    // Stiffness matrix
    assembly.LoadKRMMatrices(-1.0, 0.0, 0.0);
    temp_descriptor.SetMassFactor(0.0);
    temp_descriptor.PasteMassKRMMatrixInto(A, n_vars, 0);

    // Damping matrix
    assembly.LoadKRMMatrices(0.0, -1.0, 0.0);
    temp_descriptor.SetMassFactor(0.0);
    temp_descriptor.PasteMassKRMMatrixInto(A, n_vars, n_vars);

    // Mass matrix
    assembly.LoadKRMMatrices(0.0, 0.0, 1.0);
    temp_descriptor.SetMassFactor(1.0);
    temp_descriptor.PasteMassKRMMatrixInto(B, n_vars, n_vars);

    // Constraint Jacobian
    assembly.LoadConstraintJacobians();
    temp_descriptor.PasteConstraintsJacobianMatrixInto(A, 2 * n_vars, 0);
    temp_descriptor.PasteConstraintsJacobianMatrixTransposedInto(A, n_vars, 2 * n_vars);

    // Identity matrix
    for (auto id_sel = 0; id_sel < n_vars; ++id_sel) {
        A.SetElement(id_sel, id_sel + n_vars, 1.0);
        B.SetElement(id_sel, id_sel, 1.0);
    }
}

int ChModalSolverDamped::Solve(const ChAssembly& assembly,
                               ChMatrixDynamic<std::complex<double>>& eigvects,
                               ChVectorDynamic<std::complex<double>>& eigvals,
                               ChVectorDynamic<double>& freq,
                               ChVectorDynamic<double>& damp_ratios) const {
    ChAssembly* assembly_nonconst = const_cast<ChAssembly*>(&assembly);

    std::shared_ptr<ChSystemDescriptor> descriptor_bkp;
    if (assembly_nonconst->GetSystem()->GetSystemDescriptor())
        descriptor_bkp = assembly_nonconst->GetSystem()->GetSystemDescriptor();

    // Extract matrices from the system descriptor
    ChSystemDescriptor temp_descriptor;

    temp_descriptor.BeginInsertion();
    assembly_nonconst->InjectVariables(temp_descriptor);
    assembly_nonconst->InjectKRMMatrices(temp_descriptor);
    assembly_nonconst->InjectConstraints(temp_descriptor);
    temp_descriptor.EndInsertion();

    // Generate the A and B in state space
    int n_vars = temp_descriptor.CountActiveVariables();
    int n_constr = temp_descriptor.CountActiveConstraints();

    // Leverage the sparsity pattern learner to allocate the proper size
    ChSparsityPatternLearner A_spl(2 * n_vars + n_constr, 2 * n_vars + n_constr);
    ChSparsityPatternLearner B_spl(2 * n_vars + n_constr, 2 * n_vars + n_constr);
    BuildDampedEigenProblemMatrices(*assembly_nonconst, temp_descriptor, A_spl, B_spl, n_vars);

    // Build the actual matrices
    ChSparseMatrix A(2 * n_vars + n_constr, 2 * n_vars + n_constr);
    ChSparseMatrix B(2 * n_vars + n_constr, 2 * n_vars + n_constr);
    A_spl.Apply(A);
    B_spl.Apply(B);
    BuildDampedEigenProblemMatrices(*assembly_nonconst, temp_descriptor, A, B, n_vars);

    // Scale constraints matrix
    double scaling = 1.0;
    if (m_scaleCq) {
        scaling = 0.0;
        for (int k = 0; k < A.outerSize(); ++k) {
            for (ChSparseMatrix::InnerIterator it(A, k); it; ++it) {
                if (it.row() >= n_vars && it.row() < (2 * n_vars) && it.col() == it.row() - n_vars) {
                    scaling += it.valueRef();
                }
            }
        }
        scaling = -scaling / n_vars;
    }

    // Cq scaling
    for (auto row_i = 2 * n_vars; row_i < 2 * n_vars + n_constr; row_i++) {
        for (auto nnz_i = A.outerIndexPtr()[row_i];
             nnz_i <
             (A.isCompressed() ? A.outerIndexPtr()[row_i + 1] : A.outerIndexPtr()[row_i] + A.innerNonZeroPtr()[row_i]);
             ++nnz_i) {
            A.valuePtr()[nnz_i] *= -scaling;
        }
    }

    // CqT scaling
    for (auto k = 0; k < n_vars; ++k) {
        for (ChSparseMatrix::InnerIterator it(A, n_vars + k); it; ++it) {
            if (it.col() >= 2 * n_vars) {
                it.valueRef() *= -scaling;
            }
        }
    }

    A.makeCompressed();
    B.makeCompressed();

    std::list<std::pair<int, std::complex<double>>> eig_requests;
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
    m_solver->GetDampingRatios(eigvals, damp_ratios);
    m_timer_solution_postprocessing.stop();

    if (descriptor_bkp) {
        assembly_nonconst->GetSystem()->SetSystemDescriptor(descriptor_bkp);
        assembly_nonconst->Setup();
    }

    return found_eigs;
}

int ChModalSolverDamped::Solve(const ChSparseMatrix& K,
                               const ChSparseMatrix& R,
                               const ChSparseMatrix& M,
                               const ChSparseMatrix& Cq,
                               ChMatrixDynamic<std::complex<double>>& eigvects,
                               ChVectorDynamic<std::complex<double>>& eigvals,
                               ChVectorDynamic<double>& freq,
                               ChVectorDynamic<double>& damp_ratios) const {
    m_timer_matrix_assembly.start();

    // Generate the A and B in state space
    int n_vars = K.rows();
    int n_constr = Cq.rows();

    ChSparseMatrix A(n_vars + n_constr, n_vars + n_constr);
    ChSparseMatrix B(n_vars + n_constr, n_vars + n_constr);
    double scaling = m_solver->BuildDampedSystem(M, R, K, Cq, A, B, m_scaleCq);

    std::list<std::pair<int, std::complex<double>>> eig_requests;
    for (int i = 0; i < m_freq_spans.size(); i++) {
        eig_requests.push_back(std::make_pair(m_freq_spans[i].nmodes, m_solver->GetOptimalShift(m_freq_spans[i].freq)));
    }

    m_timer_matrix_assembly.stop();

    m_timer_eigen_solver.start();
    int found_eigs = modal::Solve<>(*m_solver, A, B, eigvects, eigvals, eig_requests, true, m_clip_position_coords ? n_vars : 0);
    m_timer_eigen_solver.stop();

    // the scaling does not affect the eigenvalues
    // but affects the constraint part of the eigenvectors
    if (!m_clip_position_coords) {
        eigvects.bottomRows(n_constr) *= scaling;
    }

    m_timer_solution_postprocessing.start();
    m_solver->GetNaturalFrequencies(eigvals, freq);
    m_solver->GetDampingRatios(eigvals, damp_ratios);
    m_timer_solution_postprocessing.stop();

    return found_eigs;
}

}  // end namespace modal

}  // end namespace chrono
