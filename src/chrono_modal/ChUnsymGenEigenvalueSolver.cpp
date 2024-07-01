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

#include <iomanip>

#include "chrono_modal/ChUnsymGenEigenvalueSolver.h"
#include "chrono_modal/ChKrylovSchurEig.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/utils/ChConstants.h"
#include "chrono/core/ChMatrix.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>

using namespace Eigen;

namespace chrono {

namespace modal {

void ChUnsymGenEigenvalueSolver::GetNaturalFrequencies(const ChVectorDynamic<ScalarType>& eigvals,
                                                       ChVectorDynamic<double>& natural_freq) {
    natural_freq.resize(eigvals.size());

    for (int i = 0; i < eigvals.size(); ++i) {
        natural_freq(i) = GetNaturalFrequency(eigvals(i));
    }
}

void ChUnsymGenEigenvalueSolver::GetDampedFrequencies(const ChVectorDynamic<ScalarType>& eigvals,
                                                      ChVectorDynamic<double>& damped_freq) {
    damped_freq.resize(eigvals.size());

    for (int i = 0; i < eigvals.size(); ++i) {
        damped_freq(i) = eigvals(i).imag() / CH_2PI;
    }
}

void ChUnsymGenEigenvalueSolver::GetDampingRatios(const ChVectorDynamic<ScalarType>& eigvals,
                                                  ChVectorDynamic<double>& damp_ratios) {
    damp_ratios.resize(eigvals.size());

    for (int i = 0; i < eigvals.size(); ++i) {
        damp_ratios(i) = -eigvals(i).real() / std::abs(eigvals(i));
    }
}

void CountNonZerosForEachRow(const ChSparseMatrix& Q, Eigen::VectorXi& nonZerosPerRow, int offset) {
    // void CountNonZerosForEachRowTransposed(const Eigen::SparseMatrix<double, Eigen::ColMajor, int>& Q,
    // Eigen::VectorXi& nonZerosPerRow, int offset) {

    for (auto row_i = 0; row_i < Q.outerSize(); row_i++) {
        nonZerosPerRow[row_i + offset] +=
            Q.isCompressed() ? Q.outerIndexPtr()[row_i + 1] - Q.outerIndexPtr()[row_i] : Q.innerNonZeroPtr()[row_i];
    }
}

void CountNonZerosForEachRowTransposed(const ChSparseMatrix& Q_transp, Eigen::VectorXi& nonZerosPerRow, int offset) {
    // void CountNonZerosForEachRow(const Eigen::SparseMatrix<double, Eigen::ColMajor, int>& Q,
    // Eigen::VectorXi& nonZerosPerRow, int offset) {
    for (auto k = 0; k < Q_transp.outerSize(); ++k)
        for (ChSparseMatrix::InnerIterator it(Q_transp, k); it; ++it)
            nonZerosPerRow[it.col() + offset]++;
}

ChUnsymGenEigenvalueSolverKrylovSchur::ChUnsymGenEigenvalueSolverKrylovSchur(
    std::shared_ptr<ChDirectSolverLScomplex> linear_solver)
    : m_linear_solver(linear_solver) {}

int ChUnsymGenEigenvalueSolverKrylovSchur::Solve(const ChSparseMatrix& A,  ///< input A matrix
                                                 const ChSparseMatrix& B,  ///< input B matrix
                                                 ChMatrixDynamic<ScalarType>& eigvects,
                                                 ChVectorDynamic<ScalarType>& eigvals,
                                                 int num_modes,
                                                 ScalarType sigma) const {
    int m = std::max(2 * num_modes, m_min_subspace_size);

    if (m > A.rows() - 1)
        m = A.rows() - 1;

    // Setup the Krylov Schur solver:
    m_timer_eigen_setup.start();
    // ChVectorDynamic<ScalarType> eigen_values;
    // ChMatrixDynamic<ScalarType> eigen_vectors;
    ChVectorDynamic<ScalarType> v1;
    v1.setRandom(A.cols());  // note: to make deterministic may be preceded by something like  std::srand((unsigned
                             // int)1234567);

    // Setup the callback for matrix * vector

    callback_Ax_sparse_complexshiftinvert Ax_function3(A, B, sigma, m_linear_solver);

    m_timer_eigen_setup.stop();

    m_timer_eigen_solver.start();

    bool isC, flag;
    int nconv, niter;
    ChKrylovSchurEig eigen_solver(
        eigvects,        ///< output matrix with eigenvectors as columns, will be resized
        eigvals,         ///< output vector with eigenvalues (real part not zero if some damping), will be resized
        isC,             ///< 0 = k-th eigenvalue is real, 1= k-th and k-th+1 are complex conjugate pairs
        flag,            ///< 0 = has converged, 1 = hasn't converged
        nconv,           ///< number of converged eigenvalues
        niter,           ///< number of used iterations
        &Ax_function3,   ///< callback for A*v
        v1,              ///< initial approx of eigenvector, or random
        A.rows(),        ///< size of A
        num_modes,       ///< number of needed eigenvalues
        m,               ///< Krylov restart threshold (largest dimension of krylov subspace)
        max_iterations,  ///< max iteration number
        tolerance        ///< tolerance
    );

    m_timer_eigen_solver.stop();

    m_timer_solution_postprocessing.start();

    // Restore eigenvals, trasform back from shift-inverted problem to original problem:
    for (int i = 0; i < eigvals.rows(); ++i) {
        eigvals(i) = (1.0 / eigvals(i)) + sigma;
    }

    if (sort_ritz_pairs)
        SortRitzPairs(eigvals, eigvects, [](const ChVectorDynamic<ScalarType>& eigv, int a, int b) {
            return std::abs(eigv(a)) < std::abs(eigv(b));
        });

    // auto perm = GetPermutationMatrix(nconv, [&](int a, int b) {
    //     return std::abs(eigvals[a]) < std::abs(eigvals[b]);
    // });
    // eigvects = eigvects * perm;
    // eigvals = perm.transpose() * eigvals;

    m_timer_solution_postprocessing.stop();

    if (verbose) {
        if (flag == 1) {
            std::cout << "KrylovSchurEig FAILED." << std::endl;
            std::cout << " shift   = (" << sigma.real() << "," << sigma.imag() << ")" << std::endl;
            std::cout << " nconv = " << nconv << std::endl;
            std::cout << " niter = " << niter << std::endl;
        } else {
            std::cout << "KrylovSchurEig successful." << std::endl;
            std::cout << " shift   = (" << sigma.real() << "," << sigma.imag() << ")" << std::endl;
            std::cout << " nconv   = " << nconv << std::endl;
            std::cout << " niter   = " << niter << std::endl;
        }

        ChVectorDynamic<ScalarType> resCallback(A.rows());
        for (int i = 0; i < nconv; i++) {
            ChVectorDynamic<ScalarType> temp;
            Ax_function3.compute(temp, eigvects.col(i));
            resCallback = temp - eigvects.col(i) / (eigvals(i) - sigma);
            std::cout << "   Eig " << i << "= " << eigvals(i).real() << ", " << eigvals(i).imag() << "i "
                      << "   freq= " << GetNaturalFrequency(eigvals(i)) << " Hz"
                      << "; res: " << resCallback.lpNorm<Eigen::Infinity>() << std::endl;
        }
    }

    return nconv;
}

}  // end namespace modal

}  // end namespace chrono
