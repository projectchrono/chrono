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

#ifndef CHEIGENVALUESOLVER_H
#define CHEIGENVALUESOLVER_H

#include "chrono_modal/ChApiModal.h"
#include "chrono/core/ChMatrix.h"
#include <complex>

namespace chrono {
namespace modal {


/// Base interface class for eigensolvers for the undamped
/// constrained generalized problem (-wsquare*M + K)*x = 0  s.t. Cq*x = 0
/// Children classes can implement this in different ways, overridding Solve()
class ChApiModal ChGeneralizedEigenvalueSolver {
public:
    virtual ~ChGeneralizedEigenvalueSolver() {};

    /// Solve the constrained eigenvalue problem (-wsquare*M + K)*x = 0 s.t. Cq*x = 0
    /// If n_modes=0, return all eigenvalues, otherwise only the first lower n_modes. 
    virtual bool Solve(
        const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
        const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
        ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        int n_modes = 0             ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.
    ) = 0;
};

/// Solves the undamped constrained eigenvalue problem with the Krylov-Schur iterative method.
/// This is an efficient method to compute only the lower n modes, ex. when there are so many degreees of 
/// freedom that it would make a full solution impossible.
/// It uses an iterative method and it exploits the sparsity of the matrices.
class ChApiModal ChGeneralizedEigenvalueSolverKrylovSchur : public ChGeneralizedEigenvalueSolver {
public:
    virtual ~ChGeneralizedEigenvalueSolverKrylovSchur() {};

    /// Solve the constrained eigenvalue problem (-wsquare*M + K)*x = 0 s.t. Cq*x = 0
    /// If n_modes=0, return all eigenvalues, otherwise only the first lower n_modes. 
    virtual bool Solve(
        const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
        const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
        ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        int n_modes = 0             ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.
    );

    // Some default settings. One can modify them before calling Solve(), if needed.
    double tolerance = 1e-10;   ///< tolerance for the iterative solver. 
    double sigma = 1e-5;        ///< for shift&invert. Too small gives ill conditioning (no convergence). Too large misses rigid body modes.
    int max_iterations = 500;   ///< upper limit for the number of iterations. If too low might not converge.
    bool verbose = false;       ///< turn to true to see some diagnostic.
};

/// Solves the undamped constrained eigenvalue problem with the Lanczos iterative method. 
/// It assumes that K and M matrices are symmetric, hence a real eigenvalue problem.
/// This is an efficient method to compute only the lower n modes, ex. when there are so many degreees of 
/// freedom that it would make a full solution impossible.
/// It uses an iterative method and it exploits the sparsity of the matrices.
class ChApiModal ChGeneralizedEigenvalueSolverLanczos : public ChGeneralizedEigenvalueSolver {
public:
    virtual ~ChGeneralizedEigenvalueSolverLanczos() {};

    /// Solve the constrained eigenvalue problem (-wsquare*M + K)*x = 0 s.t. Cq*x = 0
    /// If n_modes=0, return all eigenvalues, otherwise only the first lower n_modes. 
    virtual bool Solve(
        const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
        const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
        ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        int n_modes = 0             ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.
    );

    // Some default settings. One can modify them before calling Solve(), if needed.
    double tolerance = 1e-10;   ///< tolerance for the iterative solver. 
    double sigma = 1e-5;        ///< for shift&invert. Too small gives ill conditioning (no convergence). Too large misses rigid body modes.
    int max_iterations = 500;   ///< upper limit for the number of iterations. If too low might not converge.
    bool verbose = false;       ///< turn to true to see some diagnostic.
};



//---------------------------------------------------------------------------------------------


/// Base interface class for eigensolvers for the dynamic problem 
/// ie. the quadratic eigenvalue problem  (lambda^2*M + lambda*R + K)*x = 0
/// also (-w^2*M + i*w*R + K)*x = 0,  with complex w (where w.length() = undamped nat.freq)
/// Children classes can implement this in different ways, overridding Solve()
class ChApiModal ChQuadraticEigenvalueSolver {
public:
    virtual ~ChQuadraticEigenvalueSolver() {};

    /// Solve the quadratic eigenvalue problem (lambda^2*M + lambda*R + K)*x = 0 s.t. Cq*x = 0
    /// If n_modes=0, return all eigenvalues, otherwise only the first lower n_modes. 
    virtual bool Solve(
        const ChSparseMatrix& M,  ///< input M matrix
        const ChSparseMatrix& R,  ///< input R matrix
        const ChSparseMatrix& K,  ///< input K matrix
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with eigenvalues (real part not zero if some damping), will be resized
        ChVectorDynamic<double>& freq,  ///< output vector with n undamped frequencies [Hz], as f=w/(2*PI), will be resized.
        ChVectorDynamic<double>& damping_ratio,  ///< output vector with n damping rations r=damping/critical_damping.
        int n_modes = 0             ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.
    ) = 0;
};

/// Solves the eigenvalue problem with a direct method: first does LU factorization of Cq jacobians
/// to find the null space, then solves the problem using the direct  Eigen::EigenSolver.
/// Note: since intermediate dense matrices are built, the performance is acceptable only for small-sized problems.
/// Note: since the method is direct, all eigenvalues are computed, regardless of n_modes, but only lower n_modes are returned.
class ChApiModal ChQuadraticEigenvalueSolverNullspaceDirect : public ChQuadraticEigenvalueSolver {
public:
    virtual ~ChQuadraticEigenvalueSolverNullspaceDirect() {};

    /// Solve the quadratic eigenvalue problem (lambda^2*M + lambda*R + K)*x = 0 s.t. Cq*x = 0
    /// If n_modes=0, return all eigenvalues, otherwise only the first lower n_modes. 
    virtual bool Solve(
        const ChSparseMatrix& M, ///< input M matrix
        const ChSparseMatrix& R, ///< input R matrix
        const ChSparseMatrix& K, ///< input K matrix
        const ChSparseMatrix& Cq,   ///< input Cq matrix of constraint jacobians
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with complex eigenvalues (real part not zero if some damping), will be resized
        ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        ChVectorDynamic<double>& damping_ratio,  ///< output vector with n damping rations r=damping/critical_damping.
        int n_modes = 0             ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.
    );
};


/// Solves the eigenvalue problem with the Krylov-Schur iterative method.
/// This is an efficient method to compute only the lower n modes, ex. when there are so many degreees of 
/// freedom that it would make a full solution impossible.
/// It uses an iterative method and it exploits the sparsity of the matrices.
class ChApiModal ChQuadraticEigenvalueSolverKrylovSchur : public ChQuadraticEigenvalueSolver {
public:
    virtual ~ChQuadraticEigenvalueSolverKrylovSchur() {};

    /// Solve the quadratic eigenvalue problem (lambda^2*M + lambda*R + K)*x = 0 s.t. Cq*x = 0
    /// Returns only the first lower n_modes. If n_modes=0, it return all eigenvalues (performance warning)
    virtual bool Solve(
        const ChSparseMatrix& M, ///< input M matrix
        const ChSparseMatrix& R, ///< input R matrix
        const ChSparseMatrix& K, ///< input K matrix
        const ChSparseMatrix& Cq,   ///< input Cq matrix of constraint jacobians
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with complex eigenvalues (real part not zero if some damping), will be resized
        ChVectorDynamic<double>& freq,  ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        ChVectorDynamic<double>& damping_ratio,  ///< output vector with n damping rations r=damping/critical_damping.
        int n_modes = 0             ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.
    );

    // Some default settings. One can modify them before calling Solve(), if needed.
    double tolerance = 1e-10;   ///< tolerance for the iterative solver. 
    double sigma = 1e-5;        ///< for shift&invert. Too small gives ill conditioning (no convergence). Too large misses rigid body modes.
    int max_iterations = 500;   ///< upper limit for the number of iterations. If too low might not converge.
    bool verbose = false;       ///< turn to true to see some diagnostic.
};



}  // end namespace modal

}  // end namespace chrono

#endif
