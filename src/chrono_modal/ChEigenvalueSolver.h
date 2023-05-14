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
#include "chrono/core/ChTimer.h"

#include <complex>

namespace chrono {

// Forward references
class ChDirectSolverLScomplex;
class ChDirectSolverLS;
    
namespace modal {



/// Class for passing basic settings to the Solve() function of the various solvers 
class ChApiModal ChEigenvalueSolverSettings {
public:
    // parameter constructor: only the first parameter is mandatory 
    ChEigenvalueSolverSettings(
        int m_nmodes,              ///< optional: n. of desired lower eigenvalues. If =0, return all eigenvalues.
        int max_iters = 500,       ///< upper limit for the number of iterations. If too low might not converge.
        double mtolerance = 1e-10, ///< tolerance for the iterative solver. 
        bool mverbose = false,     ///< turn to true to see some diagnostic.
        std::complex<double> msigma = 1e-5,       ///< for shift&invert. Too small gives ill conditioning (no convergence). Too large misses rigid body modes.
        bool scaleCq = true
    ) :
        n_modes(m_nmodes),
        max_iterations(max_iters),
        tolerance(mtolerance),
        verbose(mverbose),
        sigma(msigma)
    {};

    virtual ~ChEigenvalueSolverSettings() {};

    int n_modes = 10;
    double tolerance = 1e-10;   ///< tolerance for the iterative solver. 
    std::complex<double> sigma = 1e-5;        ///< for shift&invert. Too small gives ill conditioning (no convergence). Too large misses rigid body modes.
    int max_iterations = 500;   ///< upper limit for the number of iterations. If too low might not converge.
    bool verbose = false;       ///< turn to true to see some diagnostic.
    bool scaleCq = true;
};

//---------------------------------------------------------------------------------------------



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
        ChEigenvalueSolverSettings settings = 0   ///< optional: settings for the solver, or n. of desired lower eigenvalues. If =0, return all eigenvalues.
    ) const = 0;
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
        ChEigenvalueSolverSettings settings = 0   ///< optional: settings for the solver, or n. of desired lower eigenvalues. If =0, return all eigenvalues.
    ) const override;

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
        ChEigenvalueSolverSettings settings = 0   ///< optional: settings for the solver, or n. of desired lower eigenvalues. If =0, return all eigenvalues.
    ) const  override;

};


//---------------------------------------------------------------------------------------------


/// Class for computing eigenvalues/eigenvectors for the undamped constrained system.
/// It dispatches the settings to some solver of ChGeneralizedEigenvalueSolver class.
/// It handles multiple runs of the solver if one wants to find specific ranges of frequencies.
/// Finally it guarantees that eigenvalues are sorted in the appropriate order of increasing frequency.
class ChApiModal ChModalSolveUndamped {
public:
    struct ChFreqSpan {
    public:
        int nmodes = 1;
        double freq = 1e-5;
    };

    /// Constructor for the case of N lower modes.
    /// Ex.
    ///  ChModalSolveUndamped(7);
    /// finds first 7 lowest modes, using default settings (i.e. the ChGeneralizedEigenvalueSolverLanczos).
    /// Ex. 
    ///  ChModalSolveUndamped(5, 1e-5, 500, 1e-10, false, ChGeneralizedEigenvalueSolverKrylovSchur());
    /// finds first 5 lowest modes using the ChGeneralizedEigenvalueSolverKrylovSchur() solver.
    ChModalSolveUndamped(
        int n_lower_modes,         ///< n of lower modes
        double base_freq = 1e-5,   ///< frequency to whom the nodes are clustered. Use 1e-5 to get n lower modes. As sigma in shift&invert, as: sigma = -pow(base_freq * CH_C_2PI, 2). Too small gives ill conditioning (no convergence). Too large misses rigid body modes.
        int max_iters = 500,       ///< upper limit for the number of iterations. If too low might not converge.
        double mtolerance = 1e-10, ///< tolerance for the iterative solver. 
        bool mverbose = false,     ///< turn to true to see some diagnostic.
        const ChGeneralizedEigenvalueSolver& asolver = ChGeneralizedEigenvalueSolverLanczos() /// solver to use (default Lanczos)
    ) :
        freq_spans({ {n_lower_modes, base_freq } }),
        max_iterations(max_iters),
        tolerance(mtolerance),
        verbose(mverbose),
        msolver(asolver)
    {};

    /// Constructor for the case of multiple spans of frequency analysis
    /// ex. ChModalSolveUndamped({{10,1e-5,},{5,40}} , 500) ;
    /// finds first 10 lower modes, then 5 modes closest to 40 Hz, etc., using
    /// multiple runs of the solver. Closest mean that some could be higher than 40Hz,
    /// others can be lower.
    /// Another example: suppose you want the 5 lowest modes, then you also are
    /// interested in 1 high frequency mode whose frequency is already know approximately, 
    /// ex. 205 Hz, then you can do ChModalSolveUndamped({{5,1e-5,},{1,205}}, ...).
    /// Note about overlapping ranges: if n-th run finds frequencies up to X Hz, and the (n+1)-th run finds some
    /// frequency with Y Hz where Y < X, then such Y mode(s) is discarded. 
    ChModalSolveUndamped(
        std::vector< ChFreqSpan > mfreq_spans, ///< vector of {nmodes,freq}_i , will provide first nmodes_i starting at freq_i per each i vector entry
        int max_iters = 500,       ///< upper limit for the number of iterations. If too low might not converge.
        double mtolerance = 1e-10, ///< tolerance for the iterative solver. 
        bool mverbose = false,     ///< turn to true to see some diagnostic.
        const ChGeneralizedEigenvalueSolver& asolver = ChGeneralizedEigenvalueSolverLanczos() /// solver to use (default Lanczos)
    ) :
        freq_spans(mfreq_spans),
        max_iterations(max_iters),
        tolerance(mtolerance),
        verbose(mverbose),
        msolver(asolver)
    {};

    virtual ~ChModalSolveUndamped() {};

    /// Solve the constrained eigenvalue problem (-wsquare*M + K)*x = 0 s.t. Cq*x = 0
    /// Return the n. of found modes, where n is not necessarily n_lower_modes (or the sum of ChFreqSpan::nmodes if multiple spans) 
    virtual int Solve(
        const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
        const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
        ChVectorDynamic<double>& freq   ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
    ) const;


    std::vector< ChFreqSpan > freq_spans;
    double tolerance = 1e-10;   ///< tolerance for the iterative solver. 
    int max_iterations = 500;   ///< upper limit for the number of iterations. If too low might not converge.
    bool verbose = false;       ///< turn to true to see some diagnostic.
    const ChGeneralizedEigenvalueSolver& msolver; 
};



//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------



/// Base interface class for eigensolvers for the damped dynamic problem 
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
        ChEigenvalueSolverSettings settings = 0   ///< optional: settings for the solver, or n. of desired lower eigenvalues. If =0, return all eigenvalues.
    ) const = 0;

    /// Get cumulative time for matrix assembly.
    double GetTimeMatrixAssembly() const { return m_timer_matrix_assembly(); }

    /// Get cumulative time eigensolver setup.
    double GetTimeEigenSetup() const { return m_timer_eigen_setup(); }

    /// Get cumulative time eigensolver solution.
    double GetTimeEigenSolver() const { return m_timer_eigen_solver(); }

    /// Get cumulative time for post-solver solution postprocessing.
    double GetTimeSolutionPostProcessing() const { return m_timer_solution_postprocessing(); }

protected:

    mutable ChTimer m_timer_matrix_assembly;    ///< timer for matrix assembly
    mutable ChTimer m_timer_eigen_setup;    ///< timer for eigensolver setup
    mutable ChTimer m_timer_eigen_solver;    ///< timer for eigensolver solution
    mutable ChTimer m_timer_solution_postprocessing;    ///< timer for conversion of eigensolver solution


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
        ChEigenvalueSolverSettings settings = 0   ///< optional: settings for the solver, or n. of desired lower eigenvalues. If =0, return all eigenvalues.
    ) const override;


};


/// Solves the eigenvalue problem with the Krylov-Schur iterative method.
/// This is an efficient method to compute only the lower n modes, ex. when there are so many degreees of 
/// freedom that it would make a full solution impossible.
/// It uses an iterative method and it exploits the sparsity of the matrices.
class ChApiModal ChQuadraticEigenvalueSolverKrylovSchur : public ChQuadraticEigenvalueSolver {
public:
    /// Default: uses Eigen::SparseQR as factorization for the shift&invert, 
    /// otherwise pass a custom complex sparse solver for faster factorization (ex. ChSolverComplexPardisoMKL)
    ChQuadraticEigenvalueSolverKrylovSchur(ChDirectSolverLScomplex* mlinear_solver = 0);

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
        ChEigenvalueSolverSettings settings = 0   ///< optional: settings for the solver, or n. of desired lower eigenvalues. 
    ) const override;

    ChDirectSolverLScomplex* linear_solver;
    

};





//---------------------------------------------------------------------------------------------


/// Class for computing eigenvalues/eigenvectors for the DAMPED constrained system.
/// It dispatches the settings to some solver of ChQuadraticEigenvalueSolver class.
/// It handles multiple runs of the solver if one wants to find specific ranges of frequencies.
/// Finally it guarantees that eigenvalues are sorted in the appropriate order of increasing eigenvalue modulus.
class ChApiModal ChModalSolveDamped {
public:
    struct ChFreqSpan {
    public:
        int nmodes = 1;
        double freq = 1e-5;
    };

    /// Constructor for the case of N lower modes.
    /// Ex.
    ///  ChModalSolveDamped(7); 
    /// finds first 7 lowest damped modes, using default settings (i.e. the ChQuadraticEigenvalueSolverNullspaceDirect solver).
    /// Ex. 
    ///  ChModalSolveDamped(5, 1e-5, 500, 1e-10, false, ChQuadraticEigenvalueSolverKrylovSchur()); 
    /// finds first 5 lowest damped modes using the ChQuadraticEigenvalueSolverKrylovSchur() solver.
    ChModalSolveDamped(
        int n_lower_modes,         ///< n of lower modes
        double base_freq = 1e-5,   ///< frequency to whom the nodes are clustered. Use 1e-5 to get n lower modes. As sigma in shift&invert, as: sigma = -pow(base_freq * CH_C_2PI, 2). Too small gives ill conditioning (no convergence). Too large misses rigid body modes.
        int max_iters = 500,       ///< upper limit for the number of iterations. If too low might not converge.
        double mtolerance = 1e-10, ///< tolerance for the iterative solver. 
        bool mverbose = false,     ///< turn to true to see some diagnostic.
        const ChQuadraticEigenvalueSolver& asolver = ChQuadraticEigenvalueSolverNullspaceDirect() /// solver to use (default: direct, null-space based)
    ) :
        freq_spans({ {n_lower_modes, base_freq } }),
        max_iterations(max_iters),
        tolerance(mtolerance),
        verbose(mverbose),
        msolver(asolver)
    {};

    /// Constructor for the case of multiple spans of frequency analysis
    /// ex. ChModalSolveDamped({{10,1e-5,},{5,40}} , 500);
    /// finds first 10 lower modes, then 5 modes closest to 40 Hz, etc., using
    /// multiple runs of the solver. Closest mean that some could be higher than 40Hz,
    /// others can be lower.
    /// Another example: suppose you want the 5 lowest modes, then you also are
    /// interested in 1 high frequency mode whose frequency is already know approximately, 
    /// ex. 205 Hz, then you can do ChGeneralizedEigenvalueSolverGeneric({{5,1e-5,},{1,205}}, ...).
    /// Note about overlapping ranges: if n-th run finds frequencies up to X Hz, and the (n+1)-th run finds some
    /// frequency with Y Hz where Y < X, then such Y mode(s) is discarded. 
    ChModalSolveDamped(
        std::vector< ChFreqSpan > mfreq_spans, ///< vector of {nmodes,freq}_i , will provide first nmodes_i starting at freq_i per each i vector entry
        int max_iters = 500,       ///< upper limit for the number of iterations. If too low might not converge.
        double mtolerance = 1e-10, ///< tolerance for the iterative solver. 
        bool mverbose = false,     ///< turn to true to see some diagnostic.
        const ChQuadraticEigenvalueSolver& asolver = ChQuadraticEigenvalueSolverNullspaceDirect() /// solver to use (default: direct, null-space based)
    ) :
        freq_spans(mfreq_spans),
        max_iterations(max_iters),
        tolerance(mtolerance),
        verbose(mverbose),
        msolver(asolver)
    {};

    virtual ~ChModalSolveDamped() {};

    /// Solve the constrained eigenvalue problem (-wsquare*M + K)*x = 0 s.t. Cq*x = 0
    /// Return the n. of found modes, where n is not necessarily n_lower_modes (or the sum of ChFreqSpan::nmodes if multiple spans) 
    virtual int Solve(
        const ChSparseMatrix& M,  ///< input M matrix, n_v x n_v
        const ChSparseMatrix& R,  ///< input R matrix, n_v x n_v  
        const ChSparseMatrix& K,  ///< input K matrix, n_v x n_v  
        const ChSparseMatrix& Cq, ///< input Cq matrix of constraint jacobians, n_c x n_v
        ChMatrixDynamic<std::complex<double>>& V,    ///< output matrix n x n_v with eigenvectors as columns, will be resized
        ChVectorDynamic<std::complex<double>>& eig,  ///< output vector with n eigenvalues, will be resized.
        ChVectorDynamic<double>& freq,               ///< output vector with n frequencies [Hz], as f=w/(2*PI), will be resized.
        ChVectorDynamic<double>& damp_ratios         ///< output vector with n damping ratios, will be resized.
    ) const;



protected:


    std::vector< ChFreqSpan > freq_spans;
    double tolerance = 1e-10;   ///< tolerance for the iterative solver. 
    int max_iterations = 500;   ///< upper limit for the number of iterations. If too low might not converge.
    bool verbose = false;       ///< turn to true to see some diagnostic.
    const ChQuadraticEigenvalueSolver& msolver;

};





}  // end namespace modal

}  // end namespace chrono

#endif
