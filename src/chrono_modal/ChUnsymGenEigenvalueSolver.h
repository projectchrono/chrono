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

#ifndef CHUNSYMGENEIGENVALUESOLVER_H
#define CHUNSYMGENEIGENVALUESOLVER_H

#include "chrono_modal/ChApiModal.h"
#include "chrono/solver/ChDirectSolverLScomplex.h"
#include "chrono_modal/ChGeneralizedEigenvalueSolver.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChAssembly.h"

#include <complex>

namespace chrono {

// Forward references
class ChDirectSolverLScomplex;
class ChDirectSolverLS;

namespace modal {

/// Base interface class for iterative eigenvalue solvers for generalized problem with real generic matrices.
/// Assumes shift-and-invert methods.
class ChApiModal ChUnsymGenEigenvalueSolver : public ChGeneralizedEigenvalueSolver<std::complex<double>> {
  public:
    ChUnsymGenEigenvalueSolver() {}
    virtual ~ChUnsymGenEigenvalueSolver(){};

    /// Solve the generalized eigenvalue problem A*eigvects = B*eigvects*diag(eigvals)
    /// A and B are real; potentially unsymmetric
    /// 'eigvects' will be resized to [A.rows() x num_modes]
    /// 'eigvals' will be resized to [num_modes]
    /// the number of converged eigenvalues is returned.
    virtual int Solve(const ChSparseMatrix& A,
                      const ChSparseMatrix& B,
                      ChMatrixDynamic<ScalarType>& eigvects,
                      ChVectorDynamic<ScalarType>& eigvals,
                      int num_modes,
                      ScalarType sigma) const = 0;

    static void GetNaturalFrequencies(const ChVectorDynamic<ScalarType>& eigvals,
                                      ChVectorDynamic<double>& natural_freq);

    static void GetDampedFrequencies(const ChVectorDynamic<ScalarType>& eigvals, ChVectorDynamic<double>& damped_freq);

    static void GetDampingRatios(const ChVectorDynamic<ScalarType>& eigvals, ChVectorDynamic<double>& damp_ratios);

    static double GetNaturalFrequency(ScalarType eigval) { return std::abs(eigval) / CH_2PI; };

    static ScalarType GetOptimalShift(double frequency) { return frequency * CH_2PI; };
};

/// Generalized iterative eigenvalue solver implementing Krylov-Schur shift-and-invert method for real generic matrices.
/// Features:
/// - generalized eigenvalue problem
/// - generic (i.e. also unsymmetric) sparse matrices
/// - shift-and-invert with complex shift
class ChApiModal ChUnsymGenEigenvalueSolverKrylovSchur : public ChUnsymGenEigenvalueSolver {
  public:
    /// Default: uses Eigen::SparseLU as factorization for the shift&invert,
    /// otherwise pass a custom complex sparse solver for faster factorization (ex. ChSolverComplexPardisoMKL)
    ChUnsymGenEigenvalueSolverKrylovSchur(
        std::shared_ptr<ChDirectSolverLScomplex> linear_solver = chrono_types::make_shared<ChSolverSparseComplexLU>());

    virtual ~ChUnsymGenEigenvalueSolverKrylovSchur(){};

    /// Solve the generalized eigenvalue problem A*eigvects = B*eigvects*diag(eigvals)
    /// A and B are real; potentially unsymmetric
    /// 'eigvects' will be resized to [A.rows() x num_modes]
    /// 'eigvals' will be resized to [num_modes]
    /// the number of converged eigenvalues is returned.
    virtual int Solve(const ChSparseMatrix& A,
                      const ChSparseMatrix& B,
                      ChMatrixDynamic<ScalarType>& eigvects,
                      ChVectorDynamic<ScalarType>& eigvals,
                      int num_modes,
                      ScalarType sigma) const override;

  protected:
    std::shared_ptr<ChDirectSolverLScomplex> m_linear_solver;
};

}  // end namespace modal

}  // end namespace chrono

#endif
