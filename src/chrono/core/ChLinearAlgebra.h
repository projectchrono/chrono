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

#ifndef CHLINEARALGEBRA_H
#define CHLINEARALGEBRA_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"

namespace chrono {

// template <class Real = double>
class ChApi ChLinearAlgebra {
  public:
    /// Gets the best row index to swap with the "i"current, i.e
    /// the one with highest element in the "i" column.
    static int BestPivotRow(ChMatrix<>& A, int current);

    /// Gets the best row-col index to swap with the "i"current,
    /// using the diagonal pivot.
    static int BestPivotDiag(ChMatrix<>& A, int current);

    /// Performs a diagonal pivot on a generic square matrix.
    static void DiagPivot(ChMatrix<>& A, int rowA, int rowB);

    /// Performs a diagonal pivot on a _symmetric_ square matrix,
    /// acting only on the _UPPER_ triangle, for storage convenience
    /// and speed optimizations.
    static void DiagPivotSymmetric(ChMatrix<>& A, int rowA, int rowB);

    /// Gauss resolution of the system [A]x=b, with square [A] as "this" object.
    /// Returns error code (0 = ok, >0 number of equations with zero pivot)
    /// It must get an array of integers as "int* pivarray", where the pivot
    /// ordering is written. (that array must be freed and allocated by the user,
    /// or must be a NULL pointer if the user is not interested).
    /// Computes the determinant too.
    /// Speed optimized for null-headed rows.
    static int Solve_LinSys(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X, int* pivarray, double* det);

    /// As before, but simplified, without needing the pivot vector.
    static void Solve_LinSys(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X);

    /// LU decomposition: [A]=[L][U], with square [A] as "this" object.
    /// Decomposition happens in place. (matrix [A] is overwritten)
    /// Pivot array must exist! (it will be filled with the _row_ pivots, if any)
    static int Decompose_LU(ChMatrix<>& A, int* pivarray, double* det);

    /// After LU decomposition, with Decompose_LU(),
    /// call this to solve [A]X=B, as [L][U]X=B
    static void Solve_LU(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X, int* pivarray);

    /// From this matrix, decomposed 'in place' with Decompose_LU(), fills the separate [L] and [U]
    static void Extract_LU(ChMatrix<>& A, ChMatrix<>* mL, ChMatrix<>* mU);

    /// LDLt decomposition: [A]=[L][D][L]', i.e. diagonal values [D], triangular matrix [L] and
    /// its transpose [L]'. The square [A] matrix is "this" object.
    /// Decomposition happens in place. (matrix [A] is overwritten)
    /// It works only for symmetric matrices [A].
    /// Pivot array must exist! (it will be filled with the full-diagonal pivots, if any)
    /// Note: only the upper part of [A] is used! That is, after decomposition, the diagonal
    /// of [A] ('this' matrix) will contain the [D] values, and the upper part will contain the [L]' values.
    static int Decompose_LDL(ChMatrix<>& A, int* pivarray, double* det);

    /// After LDL decomposition with Decompose_LDL(),
    /// call this to solve [A]X=B, as [L][D][L]'X=B
    static void Solve_LDL(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X, int* pivarray);

    /// From this matrix, decomposed 'in place' with Decompose_LDL(), fills the separate upper [L]' and [D]
    static void Extract_LDL(ChMatrix<>& A, ChMatrix<>* L, ChMatrix<>* D, ChMatrix<>* Lu);

    /// Gets the determinant (0 if singular, or rows not equal to columns)
    /// This is expensive as a LU decomposition, so better get determinant as a side effect of Decompose_LU() or such..
    static double Det(ChMatrix<>& A);

    /// Inverts a square matrix using a LU decomposition (expensive computation!).
    /// so that   this = (mA)^-1
    /// If matrix is 3x3, use FastInvert() for better performance.
    /// \return Returns determinant.
    static double Invert(ChMatrix<>& A, ChMatrix<>* mA);

    /// In-place inversion of a square matrix.
    static double Invert(ChMatrix<>& A);

    /// Performs SVD decomposition. Matrices U,W,V are automatically resized
    /// if passed with wrong size.
    /// [A] is 'this' matrix. so that [A]=[U][W][V]'
    /// Also computes condition number (near 1= well cond, if ->infinite, A tend to singularity)
    static int SVD(ChMatrix<>& mA, ChMatrix<>& U, ChMatrix<>& W, ChMatrix<>& V, double& cond_num);

    /// Computes and returns the condition number, by SVD decomposition
    /// Warning: this may be an expensive computation.
    static double ConditionNumber(ChMatrix<>& A);
};

}  // end namespace chrono

#endif
