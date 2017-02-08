// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CHLINEARALGEBRA_H
#define CHLINEARALGEBRA_H

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChMatrixDynamic.h"

#define PIVOT_ACCEPT_TRESHOLD 0.8
#define ACCEPT_PIVOT 0.001
#define MIN_PIVOT 0.00000001
#define INFINITE_PIVOT 1.e+34

namespace chrono {

// template <class Real = double>
class ChLinearAlgebra {
  public:
    /// Gets the best row index to swap with the "i"current, i.e
    /// the one with highest element in the "i" column.
    static int BestPivotRow(ChMatrix<>& A, int current) {
        double temp = 0;
        int i;
        int pivotrow = current;

        for (i = current; i < A.GetRows(); i++) {
            if (fabs(A.GetElement(i, current)) > temp) {
                temp = fabs(A.GetElement(i, current));
                pivotrow = i;
            }
            if (temp >= PIVOT_ACCEPT_TRESHOLD)
                break;
        }
        return pivotrow;
    }

    /// Gets the best row-col index to swap with the "i"current,
    /// using the diagonal pivot.
    static int BestPivotDiag(ChMatrix<>& A, int current) {
        double temp = 0;
        int i;
        int pivotrow = current;

        for (i = current; i < A.GetRows(); i++) {
            if (fabs(A.GetElement(i, i)) > temp) {
                temp = fabs(A.GetElement(i, i));
                pivotrow = i;
            }
            if (temp >= PIVOT_ACCEPT_TRESHOLD)
                break;
        }
        return pivotrow;
    }

    /// Performs a diagonal pivot on a generic square matrix.
    static void DiagPivot(ChMatrix<>& A, int rowA, int rowB) {
        A.SwapColumns(rowA, rowB);
        A.SwapRows(rowA, rowB);
    }

    /// Performs a diagonal pivot on a _symmetric_ square matrix,
    /// acting only on the _UPPER_ triangle, for storage convenience
    /// and speed optimizations.
    static void DiagPivotSymmetric(ChMatrix<>& A, int rowA, int rowB) {
        int elcol, elrow;
        double temp;
        for (elrow = 0; elrow < rowA; elrow++) {
            temp = A.GetElement(elrow, rowA);
            A.SetElement(elrow, rowA, A.GetElement(elrow, rowB));
            A.SetElement(elrow, rowB, temp);
        }
        for (elcol = rowB + 1; elcol < A.GetColumns(); elcol++) {
            temp = A.GetElement(rowA, elcol);
            A.SetElement(rowA, elcol, A.GetElement(rowB, elcol));
            A.SetElement(rowB, elcol, temp);
        }
        for (elcol = rowA + 1; elcol < rowB; elcol++) {
            temp = A.GetElement(rowA, elcol);
            A.SetElement(rowA, elcol, A.GetElement(elcol, rowB));
            A.SetElement(elcol, rowB, temp);
        }
        temp = A.GetElement(rowA, rowA);
        A.SetElement(rowA, rowA, A.GetElement(rowB, rowB));
        A.SetElement(rowB, rowB, temp);
    }

    /// Gauss resolution of the system [A]x=b, with square [A] as "this" object.
    /// Returns error code (0 = ok, >0 number of equations with zero pivot)
    /// It must get an array of integers as "int* pivarray", where the pivot
    /// ordering is written. (that array must be freed and allocated by the user,
    /// or must be a NULL pointer if the user is not interested).
    /// Computes the determinant too.
    /// Speed optimized for null-headed rows.
    static int Solve_LinSys(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X, int* pivarray, double* det) {
        int i, j, k, pivrow, eqpivoted;
        double r, bi, aj, x, sum, pivot, pivlast;
        int err = 0;

        if (pivarray)  // initialize pivot index array
        {
            for (int ind = 0; ind < A.GetRows(); ind++) {
                pivarray[ind] = ind;
            }
        }

        *det = 1;

        // FORWARD reduction
        for (k = 1; k < A.GetRows(); k++) {
            pivot = A.GetElement((k - 1), (k - 1));

            if (fabs(pivot) < ACCEPT_PIVOT) {
                // pivoting is needed, so swap equations
                pivrow = BestPivotRow(A, k - 1);
                A.SwapRows((k - 1), pivrow);
                B->SwapRows((k - 1), pivrow);
                *det = -*det;
                if (pivarray) {
                    eqpivoted = pivarray[pivrow];  // swap eq.ids in pivot array
                    pivarray[pivrow] = pivarray[k - 1];
                    pivarray[k - 1] = eqpivoted;
                }
                pivot = A.GetElement((k - 1), (k - 1));  // fetch new pivot
            }
            if (fabs(pivot) <= MIN_PIVOT)  // was unable to find better pivot: force solution to zero.and go ahead
            {
                *det = 0;
                pivot = INFINITE_PIVOT;
                A.SetElement(k - 1, k - 1, pivot);
                if (!err)
                    err = (1 + A.GetRows() - k);  // report deficiency
            } else
                *det = *det * pivot;

            for (i = k; i < A.GetRows(); i++) {
                r = (A.GetElement(i, (k - 1))) / pivot;

                if (r) {
                    bi = B->GetElement(i, 0) - r * B->GetElement((k - 1), 0);
                    B->SetElement(i, 0, bi);

                    for (j = k; j < A.GetRows(); j++) {
                        aj = A.GetElement(i, j) - r * A.GetElement((k - 1), j);
                        A.SetElement(i, j, aj);
                    }
                }
            }
        }

        pivlast = A.GetElement((A.GetRows() - 1), (A.GetRows() - 1));
        if (fabs(pivlast) <= MIN_PIVOT) {
            *det = 0;
            pivlast = INFINITE_PIVOT;
            A.SetElement(A.GetRows() - 1, A.GetRows() - 1, pivlast);
            if (!err)
                err = (1);  // report deficiency
        } else
            *det = *det * pivlast;

        // BACKWARD substitution
        double xlast = B->GetElement(A.GetRows() - 1, 0) / pivlast;
        X->SetElement((A.GetRows() - 1), 0, xlast);

        for (k = (A.GetRows() - 2); k >= 0; k--) {
            sum = 0;
            for (j = (k + 1); j < A.GetRows(); j++) {
                sum += (A.GetElement(k, j)) * (X->GetElement(j, 0));
            }
            x = (B->GetElement(k, 0) - sum) / A.GetElement(k, k);
            X->SetElement(k, 0, x);
        }

        return err;
    }

    /// As before, but simplified, without needing the pivot vector.
    static void Solve_LinSys(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X) {
        double det;
        Solve_LinSys(A, B, X, (int*)NULL, &det);
    }

    /// LU decomposition: [A]=[L][U], with square [A] as "this" object.
    /// Decomposition happens in place. (matrix [A] is overwritten)
    /// Pivot array must exist! (it will be filled with the _row_ pivots, if any)
    static int Decompose_LU(ChMatrix<>& A, int* pivarray, double* det) {
        int i, j, k, pivrow, eqpivoted;
        double r, aj, pivot;
        int err = 0;
        int rows = A.GetRows();

        if (pivarray) {
            // initialize pivot index array
            for (int ind = 0; ind < rows; ind++) {
                pivarray[ind] = ind;
            }
        } else {
            return 1;  // error: no pivot array.
        }

        *det = 1;

        for (k = 1; k < rows; k++) {
            pivot = A.GetElement((k - 1), (k - 1));

            if (fabs(pivot) < ACCEPT_PIVOT) {
                // pivoting is needed, so swap equations
                pivrow = BestPivotRow(A, k - 1);
                A.SwapRows((k - 1), pivrow);
                *det = -*det;

                eqpivoted = pivarray[pivrow];  // swap eq.ids in pivot array
                pivarray[pivrow] = pivarray[k - 1];
                pivarray[k - 1] = eqpivoted;

                pivot = A.GetElement((k - 1), (k - 1));  // fetch new pivot
            }

            if (fabs(pivot) <= MIN_PIVOT) {
                // was unable to find better pivot: force solution to zero.and go ahead
                *det = 0;
                pivot = INFINITE_PIVOT;
                A.SetElement(k - 1, k - 1, pivot);
                if (!err)
                    err = (1 + rows - k);  // report deficiency
            } else {
                *det = *det * pivot;
            }

            for (i = k; i < rows; i++) {
                r = (A.GetElement(i, (k - 1))) / pivot;  // compute multiplier

                A.SetElement(i, (k - 1), r);  // store the multiplier in L part

                if (r) {
                    for (j = k; j < rows; j++) {
                        aj = A.GetElement(i, j) - r * A.GetElement((k - 1), j);  // the U part
                        A.SetElement(i, j, aj);
                    }
                }
            }
        }

        pivot = A.GetElement((rows - 1), (rows - 1));
        if (fabs(pivot) <= MIN_PIVOT) {
            *det = 0;
            pivot = INFINITE_PIVOT;
            A.SetElement(rows - 1, rows - 1, pivot);
            if (!err)
                err = 1;  // report deficiency
        } else
            *det = *det * pivot;

        return err;
    }

    /// After LU decomposition, with Decompose_LU(),
    /// call this to solve [A]X=B, as [L][U]X=B
    static void Solve_LU(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X, int* pivarray) {
        int k, j;
        double sum, x;
        int rows = A.GetRows();

        // BACKWARD substitution - L
        double xlast = B->GetElement(pivarray[0], 0);  //  ../ GetElement(0,0) => ../1;
        X->SetElement(0, 0, xlast);

        for (k = 1; k < rows; k++) {
            sum = 0;
            for (j = 0; j < k; j++) {
                sum += (A.GetElement(k, j)) * (X->GetElement(j, 0));
            }
            x = (B->GetElement(pivarray[k], 0) - sum);  //  ../ GetElement(k,k) => ../1;
            X->SetElement(k, 0, x);
        }

        // BACKWARD substitution - U
        xlast = X->GetElement((rows - 1), 0) / A.GetElement(rows - 1, rows - 1);
        X->SetElement((rows - 1), 0, xlast);

        for (k = (rows - 2); k >= 0; k--) {
            sum = 0;
            for (j = (k + 1); j < rows; j++) {
                sum += (A.GetElement(k, j)) * (X->GetElement(j, 0));
            }
            x = (X->GetElement(k, 0) - sum) / A.GetElement(k, k);
            X->SetElement(k, 0, x);
        }
    }

    /// From this matrix, decomposed 'in place' with Decompose_LU(), fills the separate [L] and [U]
    static void Extract_LU(ChMatrix<>& A, ChMatrix<>* mL, ChMatrix<>* mU) {
        int i, j;
        int rows = A.GetRows();
        int columns = A.GetColumns();

        if (mL) {
            mL->Reset(rows, columns);
            for (i = 0; i < rows; i++)
                mL->SetElement(i, i, 1.0);  // diagonal = 1;
            for (i = 1; i < rows; i++)
                for (j = 0; j < i; j++)
                    mL->SetElement(i, j, A.GetElement(i, j));
        }
        if (mU) {
            mU->Reset(rows, columns);
            for (i = 0; i < rows; i++)
                for (j = i; j < columns; j++)
                    mU->SetElement(i, j, A.GetElement(i, j));
        }
    }

    /// LDLt decomposition: [A]=[L][D][L]', i.e. diagonal values [D], triangular matrix [L] and
    /// its transpose [L]'. The square [A] matrix is "this" object.
    /// Decomposition happens in place. (matrix [A] is overwritten)
    /// It works only for symmetric matrices [A].
    /// Pivot array must exist! (it will be filled with the full-diagonal pivots, if any)
    /// Note: only the upper part of [A] is used! That is, after decomposition, the diagonal
    /// of [A] ('this' matrix) will contain the [D] values, and the upper part will contain the [L]' values.
    static int Decompose_LDL(ChMatrix<>& A, int* pivarray, double* det) {
        int i, j, k, pivrow, eqpivoted;
        double r, leader, aj, pivot;
        int err = 0;
        int rows = A.GetRows();

        if (pivarray) {
            // initialize pivot index array
            for (int ind = 0; ind < rows; ind++) {
                pivarray[ind] = ind;
            }
        } else {
            return 1;  // error: no pivot array.
        }

        *det = 1;

        for (k = 1; k < rows; k++) {
            pivot = A.GetElement((k - 1), (k - 1));

            if (fabs(pivot) < ACCEPT_PIVOT) {
                // pivoting is needed, so ...
                pivrow = BestPivotDiag(A, k - 1);
                DiagPivotSymmetric(A, k - 1, pivrow);  // swap both column and row (only upper right!)
                *det = -*det;                          // invert determ.sign

                eqpivoted = pivarray[pivrow];  // swap diagonal pivot indexes
                pivarray[pivrow] = pivarray[k - 1];
                pivarray[k - 1] = eqpivoted;

                pivot = A.GetElement((k - 1), (k - 1));
            }

            if (fabs(pivot) <= MIN_PIVOT) {
                // was unable to find better pivot: force solution to zero.and go ahead
                *det = 0.0;
                pivot = INFINITE_PIVOT;
                A.SetElement(k - 1, k - 1, pivot);
                if (!err)
                    err = (1 + rows - k);  // report deficiency
            } else {
                *det = *det * pivot;
            }

            for (i = k; i < rows; i++) {
                leader = A.GetElement((k - 1), i);
                if (leader) {
                    r = (leader / pivot);  // compute multiplier (mirror look A)

                    for (j = i; j < rows; j++) {
                        // fill the remaining part (upper right)
                        aj = A.GetElement(i, j) - r * A.GetElement((k - 1), j);
                        A.SetElement(i, j, aj);
                    }

                    A.SetElement((k - 1), i, r);  // store the multiplier in L part (mirror look! is up)
                }
            }
        }

        pivot = A.GetElement((rows - 1), (rows - 1));
        if (fabs(pivot) <= MIN_PIVOT) {
            *det = 0;
            pivot = INFINITE_PIVOT;
            A.SetElement(rows - 1, rows - 1, pivot);
            if (!err)
                err = 1;  // report deficiency
        } else {
            *det = *det * pivot;
        }

        return err;
    }

    /// After LDL decomposition with Decompose_LDL(),
    /// call this to solve [A]X=B, as [L][D][L]'X=B
    static void Solve_LDL(ChMatrix<>& A, ChMatrix<>* B, ChMatrix<>* X, int* pivarray) {
        double sum, x;
        int j, k;
        int rows = A.GetRows();

        // BACKWARD substitution - L
        double xlast = B->GetElement(pivarray[0], 0);  //  ../ GetElement(0,0) => ../1;
        X->SetElement(pivarray[0], 0, xlast);

        for (k = 1; k < rows; k++) {
            sum = 0;
            for (j = 0; j < k; j++) {
                sum += (A.GetElement(j, k)) * (X->GetElement(pivarray[j], 0));
            }
            x = (B->GetElement(pivarray[k], 0) - sum);  //  ../ GetElement(k,k) => ../1;
            X->SetElement(pivarray[k], 0, x);
        }
        // BACKWARD substitution - D
        for (k = 0; k < rows; k++) {
            x = X->GetElement(pivarray[k], 0) / A.GetElement(k, k);
            X->SetElement(pivarray[k], 0, x);
        }
        // BACKWARD substitution - L'
        for (k = (rows - 2); k >= 0; k--) {
            sum = 0;
            for (j = (k + 1); j < rows; j++) {
                sum += (A.GetElement(k, j)) * (X->GetElement(pivarray[j], 0));
            }
            x = (X->GetElement(pivarray[k], 0) - sum);  // .../ GetElement(k,k); -> ../1 ;
            X->SetElement(pivarray[k], 0, x);
        }
    }

    /// From this matrix, decomposed 'in place' with Decompose_LDL(), fills the separate upper [L]' and [D]
    static void Extract_LDL(ChMatrix<>& A, ChMatrix<>* L, ChMatrix<>* D, ChMatrix<>* Lu) {
        int i, j;
        int rows = A.GetRows();
        int columns = A.GetColumns();

        if (L) {
            L->Reset(rows, columns);
            for (i = 0; i < rows; i++)
                L->SetElement(i, i, 1.0);  // diagonal = 1;
            for (i = 1; i < rows; i++)
                for (j = 0; j < i; j++)
                    L->SetElement(i, j, A.GetElement(j, i));
        }
        if (D) {
            D->Reset(rows, columns);
            for (i = 0; i < rows; i++)
                D->SetElement(i, i, A.GetElement(i, i));
        }
        if (Lu) {
            Lu->Reset(rows, columns);
            for (i = 0; i < rows; i++)
                Lu->SetElement(i, i, 1.0);  // diagonal = 1;
            for (i = 0; i < (rows - 1); i++)
                for (j = i + 1; j < columns; j++)
                    Lu->SetElement(i, j, A.GetElement(i, j));
        }
    }

    /// Gets the determinant (0 if singular, or rows not equal to columns)
    /// This is expensive as a LU decomposition, so better get determinant as a side effect of Decompose_LU() or such..
    static double Det(ChMatrix<>& A) {
        if (A.GetRows() != A.GetColumns())
            return 0;
        ChMatrixDynamic<double>* mtemp = new ChMatrixDynamic<double>;
        mtemp->CopyFromMatrix(A);
        double mdet = 0;
        int* pivarray = (int*)calloc((mtemp->GetRows()), sizeof(int));
        Decompose_LU(A, pivarray, &mdet);
        free(pivarray);
        delete mtemp;
        return mdet;
    }
    /// Inverts a square matrix using a LU decomposition (expensive computation!).
    /// so that   this = (mA)^-1
    /// If matrix is 3x3, use FastInvert() for better performance.
    /// \return Returns determinant.
    static double Invert(ChMatrix<>& A, ChMatrix<>* mA) {
        A.Reset(mA->GetRows(), mA->GetColumns());

        int rows = A.GetRows();
        int columns = A.GetColumns();

        if (rows != columns)
            return 0;

        int n = A.GetRows();
        double mdet = 0;
        ChMatrixDynamic<double> tempB(n, 1);
        ChMatrixDynamic<double> tempX(n, 1);
        int i, j;
        int* pivarray = (int*)calloc(n, sizeof(int));
        Decompose_LU(A, pivarray, &mdet);
        for (j = 0; j < n; j++) {
            for (i = 0; i < n; i++)
                tempB.SetElement(i, 0, 0.0);
            tempB.SetElement(j, 0, 1.0);
            Solve_LU(A, &tempB, &tempX, pivarray);
            for (i = 0; i < n; i++)
                A.SetElement(i, j, tempX.GetElement(i, 0));
        }
        free(pivarray);
        return mdet;
    }

    /// In-place inversion of a square matrix.
    static double Invert(ChMatrix<>& A) {
        ChMatrixDynamic<>* mA = new ChMatrixDynamic<>(A.GetRows(), A.GetColumns());
        mA->CopyFromMatrix(A);
        double mdet = Invert(A, mA);
        delete mA;
        return mdet;
    }

    static inline double ch_sign(double a, double b) {
        if (b >= 0.0)
            return (fabs(a));
        else
            return (-fabs(a));
    }

    /// Performs SVD decomposition. Matrices U,W,V are automatically resized
    /// if passed with wrong size.
    /// [A] is 'this' matrix. so that [A]=[U][W][V]'
    /// Also computes condition number (near 1= well cond, if ->infinite, A tend to simgularity)
    static int SVD(ChMatrix<>& mA, ChMatrix<>& U, ChMatrix<>& W, ChMatrix<>& V, double& cond_num) {
        const double MACHEP = 1.4e-17;  // measure of machine precision for SVD
        const double MAXCN = 1.0e+12;

        ChMatrix<double>* A = &mA;
        int i, j, k, l, mn, its, n, m;
        double c, f, g, h, s, x, y, z, eps, scale, machep;
        double* rv1;
        double u, v, w;  // cond_num;

        machep = MACHEP;

        // get dimension of input matrix
        n = A->GetRows();
        m = A->GetColumns();

        rv1 = new double[m];  // work space

        // Redimension return matrices
        U.Reset(n, m);
        V.Reset(m, m);
        W.Reset(m, m);

        // U=A' <=== ??????
        //	U=CopyFromMatrixT(A); ???
        U.CopyFromMatrix(*A);

        // Begin housholder reduction

        g = x = scale = 0.0;

        for (i = 0; i < m; i++) {
            l = i + 1;
            rv1[i] = scale * g;
            g = s = scale = 0.0;

            if (i < n)
                for (k = i; k < n; k++) {
                    if (U(k, i) > 0.0)
                        scale += U(k, i);
                    else
                        scale -= U(k, i);
                }

            if (scale != 0.0) {
                for (k = i; k < n; k++)
                    U(k, i) /= scale;
                for (k = i; k < n; k++)
                    s += U(k, i) * U(k, i);
                f = U(i, i);
                g = -ch_sign(sqrt(s), f);
                h = f * g - s;
                U(i, i) = f - g;
                if (i != m - 1)
                    for (j = l; j < m; j++) {
                        s = 0.0;
                        for (k = i; k < n; k++)
                            s += U(k, i) * U(k, j);
                        f = s / h;
                        for (k = i; k < n; k++)
                            U(k, j) += f * U(k, i);
                    }
                for (k = i; k < n; k++)
                    U(k, i) *= scale;
            }

            W(i, i) = scale * g;
            g = s = scale = 0.0;

            if (i < n && i != m - 1) {
                for (k = l; k < m; k++) {
                    if (U(i, k) > 0.0)
                        scale += U(i, k);
                    else
                        scale -= U(i, k);
                }
                if (scale != 0.0) {
                    for (k = l; k < m; k++)
                        U(i, k) /= scale;
                    for (k = l; k < m; k++)
                        s += U(i, k) * U(i, k);
                    f = U(i, l);
                    g = -ch_sign(sqrt(s), f);
                    h = f * g - s;
                    U(i, l) = f - g;
                    for (k = l; k < m; k++)
                        rv1[k] = U(i, k) / h;

                    if (i != n - 1)
                        for (j = l; j < n; j++) {
                            s = 0.0;
                            for (k = l; k < m; k++)
                                s += U(j, k) * U(i, k);
                            for (k = l; k < m; k++)
                                U(j, k) += s * rv1[k];
                        }
                    for (k = l; k < m; k++)
                        U(i, k) *= scale;
                }
            }

            y = fabs(W(i, i)) + fabs(rv1[i]);
            if (y > x)
                x = y;
        }

        for (i = m - 1; i >= 0; i--) {
            if (i != m - 1) {
                if (g != 0.0) {
                    // double division avoids possible underflow
                    for (j = l; j < m; j++)
                        V(j, i) = (U(i, j) / U(i, l)) / g;
                    for (j = l; j < m; j++) {
                        s = 0.0;
                        for (k = l; k < m; k++)
                            s += U(i, k) * V(k, j);
                        for (k = l; k < m; k++)
                            V(k, j) += s * V(k, i);
                    }
                }
                for (j = l; j < m; j++)
                    V(i, j) = 0.0;
                for (j = l; j < m; j++)
                    V(j, i) = 0.0;
            }
            V(i, i) = 1.0;
            g = rv1[i];
            l = i;
        }

        mn = n;
        if (m < n)
            mn = m; /* mn = min(m,n) */
        for (i = mn - 1; i >= 0; i--) {
            l = i + 1;
            g = W(i, i);
            if (i != m - 1)
                for (j = l; j < m; j++)
                    U(i, j) = 0.;

            if (g != 0.0) {
                if (i != mn - 1) {
                    for (j = l; j < m; j++) {
                        s = 0.0;
                        for (k = l; k < n; k++)
                            s += U(k, i) * U(k, j);
                        /* double division avoids possible underflow */
                        f = (s / U(i, i)) / g;
                        for (k = i; k < n; k++)
                            U(k, j) += f * U(k, i);
                    }
                }
                for (j = i; j < n; j++)
                    U(j, i) /= g;
            } else
                for (j = i; j < n; j++)
                    U(i, j) = 0.0;
            U(i, i) += 1.0;
        }

        eps = machep * x;
        for (k = m - 1; k >= 0; k--) {
            its = 0;

        testsplit:  // test for splitting

            for (l = k; l >= 0; l--) {
                if (fabs(rv1[l]) <= eps)
                    goto testcon;
                /* rv1[0] is always zero, so there is no exit
                through the bottom of the loop  */
                if (fabs(W(l - 1, l - 1)) <= eps)
                    goto cancellation;
            }

        cancellation:  // cancellation of rv1[l] if l greater than 0

            c = 0.0;
            s = 1.0;
            for (i = l; i <= k; i++) {
                f = s * rv1[i];
                rv1[i] = c * rv1[i];
                if (fabs(f) <= eps)
                    goto testcon;
                g = W(i, i);
                // h = sqrt(f*f+g*g); modified to avoid underflow
                if (fabs(f) > fabs(g))
                    h = fabs(f) * sqrt(1.0 + (g / f) * (g / f));
                else if (fabs(g) > fabs(f))
                    h = fabs(g) * sqrt(1.0 + (f / g) * (f / g));
                else
                    h = fabs(f) * sqrt(2.0);

                W(i, i) = h;
                c = g / h;
                s = -f / h;

                for (j = 0; j < n; j++) {
                    y = U(j, l - 1);
                    z = U(j, i);
                    U(j, l - 1) = y * c + z * s;
                    U(j, i) = -y * s + z * c;
                }
            }

        testcon:  // test for convergence

            z = W(k, k);
            if (l == k)
                goto convergence;
            if (its == 30)
                return (k + 1); /* no convergence to k'th */
            /* singular value */

            /* shift from bottom 2 by 2 minor */
            its++;
            x = W(l, l);
            y = W(k - 1, k - 1);
            g = rv1[k - 1];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            /* g = sqrt(f*f+1.0); modified to avoid over flow */

            if (fabs(f) > 1.0)
                g = fabs(f) * sqrt(1.0 + (1.0 / f) * (1.0 / f));
            else
                g = sqrt(f * f + 1.0);

            f = ((x - z) * (x + z) + h * (y / (f + ch_sign(g, f)) - h)) / x;

            /* next qr transformation */
            c = s = 1.0;
            for (i = l + 1; i <= k; i++) {
                g = rv1[i];
                y = W(i, i);
                h = s * g;
                g = c * g;
                /* z = sqrt(f*f+h*h); modified to avoid underflow */
                if (fabs(f) > fabs(h))
                    z = fabs(f) * sqrt(1.0 + (h / f) * (h / f));
                else if (fabs(h) > fabs(f))
                    z = fabs(h) * sqrt(1.0 + (f / h) * (f / h));
                else
                    z = fabs(f) * sqrt(2.0);

                rv1[i - 1] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = -x * s + g * c;
                h = y * s;
                y = y * c;

                for (j = 0; j < m; j++) {
                    x = V(j, i - 1);
                    z = V(j, i);
                    V(j, i - 1) = x * c + z * s;
                    V(j, i) = -x * s + z * c;
                }

                /* z = sqrt(f*f+h*h); modified to avoid underflow */
                if (fabs(f) > fabs(h))
                    z = fabs(f) * sqrt(1.0 + (h / f) * (h / f));
                else if (fabs(h) > fabs(f))
                    z = fabs(h) * sqrt(1.0 + (f / h) * (f / h));
                else
                    z = fabs(f) * sqrt(2.0);

                W(i - 1, i - 1) = z;
                if (z != 0.0) { /* rotation can be arbitrary if z is zero */
                    c = f / z;
                    s = h / z;
                }
                f = c * g + s * y;
                x = -s * g + c * y;
                for (j = 0; j < n; j++) {
                    y = U(j, i - 1);
                    z = U(j, i);
                    U(j, i - 1) = y * c + z * s;
                    U(j, i) = -y * s + z * c;
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            W(k, k) = x;
            goto testsplit;

        convergence:

            if (z < 0.0) { /* w[k] is made non-negative */
                W(k, k) = -z;
                for (j = 0; j < m; j++)
                    V(j, k) = -V(j, k);
            }
        }
        delete rv1;

        // sort by singular values
        for (i = 0; i < m - 1; i++) {
            k = i;
            w = W(i, i);
            for (j = i + 1; j < m; j++)
                if (W(j, j) >= w) {
                    k = j;
                    w = W(j, j);
                }

            if (k != i) {
                W(k, k) = W(i, i);
                W(i, i) = w;
                for (j = 0; j < m; j++) {
                    v = V(i, j);
                    V(i, j) = V(j, k);
                    V(j, k) = v;
                }
                for (j = 0; j < n; j++) {
                    u = U(j, i);
                    U(j, i) = U(j, k);
                    U(j, k) = u;
                }
            }
        }

        if (W(m - 1, m - 1) == 0.0) {
            cond_num = 1E30;
        } else {
            cond_num = W(0, 0) / W(m - 1, m - 1);
        }

        return (0);
    }

    /// Computes and returns the condition number, by SVD decomposition
    /// Warning: this may be an expensive computation.
    static double ConditionNumber(ChMatrix<>& A) {
        ChMatrixDynamic<double> U;
        ChMatrixDynamic<double> W;
        ChMatrixDynamic<double> V;
        double cond = 0;
        SVD(A, U, W, V, cond);
        return cond;
    }
};

}  // end namespace chrono

#endif
