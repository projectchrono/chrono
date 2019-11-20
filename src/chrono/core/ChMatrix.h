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

#ifndef CHMATRIX_H
#define CHMATRIX_H

#define EIGEN_MATRIXBASE_PLUGIN "chrono/core/ChMatrixEigenExtensions.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "chrono/core/ChSparseMatrixEigenExtensions.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"

#include "chrono/ChConfig.h"
#include "chrono/core/ChTypes.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

// -----------------------------------------------------------------------------

template <typename T = double>
using ChMatrixDynamic = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

template <typename T, int M, int N>
using ChMatrixNM = Eigen::Matrix<T, M, N, Eigen::RowMajor>;

template <typename T, int M, int N>
using ChMatrixNMnoalign = Eigen::Matrix<T, M, N, Eigen::RowMajor | Eigen::DontAlign>;

// -----------------------------------------------------------------------------

template <typename T = double>
using ChVectorDynamic = Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor>;

template <typename T = double>
using ChRowVectorDynamic = Eigen::Matrix<T, 1, Eigen::Dynamic, Eigen::RowMajor>;

template <typename T, int N>
using ChVectorN = Eigen::Matrix<T, N, 1>;

template <typename T, int N>
using ChRowVectorN = Eigen::Matrix<T, 1, N, Eigen::RowMajor>;

// -----------------------------------------------------------------------------

//// RADU
//// Consider templating the following by precision

using ChMatrixRef = Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
using ChMatrixConstRef = const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>&;

using ChVectorRef = Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>>;
using ChVectorConstRef = const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>>&;

using ChRowVectorRef = Eigen::Ref<Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>>;
using ChRowVectorConstRef = const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>>&;

// -----------------------------------------------------------------------------

template <typename T = double>
using ChArray = Eigen::Array<T, Eigen::Dynamic, 1, Eigen::ColMajor>;

// -----------------------------------------------------------------------------

// Sparse matrices
using ChSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor, int>;

// -----------------------------------------------------------------------------

/// Serialization of a matrix into an ASCII stream (e.g. a file) as a Matlab .dat output.
inline void StreamOUTdenseMatlabFormat(ChMatrixConstRef A, ChStreamOutAscii& stream) {
    for (int ii = 0; ii < A.rows(); ii++) {
        for (int jj = 0; jj < A.cols(); jj++) {
            stream << A(ii, jj);
            if (jj < A.cols() - 1)
                stream << " ";
        }
        stream << "\n";
    }
}

//// RADU
//// TODO: serialization of matrix classes

//// RADU
//// Implement some utilities to abstract use of Eigen::Map to copy vectors into matrices and vice-versa

// -----------------------------------------------------------------------------

/// Paste a given matrix into \a this sparse matrix at position (\a insrow, \a inscol).
/// The matrix \a matrFrom will be copied into \a this[insrow : insrow + \a matrFrom.GetRows()][[inscol : inscol +
/// matrFrom.GetColumns()]] \param[in] matrFrom The source matrix that will be copied; \param[in] insrow The row index
/// where the first element will be copied; \param[in] inscol The column index where the first element will be
/// copied; \param[in] overwrite Tells if the copied element will overwrite an existing element or be summed to it;
inline void PasteMatrix(ChSparseMatrix& matrTo, ChMatrixConstRef matrFrom, int insrow, int inscol, bool overwrite = true) {
    if (overwrite) {
        for (auto i = 0; i < matrFrom.rows(); i++) {
            for (auto j = 0; j < matrFrom.cols(); j++) {
                matrTo.SetElement(insrow + i, inscol + j, matrFrom(i, j), true);
            }
        }
    } else {
        for (auto i = 0; i < matrFrom.rows(); i++) {
            for (auto j = 0; j < matrFrom.cols(); j++) {
                matrTo.SetElement(insrow + i, inscol + j, matrFrom(i, j), false);
            }
        }
    }
}

/// Method to allow serializing transient data into in ASCII stream (e.g., a file) as a
/// Matlab sparse matrix format; each row in file has three elements: {row, column, value}.
/// Note: the row and column indexes start from 1.
inline void StreamOUTsparseMatlabFormat(ChSparseMatrix& matr, ChStreamOutAscii& mstream) {
    for (int ii = 0; ii < matr.rows(); ii++) {
        for (int jj = 0; jj < matr.cols(); jj++) {
            double elVal = matr.coeff(ii, jj);
            if (elVal || (ii == matr.rows() - 1 && jj == matr.cols() - 1)) {
                mstream << ii + 1 << " " << jj + 1 << " " << elVal << "\n";
            }
        }
    }
}

inline void StreamOUT(ChSparseMatrix& matr, ChStreamOutAscii& stream) {
	int mrows = static_cast<int>(matr.rows());
	int mcols = static_cast<int>(matr.cols());
    stream << "\n"
        << "Matrix " << mrows << " rows, " << mcols << " columns."
        << "\n";
    for (int i = 0; i < std::min(mrows, 8); i++) {
        for (int j = 0; j < std::min(mcols, 8); j++)
            stream << static_cast<double>(matr.coeff(i, j)) << "  ";
        if (matr.cols() > 8)
            stream << "...";
        stream << "\n";
    }
    if (matr.rows() > 8)
        stream << "... \n\n";
}


}  // end namespace chrono

#endif
