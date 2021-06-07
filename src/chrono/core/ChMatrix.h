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
// Authors: Radu Serban, Alessandro Tasora
// =============================================================================

#ifndef CHMATRIX_H
#define CHMATRIX_H

// Include these before ChMatrixEigenExtensions
#include "chrono/serialization/ChArchive.h"
#include "chrono/serialization/ChArchiveAsciiDump.h"

// -----------------------------------------------------------------------------

namespace chrono {
// A collective tag for storing version in ArchiveIN / ArchiveOUT:
class ChMatrix_dense_version_tag {};
CH_CLASS_VERSION(ChMatrix_dense_version_tag, 1)
}  // end namespace chrono

// -----------------------------------------------------------------------------

// Define Eigen::eigen_assert_exception. Note that this must precede inclusion of Eigen headers.
// Required for Eigen 3.3.8 (bug in Eigen?)
namespace Eigen {
static const bool should_raise_an_assert = false;

// Used to avoid to raise two exceptions at a time in which case the exception is not properly caught.
// This may happen when a second exceptions is triggered in a destructor.
static bool no_more_assert = false;
////static bool report_on_cerr_on_assert_failure = true;

struct eigen_assert_exception {
    eigen_assert_exception(void) {}
    ~eigen_assert_exception() { Eigen::no_more_assert = false; }
};

struct eigen_static_assert_exception {
    eigen_static_assert_exception(void) {}
    ~eigen_static_assert_exception() { Eigen::no_more_assert = false; }
};
}  // end namespace Eigen

// -----------------------------------------------------------------------------

#define EIGEN_MATRIXBASE_PLUGIN "chrono/core/ChMatrixEigenExtensions.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "chrono/core/ChSparseMatrixEigenExtensions.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"

#include "chrono/ChConfig.h"
#include "chrono/core/ChTypes.h"

namespace chrono {

/// @addtogroup chrono_linalg
/// @{

// -----------------------------------------------------------------------------

/// Dense matrix with *dynamic size* (i.e., with size a run-time variable, unknown at compile time).
/// A ChMatrixDynamic is templated by the type of its coefficients (default: double).
template <typename T = double>
using ChMatrixDynamic = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/// Dense matrix with *fixed size* (known at compile time).
/// A ChMatrixNM is templated by the type of its coefficients and by the matrix dimensions (number of rows and columns).
template <typename T, int M, int N>
using ChMatrixNM = Eigen::Matrix<T, M, N, Eigen::RowMajor>;

////template <typename T, int M, int N>
////using ChMatrixNMnoalign = Eigen::Matrix<T, M, N, Eigen::RowMajor | Eigen::DontAlign>;

// -----------------------------------------------------------------------------

/// Column vector with *dynamic size* (i.e., with size a run-time variable, unknown at compile time).
/// A ChVectorDynamic is templated by the type of its coefficients (default: double).
template <typename T = double>
using ChVectorDynamic = Eigen::Matrix<T, Eigen::Dynamic, 1, Eigen::ColMajor>;

/// Row vector with *dynamic size* (i.e., with size a run-time variable, unknown at compile time).
/// A ChRowVectorDynamic is templated by the type of its coefficients (default: double).
template <typename T = double>
using ChRowVectorDynamic = Eigen::Matrix<T, 1, Eigen::Dynamic, Eigen::RowMajor>;

/// Column vector with *fixed size* (known at compile time).
/// A ChVectorN is templated by the type of its coefficients and the number of elements.
template <typename T, int N>
using ChVectorN = Eigen::Matrix<T, N, 1>;

/// Row vector with *fixed size* (known at compile time).
/// A ChRowVectorN is templated by the type of its coefficients and the number of elements.
template <typename T, int N>
using ChRowVectorN = Eigen::Matrix<T, 1, N, Eigen::RowMajor>;

// -----------------------------------------------------------------------------

//// RADU
//// Consider templating the following by precision

/// Reference to a dense matrix expression, with double coefficients.
/// This allows writing non-template functions that can accept either a ChMatrixDynamic or a ChMatrixNM.
using ChMatrixRef = Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

/// Constant reference to a dense matrix expression, with double coefficients.
/// This allows writing non-template functions that can accept either a ChMatrixDynamic or a ChMatrixNM.
using ChMatrixConstRef = const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>&;

/// Reference to a column vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a ChVectorDynamic or a ChVectorN.
using ChVectorRef = Eigen::Ref<Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>>;

/// Constant reference to a column vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a ChVectorDynamic or a ChRowVectorN.
using ChVectorConstRef = const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>>&;

/// Reference to a row vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a ChRowVectorDynamic or a CVectorN.
using ChRowVectorRef = Eigen::Ref<Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>>;

/// Constant reference to a row vector expression, with double coefficients.
/// This allows writing non-template functions that can accept either a ChRowVectorDynamic or a CVectorN.
using ChRowVectorConstRef = const Eigen::Ref<const Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>>&;

// -----------------------------------------------------------------------------

/// General-purpose column array with *dynamic size*.
/// This class provides easy-access to coefficient-wise operations.
template <typename T = double>
using ChArray = Eigen::Array<T, Eigen::Dynamic, 1, Eigen::ColMajor>;

// -----------------------------------------------------------------------------

/// Sparse matrix representation.
/// A ChSparseMatrix is an Eigen SparseMatrix with double coefficients, row-major storage order, and int indices.
using ChSparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor, int>;

// -----------------------------------------------------------------------------

/// Serialization of a dense matrix or vector into an ASCII stream (e.g. a file) in Matlab format.
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

/// Paste a given matrix into a sparse matrix at position (\a insrow, \a inscol).
/// The matrix \a matrFrom will be copied into \a matrTo[insrow : insrow + \a matrFrom.GetRows()][inscol : inscol +
/// matrFrom.GetColumns()] 
/// \param[out] matrTo The output sparse matrix
/// \param[in] matrFrom The source matrix that will be copied
/// \param[in] insrow The row index where the first element will be copied
/// \param[in] inscol The column index where the first element will be copied
/// \param[in] overwrite Indicate if the copied elements will overwrite existing elements or be summed to them
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

/// Serialization of a sparse matrix to an ASCI stream (e.g., a file) in Matlab sparse matrix format.
/// Note that row and column indices start at 1.
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

/// Serialization of a sparse matrix to an ASCII stream (for debugging; only the top-left 8x8 corner is printed).
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

/// @} chrono_linalg





}  // end namespace chrono

#endif
