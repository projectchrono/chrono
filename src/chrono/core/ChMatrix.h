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
#include "Eigen/Dense"

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

}  // end namespace chrono

#endif
