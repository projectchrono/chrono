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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================

#ifndef CHCSMATRIX_EIGEN_H
#define CHCSMATRIX_EIGEN_H

#define EIGEN_SPARSEMATRIX_PLUGIN "chrono/core/ChSparseMatrixEigenExtensions.h"
#include "Eigen/Sparse"

#include "ChSparseMatrix.h"

namespace chrono {
/// @addtogroup chrono
/// @{

using ChCSMatrix_Eigen = Eigen::SparseMatrix<double, Eigen::RowMajor>;


/// @} chrono

};  // end namespace chrono

#endif
