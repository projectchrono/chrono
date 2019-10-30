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
// Authors: Dario Mangoni
// =============================================================================
//
// Chrono-specific extensions to Eigen::SparseMatrixBase
//
// =============================================================================

#ifndef CHSPARSEMATRIXEIGENEXTENSIONS_H
#define CHSPARSEMATRIXEIGENEXTENSIONS_H

virtual void SetElement(int row, int col, double el, bool overwrite = true) {
    overwrite ? coeffRef(row, col) = el : coeffRef(row, col) += el;
}

virtual void Reset(int row, int col, int nonzeros = 0) {
    resize(row, col);
    if (nonzeros)
        reserve(nonzeros);
}

#endif
