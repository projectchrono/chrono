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

#ifndef CHCOOMATRIX_H
#define CHCOOMATRIX_H

#include <vector>

#include "chrono/core/ChCSMatrix.h"

namespace chrono {

class ChApi ChCOOMatrix : public ChCSMatrix {
  public:
    explicit ChCOOMatrix(int mat_rows = 3, int mat_cols = 3, bool one_indexed = true);

    virtual ~ChCOOMatrix() {}

    void SetOneIndexing(bool val) { one_indexed = val; }

    /// Return the row index array in the COO representation of this matrix.
    int* GetCOO_RowIndexArray() const;

    /// Return the column index array in the COO representation of this matrix.
    int* GetCOO_ColIndexArray() const;

    /// Return the array of matrix values in the CSR representation of this matrix.
    double* GetCOO_ValuesAddress() const;

  private:
    std::vector<int> rowIndex;
    std::vector<int> colIndex;
    std::vector<double> values;
    bool one_indexed = true;
};

}  // namespace chrono

#endif
