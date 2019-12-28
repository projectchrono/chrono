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

#ifndef CHSPARSITYPATTERNLEARNER_H
#define CHSPARSITYPATTERNLEARNER_H

#include "chrono/core/ChMatrix.h"

namespace chrono {

/// @addtogroup chrono_linalg
/// @{

/// Utility class for extracting sparsity patter from a sparse matrix.
/// Derived from ChSparseMatrix, ChSparsityPatternLearner does not allocate values, but only element indices.
/// The sparsity pattern can then be applied to a given sparse matrix.
class ChSparsityPatternLearner : public Eigen::SparseMatrix<double, Eigen::RowMajor, int> {
  public:
    ChSparsityPatternLearner(int nrows, int ncols) : ChSparseMatrix(nrows, ncols), processed(false) {
        // RowMajor: outerSize == nrows
        // ColMajor: outerSize == ncols
        innerVectors.resize(outerSize());
    }

    ~ChSparsityPatternLearner() {}

    virtual void SetElement(int row, int col, double val, bool overwrite = true) override {
        const Index outer = IsRowMajor ? row : col;
        const Index inner = IsRowMajor ? col : row;
        innerVectors[outer].push_back((int)inner);
    }

    void Apply(ChSparseMatrix& mat) {
        if (!processed)
            process();

        // Resize the matrix and reserve space for non-zero elements in each inner vector
        mat.resize(rows(), cols());
        mat.reserve(innerVectors_size);

        int col_el = 0;
        for (auto outer = 0; outer < innerVectors.size(); ++outer) {
            for (auto inner = innerVectors[outer].begin(); inner != innerVectors[outer].end(); ++inner) {
                mat.innerIndexPtr()[col_el] = *inner;
                col_el++;
            }
        }
    }

  private:
    void process() {
        // Find the unique indices of non-zero elements in each inner vector
        for (auto vec = innerVectors.begin(); vec != innerVectors.end(); ++vec) {
            vec->sort();
            vec->unique();
        }

        // Cache the number of non-zero elements in each inner vector
        // (in each row if RowMajor, in each column if ColMajor)
        innerVectors_size.resize(innerVectors.size());
        for (auto i = 0; i < innerVectors.size(); ++i) {
            innerVectors_size[i] = static_cast<int>(innerVectors[i].size());
        }

        processed = true;
    }

    // RowMajor:  innerVectors[i] contains the column indices of non-zero elements in row i
    // ColMajor:  innerVectors[i] contains the row indices of non-zero elements in column i
    std::vector<std::list<int>> innerVectors;

    // RowMajor: innerVectors_size[i] contains the number of non-zero elements in row i
    // ColMajor: innerVectors_size[i] contains the number of non-zero elements in column i
    std::vector<int> innerVectors_size;

    bool processed;
};

/// @} chrono_linalg

};  // end namespace chrono

#endif
