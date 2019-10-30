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

/** \class ChSparsityPatternLearner
\brief A dummy matrix that acquires the sparsity pattern.

ChSparsityPatternLearner estimates the sparsity pattern without actually allocating any value, but the elements indexes.
Other matrices (like ChCSMatrix) can acquire the sparsity pattern information from this matrix.
*/
class ChApi ChSparsityPatternLearner : public Eigen::SparseMatrix<double, Eigen::RowMajor, int> {
  protected:
    std::vector<std::list<int>> rowVector_list;

  public:
    ChSparsityPatternLearner(int nrows, int ncols) : Eigen::SparseMatrix<double, Eigen::RowMajor, int>(nrows, ncols){
        rowVector_list.resize(rows());
    }

    virtual ~ChSparsityPatternLearner() {}

    void SetElement(int insrow, int inscol, double insval, bool overwrite = true) override {
        rowVector_list[insrow].push_back(inscol);
    }

    void Reset(int row, int col, int nonzeros = 0) override {
		resize(row, col);
        rowVector_list.clear();
        rowVector_list.resize(row);
    }


    std::vector<std::list<int>>& GetSparsityPattern() {
        for (auto list_iter = rowVector_list.begin(); list_iter != rowVector_list.end(); ++list_iter) {
            list_iter->sort();
            list_iter->unique();
        }
        return rowVector_list;
    }

    int GetNNZ() const override {
        int nnz_temp = 0;
        for (auto list_iter = rowVector_list.begin(); list_iter != rowVector_list.end(); ++list_iter)
            nnz_temp += static_cast<int>(list_iter->size());

        return nnz_temp;
    }
};

/// @} chrono

};  // end namespace chrono

#endif
