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

/// Utility class for extracting sparsity patter from a sparse matrix.
/// Derived from ChSparseMatrix, ChSparsityPatternLearner does not allocate values, but only element indices.
/// The sparsity pattern can then be applied to a given sparse matrix.
class ChSparsityPatternLearner : public Eigen::SparseMatrix<double, Eigen::RowMajor, int> {
  public:
    ChSparsityPatternLearner(int nrows, int ncols) : ChSparseMatrix(nrows, ncols), processed(false) {
        rowVector_list.resize(rows());
    }

    ~ChSparsityPatternLearner() {}

    virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true) override {
        rowVector_list[insrow].push_back(inscol);
    }

    //// TODO: Is this needed at all?
    ////       If not, eleminate Reset from ChSparseMatrixEigenExtensions
    virtual void Reset(int row, int col, int nonzeros = 0) override {
		resize(row, col);
        rowVector_list.clear();
        rowVector_list.resize(row);
    }

    //// TODO:  Complete Process() and Apply()

    void Apply(ChSparseMatrix& mat) {
        if (!processed) {
            Process();
            processed = true;
        }

        mat.resize(rows(), cols());
        mat.reserve(rowDimensions_list);
        for (auto row_sel = 0; row_sel < rowVector_list.size(); ++row_sel) {
            int col_el = 0;
            for (auto it = rowVector_list.at(row_sel).begin(); it != rowVector_list.at(row_sel).end(); ++it) {
                mat.innerIndexPtr()[col_el] = *it;
                col_el++;
            }
        }
    }

  private:
    void Process() {
        rowDimensions_list.resize(rowVector_list.size());
        for (auto i = 0; i < rowVector_list.size(); ++i) {
            rowDimensions_list.at(i) = (int)rowVector_list.at(i).size();
        }
    }

    std::vector<std::list<int>> rowVector_list;
    std::vector<int> rowDimensions_list;
    bool processed;
};

/// @} chrono

};  // end namespace chrono

#endif
