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
        rowVector_list.resize(rows());
    }

    ~ChSparsityPatternLearner() {}

    virtual void SetElement(int insrow, int inscol, double insval, bool overwrite = true) override {
        rowVector_list[insrow].push_back(inscol);
    }


    void Apply(ChSparseMatrix& mat) {
        if (!processed)
			process();

        mat.resize(rows(), cols());
        mat.reserve(rowDimensions_list);
		//int rowIndexCurrent = 0;
		int col_el = 0;
        for (auto row_sel = 0; row_sel < rowVector_list.size(); ++row_sel) {

			//mat.outerIndexPtr()[row_sel] = rowIndexCurrent;

            for (auto it = rowVector_list[row_sel].begin(); it != rowVector_list[row_sel].end(); ++it) {
                mat.innerIndexPtr()[col_el] = *it;
                col_el++;
            }

			//rowIndexCurrent += rowDimensions_list[row_sel];
        }
		//mat.outerIndexPtr()[rows()] = rowIndexCurrent;
    }

  private:
    void process() {

		for (auto list_iter = rowVector_list.begin(); list_iter != rowVector_list.end(); ++list_iter) {
            list_iter->sort();
            list_iter->unique();
        }

		rowDimensions_list.resize(rowVector_list.size());
        for (auto i = 0; i < rowVector_list.size(); ++i) {
            rowDimensions_list[i] = static_cast<int>(rowVector_list[i].size());
        }

		processed = true;
    }

    std::vector<std::list<int>> rowVector_list;
    std::vector<int> rowDimensions_list;
    bool processed;
};

/// @} chrono_linalg

};  // end namespace chrono

#endif
