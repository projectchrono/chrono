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

virtual void SetElement(int row, int col, double el, bool overwrite=true){
	overwrite ? coeffRef(row, col) = el : coeffRef(row, col) += el;
}


virtual int GetNNZ() const { return nonZeros(); }

virtual void Reset(int row, int col, int nonzeros = 0){
	resize(row,col);
	if (nonzeros)
		reserve(nonzeros);
}

////inline void LoadSparsityPattern(const std::vector<std::list<int>>& rowVector_list){
////	std::vector<int> rowDimensions_list;
////	rowDimensions_list.resize(rowVector_list.size());
////	for(auto i =0; i<rowVector_list.size(); ++i){
////		rowDimensions_list.at(i) = rowVector_list.at(i).size();
////	}
////
////	reserve(rowDimensions_list);
////	for(auto row_sel = 0; row_sel<rowVector_list.size(); ++row_sel){
////		int col_el = 0;
////		for(auto it = rowVector_list.at(row_sel).begin(); it!=rowVector_list.at(row_sel).end(); ++it){
////			innerIndexPtr[col_el] = *it;
////			col_el++;
////		}
////	}
////}

#endif
