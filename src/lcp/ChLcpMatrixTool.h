//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLCPMATRIXTOOL_H
#define CHLCPMATRIXTOOL_H

#include "core/ChSpmatrix.h"

namespace chrono{

	typedef void (ChSparseMatrixBase::* ChSparseMatrixSetElementPtr)(int, int, double); // type for SetElement
	typedef void (ChSparseMatrixBase::* ChSparseMatrixPasteMatrixPtr)(ChMatrix<>*, int, int); // type for PasteMatrix

	class ChApi ChLcpMatrixTool{
		//
		// POINTERs TO MATRIX MEMBER FUNCTIONS
		//
	public:
		static int prova; // dummy variable used in testing
		static ChSparseMatrixBase* output_matrix;
		static struct MatrixFunctions {
			static ChSparseMatrixSetElementPtr SetElementPtr;
			static ChSparseMatrixPasteMatrixPtr PasteMatrixPtr;
		}; // addresses of various methods of the (derived) matrix

		template <class SparseMatrixType>
		void SetMatrixTools(SparseMatrixType* dest_matrix){
			output_matrix = (ChSparseMatrixBase*)dest_matrix; // explicit just to be clear
			MatrixFunctions::SetElementPtr = (ChSparseMatrixSetElementPtr)&SparseMatrixType::SetElement;
			MatrixFunctions::ChSparseMatrixPasteMatrixPtr = (ChSparseMatrixSetElementPtr)&SparseMatrixType::PasteMatrix;
		}
	}; // END class ChLcpMatrixTool
} // END namespace chrono

#endif