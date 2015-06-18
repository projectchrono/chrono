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
	typedef void (ChSparseMatrixBase::* ChSparseMatrixPasteMatrixFloatPtr)(ChMatrix<float>*, int, int); // type for PasteMatrixFloat and transposed version
	typedef void (ChSparseMatrixBase::* ChSparseMatrixPasteClippedMatrixPtr)(ChMatrix<>*, int, int, int, int, int, int ); // type for PasteClippedMatrix and PasteSumClippedMatrix
	typedef void (ChSparseMatrixBase::* ChSparseMatrixResetPtr)(int, int); // type for Reset

	class ChLcpMatrixTool{
		//
		// POINTERs TO MATRIX MEMBER FUNCTIONS
		//
	public: // it should be protected and made friend of LcpVariables, LcpConstraint, LcpKblock?, LcpSystemDescriptor?

		static int n; // overall matrix dimension
		static int m; // stiffness/mass matrix dimension
		static ChSparseMatrixBase* output_matrix;
		static struct MatrixFunctions {
			static ChSparseMatrixSetElementPtr			SetElementPtr;
			static ChSparseMatrixPasteMatrixPtr			PasteMatrixPtr;
			static ChSparseMatrixPasteMatrixPtr			PasteTranspMatrixPtr;
			static ChSparseMatrixPasteMatrixFloatPtr	PasteMatrixFloatPtr; // is this function replaceable?
			static ChSparseMatrixPasteMatrixFloatPtr	PasteTranspMatrixFloatPtr;
			//static ChSparseMatrixPasteClippedMatrixPtr	PasteClippedMatrixPtr;
			//static ChSparseMatrixPasteClippedMatrixPtr	PasteSumClippedMatrixPtr;
			static ChSparseMatrixResetPtr				ResetPtr;
		};

		template <class SparseMatrixDerivedType>
		static void SetMatrixTools(SparseMatrixDerivedType* dest_matrix){
			output_matrix = (ChSparseMatrixBase*)dest_matrix; // explicit just to be clear
			MatrixFunctions::SetElementPtr				= (ChSparseMatrixSetElementPtr)			&SparseMatrixDerivedType::SetElement;
			MatrixFunctions::PasteMatrixPtr				= (ChSparseMatrixPasteMatrixPtr)		&SparseMatrixDerivedType::PasteMatrix;
			MatrixFunctions::PasteTranspMatrixPtr		= (ChSparseMatrixPasteMatrixPtr)		&SparseMatrixDerivedType::PasteTranspMatrix;
			MatrixFunctions::PasteMatrixFloatPtr		= (ChSparseMatrixPasteMatrixFloatPtr)	&SparseMatrixDerivedType::PasteMatrixFloat;
			MatrixFunctions::PasteTranspMatrixFloatPtr	= (ChSparseMatrixPasteMatrixFloatPtr)	&SparseMatrixDerivedType::PasteTranspMatrixFloat;
			//MatrixFunctions::PasteClippedMatrixPtr		= (ChSparseMatrixPasteClippedMatrixPtr)	&SparseMatrixDerivedType::PasteClippedMatrix;
			//MatrixFunctions::PasteSumClippedMatrixPtr	= (ChSparseMatrixPasteClippedMatrixPtr)	&SparseMatrixDerivedType::PasteSumClippedMatrix;
			MatrixFunctions::ResetPtr					= (ChSparseMatrixResetPtr)				&SparseMatrixDerivedType::Reset;
			m = 3;
		}


	}; // END class ChLcpMatrixTool

} // END namespace chrono

#endif