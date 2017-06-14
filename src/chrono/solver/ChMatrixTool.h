// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================


#ifndef CHLCPMATRIXTOOL_H
#define CHLCPMATRIXTOOL_H

#include "core/ChSpmatrix.h"

namespace chrono{

	typedef void (ChSparseMatrixBase::* ChSparseMatrixSetElementPtr)(int, int, double); // type for SetElement
	typedef void (ChSparseMatrixBase::* ChSparseMatrixPasteMatrixPtr)(ChMatrix<>*, int, int); // type for PasteMatrix
	typedef void (ChSparseMatrixBase::* ChSparseMatrixPasteMatrixFloatPtr)(ChMatrix<float>*, int, int); // type for PasteMatrixFloat and transposed version
	typedef void (ChSparseMatrixBase::* ChSparseMatrixPasteClippedMatrixPtr)(ChMatrix<>*, int, int, int, int, int, int ); // type for PasteClippedMatrix and PasteSumClippedMatrix
	typedef void (ChSparseMatrixBase::* ChSparseMatrixResetPtr)(int, int); // type for Reset

	class ChApi ChLcpMatrixTool{
	public:
		ChSparseMatrixBase* output_matrix;
		struct {
			ChSparseMatrixSetElementPtr				SetElementPtr;
			ChSparseMatrixPasteMatrixPtr			PasteMatrixPtr;
			ChSparseMatrixPasteMatrixPtr			PasteTranspMatrixPtr;
			ChSparseMatrixPasteMatrixFloatPtr		PasteMatrixFloatPtr; // is this function replaceable?
			ChSparseMatrixPasteMatrixFloatPtr		PasteTranspMatrixFloatPtr;
			ChSparseMatrixPasteClippedMatrixPtr		PasteClippedMatrixPtr;
			ChSparseMatrixPasteClippedMatrixPtr		PasteSumClippedMatrixPtr;
			ChSparseMatrixResetPtr					ResetPtr;
		} MatrixFunctions;


		/// Sets the Pointers-To-Member-Function based on destination matrix type.
		template <class SparseMatrixDerivedType>
		void SetMatrixTools(SparseMatrixDerivedType* dest_matrix){
			output_matrix = static_cast<ChSparseMatrixBase*>(dest_matrix);
			MatrixFunctions.SetElementPtr				= static_cast<ChSparseMatrixSetElementPtr>(&SparseMatrixDerivedType::SetElement);
			MatrixFunctions.PasteMatrixPtr				= static_cast<ChSparseMatrixPasteMatrixPtr>(&SparseMatrixDerivedType::PasteMatrix);
			MatrixFunctions.PasteTranspMatrixPtr		= static_cast<ChSparseMatrixPasteMatrixPtr>(&SparseMatrixDerivedType::PasteTranspMatrix);
			MatrixFunctions.PasteMatrixFloatPtr			= static_cast<ChSparseMatrixPasteMatrixFloatPtr>(&SparseMatrixDerivedType::PasteMatrixFloat);
			MatrixFunctions.PasteTranspMatrixFloatPtr	= static_cast<ChSparseMatrixPasteMatrixFloatPtr>(&SparseMatrixDerivedType::PasteTranspMatrixFloat);
			MatrixFunctions.PasteClippedMatrixPtr		= static_cast<ChSparseMatrixPasteClippedMatrixPtr>(&SparseMatrixDerivedType::PasteClippedMatrix);
			MatrixFunctions.PasteSumClippedMatrixPtr	= static_cast<ChSparseMatrixPasteClippedMatrixPtr>(&SparseMatrixDerivedType::PasteSumClippedMatrix);
			MatrixFunctions.ResetPtr					= static_cast<ChSparseMatrixResetPtr>(&SparseMatrixDerivedType::Reset);
		}


	}; // END class ChLcpMatrixTool

} // END namespace chrono

#endif