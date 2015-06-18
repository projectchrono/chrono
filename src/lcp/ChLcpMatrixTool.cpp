#include "ChLcpMatrixTool.h"

namespace chrono{

	// Initialization of static variables
	// It gives the initialization needed inside the library
	// Another initialization must be done in the scope of the project developed

	ChSparseMatrixBase*					ChLcpMatrixTool::output_matrix = 0;
	ChSparseMatrixSetElementPtr			ChLcpMatrixTool::MatrixFunctions::SetElementPtr = 0;
	ChSparseMatrixPasteMatrixPtr		ChLcpMatrixTool::MatrixFunctions::PasteMatrixPtr = 0;
	ChSparseMatrixPasteMatrixPtr		ChLcpMatrixTool::MatrixFunctions::PasteTranspMatrixPtr = 0;
	ChSparseMatrixPasteMatrixFloatPtr	ChLcpMatrixTool::MatrixFunctions::PasteMatrixFloatPtr = 0;
	ChSparseMatrixPasteMatrixFloatPtr	ChLcpMatrixTool::MatrixFunctions::PasteTranspMatrixFloatPtr = 0;
	//ChSparseMatrixPasteClippedMatrixPtr	ChLcpMatrixTool::MatrixFunctions::PasteClippedMatrixPtr = 0;
	//ChSparseMatrixPasteClippedMatrixPtr	ChLcpMatrixTool::MatrixFunctions::PasteSumClippedMatrixPtr = 0;
	ChSparseMatrixResetPtr				ChLcpMatrixTool::MatrixFunctions::ResetPtr = 0;
	int									ChLcpMatrixTool::m = 13;
	int									ChLcpMatrixTool::n = 0;

}