#include "ChLcpMatrixTool.h"

namespace chrono{
	// Initialization of static variables
	ChSparseMatrixBase*					ChLcpMatrixTool::output_matrix = 0;
	ChSparseMatrixSetElementPtr			ChLcpMatrixTool::MatrixFunctions::SetElementPtr = 0;
	ChSparseMatrixPasteMatrixPtr		ChLcpMatrixTool::MatrixFunctions::PasteMatrixPtr = 0;
	ChSparseMatrixPasteMatrixFloatPtr	ChLcpMatrixTool::MatrixFunctions::PasteMatrixFloatPtr = 0;
	int									ChLcpMatrixTool::prova = 6;
	int									ChLcpMatrixTool::m = 0;
	int									ChLcpMatrixTool::n = 0;
}