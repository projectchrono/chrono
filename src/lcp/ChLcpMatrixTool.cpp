#include "ChLcpMatrixTool.h"

namespace chrono{
	// Initialization of static variables
	ChSparseMatrixBase* output_matrix = 0;
	ChSparseMatrixSetElementPtr ChLcpMatrixTool::MatrixFunctions::SetElementPtr = 0;
	ChSparseMatrixPasteMatrixPtr ChLcpMatrixTool::MatrixFunctions::PasteMatrixPtr = 0;
	int ChLcpMatrixTool::prova = 6;
}