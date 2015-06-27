#include <iostream>
#include <cstdlib>
#include <VSPerf.h>
#include <lcp/ChLcpSystemDescriptor.h>
#include <unit_MKL/ChCSR3matrix.h>
#include <lcp/ChLcpMatrixTool.h>

using namespace std;
using namespace chrono;


int main(){
	// Initialize matrix (CSR3) and its pointer
	const unsigned int n = 750;
	std::vector<int> resizeSize(n);
	for (unsigned int i = 1; i < n; i++){
		resizeSize[i] = n;
	}
	ChEigenMatrix Zcsr(n, n, resizeSize);
	
	// Set Pointer to Member
	ChLcpMatrixTool MatToolCSR;
	MatToolCSR.SetMatrixTools(&Zcsr);

	// Set Pointer to Base
	ChSparseMatrixBase* Zcsr_ptr = &Zcsr;


	StartProfile(PROFILE_GLOBALLEVEL, PROFILE_CURRENTID);

	for (int cont = 0; cont < 1; cont++){

		CommentMarkProfile(75000, "START PtrToMember");
		for (auto i = 0; i < n; i++)
			for (auto j = 0; j < n; j++)
			{
				(MatToolCSR.output_matrix->*MatToolCSR.MatrixFunctions.SetElementPtr)(i, j, i + j);
			}
		CommentMarkProfile(75001, "STOP PtrToMember");

		CommentMarkProfile(75002, "START VirtualCall");
		for (auto i = 0; i < n; i++)
			for (auto j = 0; j < n; j++)
			{
				Zcsr_ptr->SetElement(i, j, i + j);
			}
		CommentMarkProfile(75003, "STOP VirtualCall");

		//cout << cont << endl;
	}

	StopProfile(PROFILE_GLOBALLEVEL, PROFILE_CURRENTID);

	//////////////////////////////////////////////////////////////////

	//// Initialize matrix (ChSparseMatrix)
	//ChSparseMatrix ZLL(n, n, 1);
	//ChSparseMatrixBase* ZLL_ptr = &ZLL;

	//// Set Pointer to Member
	//ChLcpMatrixTool MatToolLL;
	//MatToolLL.SetMatrixTools(&ZLL);

}