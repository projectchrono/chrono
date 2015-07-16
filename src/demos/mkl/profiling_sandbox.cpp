#include <iostream>
#include <cstdlib>
#include <VSPerf.h>
#include <lcp/ChLcpSystemDescriptor.h>
#include <unit_MKL/ChCSR3matrix.h>
#include <lcp/ChLcpMatrixTool.h>

using namespace std;
using namespace chrono;


int main(){
	const unsigned int n = 10;

	// Initialize CSR matrix with templatized functions
	std::vector<int> resizeSize(n);
	for (unsigned int i = 1; i < n; i++){
		resizeSize[i] = n;
	}
	ChEigenMatrix Zcsr(n, n, resizeSize);

	// Initialize CSR matrix with virtual functions
	ChEigenMatrixVRT ZcsrVRT(n, n, resizeSize);


	// Set Pointer to Member
	ChLcpMatrixTool MatToolCSR;
	MatToolCSR.SetMatrixTools(&Zcsr);


	// Set Pointer to Base
	ChSparseMatrixBase* ZcsrVRT_ptr = &ZcsrVRT;

	//StartProfile(PROFILE_GLOBALLEVEL, PROFILE_CURRENTID);

	for (int cont = 0; cont < 100; cont++){

		//CommentMarkProfile(75002, "START VirtualCall");
		for (auto i = 0; i < n; i++)
			for (auto j = 0; j < n; j++)
			{
				ZcsrVRT_ptr->SetElement(i, j, i + j);
			}
		//CommentMarkProfile(75003, "STOP VirtualCall");

		//CommentMarkProfile(75000, "START PtrToMember");
		for (auto i = 0; i < n; i++)
			for (auto j = 0; j < n; j++)
			{
				(MatToolCSR.output_matrix->*MatToolCSR.MatrixFunctions.SetElementPtr)(i, j, i + j);
			}
		//CommentMarkProfile(75001, "STOP PtrToMember");

	}

	//StopProfile(PROFILE_GLOBALLEVEL, PROFILE_CURRENTID);

	//////////////////////////////////////////////////////////////////

	//// Initialize matrix (ChSparseMatrix)
	//ChSparseMatrix ZLL(n, n, 1);
	//ChSparseMatrixBase* ZLL_ptr = &ZLL;

	//// Set Pointer to Member
	//ChLcpMatrixTool MatToolLL;
	//MatToolLL.SetMatrixTools(&ZLL);

}