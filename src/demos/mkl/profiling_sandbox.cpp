#include <lcp/ChLcpSystemDescriptor.h>
#include <iostream>
#include <cstdlib>
#include <VSPerf.h>
#include <unit_MKL/ChCSR3matrix.h>
#include <lcp/ChLcpMatrixTool.h>

using namespace std;
using namespace chrono;


int main(){
	// Initialize matrix (CSR3) and its pointer
	const unsigned int n = 10;
	std::vector<int> resizeSize(n);
	for (unsigned int i = 1; i < n; i++){
		resizeSize[i] = n;
	}
	ChEigenMatrix Zcsr(n, n, resizeSize);
	ChSparseMatrixBase* Zcsr_ptr = &Zcsr;
	
	// Set Pointer to Member
	ChLcpMatrixTool MatToolCSR;
	MatToolCSR.SetMatrixTools(&Zcsr);


	// Different calls...
	cout << "Eigen Matrix" << endl;
	(MatToolCSR.output_matrix->*MatToolCSR.MatrixFunctions.SetElementPtr)(1, 1, 3.4);
	cout << "PtrMember: " << Zcsr.GetElement(1, 1) << endl;
	//vZcsr_ptr->SetElement(1, 1, 7.9);
	//cout << "Base->Member " << Zcsr.GetElement(1, 1) << endl;

	//cout << "Zcsr_ptr->GetElement(1, 1): " << Zcsr_ptr->GetElement(1, 1) << endl;

	cout << endl;

	//////////////////////////////////////////////////////////////////

	// Initialize matrix (ChSparseMatrix)
	ChSparseMatrix ZLL(n, n, 1);
	ChSparseMatrixBase* ZLL_ptr = &ZLL;

	// Set Pointer to Member
	ChLcpMatrixTool MatToolLL;
	MatToolLL.SetMatrixTools(&ZLL);


	// Different calls...
	cout << "LinkedList Matrix" << endl;
	(MatToolLL.output_matrix->*MatToolLL.MatrixFunctions.SetElementPtr)(1, 1, 9.5);
	cout << "PtrMember: " << ZLL.GetElement(1, 1) << endl;
	//ZLL_ptr->SetElement(1, 1, 1.6);
	//cout << "Base->Member: " << ZLL.GetElement(1, 1) << endl;

	//cout << "ZLL_ptr->GetElement(1, 1): " << ZLL_ptr->GetElement(1, 1) << endl;

}