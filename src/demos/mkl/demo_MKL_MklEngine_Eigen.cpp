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

#include "chrono/core/ChLog.h"
#include "chrono/core/ChCSMatrix_Eigen.h"
#include "chrono_mkl/ChSolverMKL_Eigen.h"

using namespace chrono;

void test_CSR3()
{
	std::cout << "//////////// CSR3 Matrix: basic functions testing //////////////" << std::endl;
	int m = 3;
	int n = 5;
	ChCSMatrix_Eigen matCSR3_1(m, n);
	int prova = 0;
	MKL_INT64 mem = mkl_mem_stat(&prova);

	for (int i = 0; i < m; i++)
	{
		for (int j = i; j < n; j++)
		{
			matCSR3_1.coeffRef(i, j) = 2.5;
		}
	}

	mem = mkl_mem_stat(&prova);

	matCSR3_1.coeffRef(0, 1) = 0.7;

	double elem = matCSR3_1.coeffRef(2, 0);
	matCSR3_1.coeffRef(2, 0) = matCSR3_1.coeffRef(0, 1);
	//double* elem = &(matCSR3_1.coeffRef(2, 0));

	matCSR3_1.coeffRef(2, 2) = 5;

	matCSR3_1.makeCompressed();

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			printf("%f ", matCSR3_1.coeff(i, j));
		}
		printf("\n");
	}

}

void test_MklEngine()
{
	std::cout << std::endl << "//////////// Simple ChMklEngine call //////////////" << std::endl;
	std::cout << "This example should give the same results as examples_core_c/solverc/pardiso_unsym_c.c";

	// This demo should reply the same result of sample "pardiso_unsym_c.c" in "examples_core_c/solverc"
	// those should be the values of the 3 arrays of matCSR3
	// int ia[6] = {       0,                3,          5,                8,               11,           13 };
	// int ja[13] ={       0,    1,    3,    0,    1,    2,    3,    4,    0,    2,    3,    1,    4  };
	// double a[13] =  {  1.0, -1.0, -3.0, -2.0,  5.0,  4.0,  6.0,  4.0, -4.0,  2.0,  7.0,  8.0, -5.0 };


	int n = 5;
	ChCSMatrix_Eigen matCSR3(n, n);
	ChVectorDynamic<double> rhs(n);
	ChVectorDynamic<double> sol(n);

	matCSR3.coeffRef(0, 0) = 1.0;
	matCSR3.coeffRef(0, 1) = -1.0;
	matCSR3.coeffRef(0, 3) = -3.0;
	matCSR3.coeffRef(1, 0) = -2.0;
	matCSR3.coeffRef(1, 1) = 5.0;
	matCSR3.coeffRef(2, 2) = 4.0;
	matCSR3.coeffRef(2, 3) = 6.0;
	matCSR3.coeffRef(2, 4) = 4.0;
	matCSR3.coeffRef(3, 0) = -4.0;
	matCSR3.coeffRef(3, 2) = 2.0;
	matCSR3.coeffRef(3, 3) = 7.0;
	matCSR3.coeffRef(4, 1) = 8.0;
	matCSR3.coeffRef(4, 4) = -5.0;

	for (int cont = 0; cont < n; cont++)
		rhs(cont) = 1.0;


	// Solve with Pardiso Sparse Direct Solver
	ChMklEngine_Eigen pardiso_solver(n, ChSparseMatrix::GENERAL);
	matCSR3.makeCompressed();
	pardiso_solver.SetProblem(matCSR3, rhs, sol);

	int pardiso_message = pardiso_solver.PardisoCall(13, 0);
	printf("\nPardiso exited with code: %d\n", pardiso_message);

	// Print statistics
	double res_norm = pardiso_solver.GetResidualNorm();
	GetLog() << "\nResidual Norm: " << res_norm << "\n\n";
}

int main(){
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	test_CSR3();

	test_MklEngine();
	
}

