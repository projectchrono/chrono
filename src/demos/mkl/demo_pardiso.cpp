#include "demo_pardiso.h"
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* getenv */

MKL_INT main(void)
{

	MKL_INT n = 3;
	double A[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	double b[3] = { 1, 2, 3 };

	// build the CSR3 format matrix
	double *a = NULL;  //ptr to the array of CSR3 matrix
	MKL_INT *ja = NULL; //ptr to column vector of CSR3 matrix
	MKL_INT *ia = NULL; //ptr to row index vector of CSR3 matrix

	allocateCSR3((double*)A, &a, &ja, &ia, n);
	convertMatrixToCSR3((double*)A, a, ja, ia, n);

	double *f = NULL;
	double *u = NULL;
	double *fs = NULL;
	double *us = NULL;
	double res;

	/* PARDISO control parameters */
	void    *pt[64]; // Handle to internal data structure
	MKL_INT iparm[64];
	char    *uplo = "non-transposed"; // specifies if upper triangular "U" or lower triangular "L"

	for (int i = 0; i < 64; i++)
	{
		iparm[i] = 0;
	}
	iparm[0] = 1;        /* No default values for solver */
	iparm[1] = 2;        /* Fill-in reducing ordering from METIS */
	iparm[3] = 0;        /* No iterative-direct algorithm */
	iparm[4] = 0;        /* No user fill-in reducing permutation */
	iparm[5] = 0;        /* Write solution on x */
	iparm[6] = 0;        /* Not in use */
	iparm[7] = 2;        /* Maximum number of iterative refinement steps */
	iparm[8] = 0;        /* Not in use */
	iparm[9] = 13;       /* Perturb the pivot elements with 1E-13 */
	iparm[10] = 1;        /* Use nonsymmetric permutation and scaling MPS */
	iparm[11] = 0;        /* Solve with transposed/conjugate transposed matrix */
	iparm[12] = 1;        /* Maximum weighted matching algorithm is enabled (default for nonsymmetric matrices) */
	iparm[13] = 0;        /* Output: Number of perturbed pivots */
	iparm[14] = 0;        /* Not in use */
	iparm[15] = 0;        /* Not in use */
	iparm[16] = 0;        /* Not in use */
	iparm[17] = -1;       /* Output: Number of non-zero values in the factor LU */
	iparm[18] = -1;       /* Output: Mflops for LU factorization */
	iparm[19] = 0;        /* Output: Numbers of CG Iterations */
	iparm[34] = 1;        /* Zero based indexing */
	MKL_INT maxfct = 1;			  /* Maximum number of numerical factorizations. */
	MKL_INT mnum = 1;             /* Which factorization to use. */
	MKL_INT msglvl = 0;           /* Print statistical information in file */
	MKL_INT error = 0;            /* Initialize error flag */
	MKL_INT phase = -10;          /* Before PARDISO call flag */
	MKL_INT mtype = 11;         /* real nonsymmetric matrix */
	MKL_INT idum;         /* real nonsymmetric matrix */
	MKL_INT nrhs = 1;           /* number of right hand sides */



	/* Allocate memory for solution and rhs vector */
	u = (double*)mkl_malloc(sizeof(double) * n, 64); // result vector
	us = (double*)mkl_malloc(sizeof(double) * n, 64); // previous result vector
	f = (double*)mkl_malloc(sizeof(double) * n, 64); // rhs
	fs = (double*)mkl_malloc(sizeof(double) * n, 64); // previous rhs

	// Initialize vectors
	for (int i = 0; i < n; i++)
	{
		f[i] = b[i];
		us[i] = 1.0;
		u[i] = 1.0;
	}

	/* Initialize PARDISO handle */
	for (int i = 0; i < 64; i++)
	{
		pt[i] = 0;
	}

	/* Continue loop over nonlinearity until required value of residual is reached */
	res = 1.0;
	while (res > 1.e-8)
	{
		phase = 13;
		pardiso(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, f, u, &error);

		for (int i = 0; i < n; i++)
		{
			us[i] = u[i];
		}


		/* Compute residual */
		mkl_cspblas_dcsrgemv(uplo, &n, a, ia, ja, u, fs);

		res = 0;
		for (int i = 0; i < n; i++)
		{
			res += (fs[i] - f[i]) * (fs[i] - f[i]);
		}
		res = sqrt(res);

		printf("\nRelative residual = %e", res);
		printf("\nSolution = [%e; %e; %e]", u[0], u[1], u[2]);
	}

	freeCSR3(a, ja, ia);
	getchar();

}



void convertMatrixToCSR3(double* A, double* A_CSR3_values, MKL_INT* A_CSR3_columns, MKL_INT* A_CSR3_RowIndex, MKL_INT n){
	int j = 0;
	int row;
	for (row = 0; row < n; row++){
		for (int col = 0; col < n; col++){
			if (A[row*n + col] != 0){
				A_CSR3_values[j] = A[row*n + col];
				A_CSR3_columns[j] = col;
				if (A_CSR3_RowIndex[row] == -1)
					A_CSR3_RowIndex[row] = j;
				j++;
			}
		}
	}
	A_CSR3_RowIndex[row] = j;



}

void allocateCSR3(double* A, double** A_CSR3_values_ptr, MKL_INT** A_CSR3_columns_ptr, MKL_INT** A_CSR3_RowIndex_ptr, MKL_INT n){
	if ((*A_CSR3_values_ptr == NULL) && (*A_CSR3_columns_ptr == NULL) && (*A_CSR3_RowIndex_ptr == NULL)){
		// Find the right space for variables
		int nonzero_elements = 0;
		for (int row = 0; row < n; row++){
			for (int col = 0; col < n; col++){
				if (A[row*n + col] != 0){
					nonzero_elements++;
				}
			}
		}

		// Allocate variables
		*A_CSR3_values_ptr = (double*)malloc(sizeof(double)*nonzero_elements);
		*A_CSR3_columns_ptr = (MKL_INT*)malloc(sizeof(MKL_INT)*nonzero_elements);
		*A_CSR3_RowIndex_ptr = (MKL_INT*)malloc(sizeof(MKL_INT)*(n + 1));

		//Zeroing variables (could be avoided if use calloc)
		for (int i = 0; i < nonzero_elements; i++){
			(*A_CSR3_values_ptr)[i] = 0;
			(*A_CSR3_columns_ptr)[i] = 0;
		}

		for (int i = 0; i <= n; i++){
			(*A_CSR3_RowIndex_ptr)[i] = -1;
		}
	}
}

void freeCSR3(double* A_CSR3_values, MKL_INT* A_CSR3_columns, MKL_INT* A_CSR3_RowIndex){
	free(A_CSR3_RowIndex);
	free(A_CSR3_values);
	free(A_CSR3_columns);

}

//void convertCSR3ToMatrix(double* A, double* A_CSR3_values, MKL_INT* A_CSR3_columns, double** A_CSR3_RowIndex){
//
//
//}