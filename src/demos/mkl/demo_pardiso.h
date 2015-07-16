#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "memory.h"

#include "mkl_pardiso.h"
#include "mkl_types.h"
#include "mkl_spblas.h"
#include "mkl_service.h"

//void construct_matrix_structure(MKL_INT nx, MKL_INT ny, MKL_INT nz, MKL_INT** ia_ptr, MKL_INT** ja_ptr, double** a_ptr, MKL_INT *error);
//void construct_matrix_values(MKL_INT nx, MKL_INT ny, MKL_INT nz, double*u, MKL_INT* ia, MKL_INT* ja, double* a_ptr, MKL_INT *error);
//MKL_INT finalize(void *pt, MKL_INT maxfct, MKL_INT mnum, MKL_INT mtype, MKL_INT phase, MKL_INT n,
//				MKL_INT nrhs, MKL_INT *iparm, MKL_INT msglvl,
//				double *u, double *us, double *f, double *fs,
//				double *a, MKL_INT *ia, MKL_INT *ja,
//				MKL_INT error);

void convertMatrixToCSR3(double* A, double* A_CSR3_values, MKL_INT* A_CSR3_columns, MKL_INT* A_CSR3_RowIndex, MKL_INT n);
void allocateCSR3(double* A, double** A_CSR3_values_ptr, MKL_INT** A_CSR3_columns_ptr, MKL_INT** A_CSR3_RowIndex_ptr, MKL_INT n);
void freeCSR3(double* A_CSR3_values, MKL_INT* A_CSR3_columns, MKL_INT* A_CSR3_RowIndex);