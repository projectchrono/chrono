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
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test for MPR collision detection
// =============================================================================

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "test_matvec.h"
#include <cusparse.h>
#include <thrust/device_vector.h>
#define THRUSTCASTI(x) (int*)thrust::raw_pointer_cast(&x[0])
#define THRUSTCASTF(x) (real*)thrust::raw_pointer_cast(&x[0])

void mat_vec_cusparse(thrust::host_vector<int> & h_row,
                      thrust::host_vector<int> & h_col,
                      thrust::host_vector<real> & h_val,
                      thrust::host_vector<real> & h_rhs,
                      thrust::host_vector<real> & h_x,
                      int M,
                      int N,
                      int NNZ) {

   thrust::device_vector<int> d_row = h_row;
   thrust::device_vector<int> d_col = h_col;
   thrust::device_vector<real> d_val = h_val;
   thrust::device_vector<real> d_rhs = h_rhs;
   thrust::device_vector<real> d_x = h_x;

   real alpha = 1.0;
   real alpham1 = -1.0;
   real beta = 0.0;
   real r0 = 0.;

   cusparseHandle_t cusparseHandle = 0;
   cusparseStatus_t cusparseStatus;
   cusparseStatus = cusparseCreate(&cusparseHandle);

   cusparseMatDescr_t descr = 0;
   cusparseStatus = cusparseCreateMatDescr(&descr);

   cusparseSetMatType(descr, CUSPARSE_MATRIX_TYPE_GENERAL);
   cusparseSetMatIndexBase(descr, CUSPARSE_INDEX_BASE_ZERO);

   cudaEvent_t start, stop;
   float time;
   cudaEventCreate(&start);
   cudaEventCreate(&stop);
   cusparseStatus_t status;
   thrust::host_vector<int> h_csr(M+1,0);

   thrust::device_vector<int> d_csr = h_csr;
   cusparseXcoo2csr(cusparseHandle, THRUSTCASTI(d_row), NNZ, M, THRUSTCASTI(d_csr),CUSPARSE_INDEX_BASE_ZERO);


   
   cudaEventRecord(start, 0);
   for (int i = 0; i < 100; i++) {
      cusparseScsrmv(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, M, N, NNZ, &alpha, descr, THRUSTCASTF(d_val), THRUSTCASTI(d_csr), THRUSTCASTI(d_col),
                     THRUSTCASTF(d_rhs), &beta, THRUSTCASTF(d_x));
   }
   cudaEventRecord(stop, 0);
   cudaEventSynchronize(stop);
   cudaEventElapsedTime(&time, start, stop);
   
   h_x = d_x;
   
   std::cout << time/1000.0 << std::endl;
}
