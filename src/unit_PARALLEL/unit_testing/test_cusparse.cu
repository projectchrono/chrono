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
#include <vector>
#include <cmath>
#include "test_matvec.h"
#include <cusparse.h>
#include <thrust/device_vector.h>
#define THRUSTCASTI(x) (int*)thrust::raw_pointer_cast(&x[0])
#define THRUSTCASTF(x) (float*)thrust::raw_pointer_cast(&x[0])

void mat_vec_cusparse(thrust::host_vector<int> & h_row,
                      thrust::host_vector<int> & h_col,
                      thrust::host_vector<float> & h_val,
                      thrust::host_vector<float> & h_rhs,
                      thrust::host_vector<float> & h_x, 
                      int M, int N, int NNZ) {

   thrust::device_vector<int> d_row = h_row;
   thrust::device_vector<int> d_col = h_col;
   thrust::device_vector<float> d_val = h_val;
   thrust::device_vector<float> d_rhs = h_rhs;
   thrust::device_vector<float> d_x = h_x;

   float alpha = 1.0;
   float alpham1 = -1.0;
   float beta = 0.0;
   float r0 = 0.;

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
   
   
   cudaEventRecord(start, 0);
   cusparseScsrmv(cusparseHandle, CUSPARSE_OPERATION_NON_TRANSPOSE, M, N, NNZ, &alpha, descr, THRUSTCASTF(d_val.data()), THRUSTCASTI(d_row.data()), THRUSTCASTI(d_col.data()),
                  THRUSTCASTF(d_rhs.data()), &beta, THRUSTCASTF(d_x.data()));
   cudaEventRecord(stop, 0);
   cudaEventSynchronize(stop);
   cudaEventElapsedTime(&time, start, stop);
   printf ("Time for the kernel: %f s\n", time/1000.0);
}
