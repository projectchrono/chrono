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
#include <thrust/host_vector.h>

void mat_vec_cusparse(
      thrust::host_vector<int> & h_row,
      thrust::host_vector<int> & h_col,
      thrust::host_vector<float> & h_val,
      thrust::host_vector<float> & h_rhs,
      thrust::host_vector<float> & h_x,
      int M, int N, int NNZ);
