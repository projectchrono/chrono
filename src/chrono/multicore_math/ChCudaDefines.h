// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Macros for using mutlicore_math with CUDA
//
// =============================================================================

#pragma once

#ifdef __CUDACC__
    #define CUDA_HOST_DEVICE __host__ __device__
    #define CUDA_DEVICE __device__
    #define CUDA_CONSTANT __device__ __constant__
    #define CUDA_SHARED __shared__
    #define CUDA_GLOBAL __global__
#else
    #define CUDA_HOST_DEVICE
    #define CUDA_DEVICE
    #define CUDA_CONSTANT
    #define CUDA_SHARED
    #define CUDA_GLOBAL
#endif
