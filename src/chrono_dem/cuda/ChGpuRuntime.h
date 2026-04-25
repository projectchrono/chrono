// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Florian Reinle
// =============================================================================
// Use: Adaption for HIP/CUDA dual-backend support
// =============================================================================

#pragma once

#include "chrono_dem/cuda/ChGpuPrimitives.h"

#include <cstddef>
#include <cstdlib>

#if defined(__CUDACC__)
  #include <cuda_runtime.h>
  #include <cuda_runtime_api.h>
  // CUB is not used by this compatibility header. Keep this include optional so
  // normal CUDA builds do not require CUB to be on the host compiler include path.
  #if __has_include(<cub/cub.cuh>)
    #include <cub/cub.cuh>
  #endif
  #define CHGPU_DEVICE_ABORT() cuda::std::terminate()
#elif defined(CHRONO_USE_CUDA)
  // Host-side CUDA build: .cpp translation units need CUDA runtime declarations
  // for cudaMallocManaged/cudaFree and CUDA vector types, but must not pull CUB.
  #include <cuda_runtime.h>
  #include <cuda_runtime_api.h>
  #define CHGPU_DEVICE_ABORT() std::abort()
#elif defined(__HIPCC__) || defined(__HIP_DEVICE_COMPILE__)
  #ifndef __HIP_PLATFORM_AMD__
    #define __HIP_PLATFORM_AMD__
  #endif
  #include <hip/hip_runtime.h>
  #include <hip/hip_runtime_api.h>
  #if __has_include(<hip/math_constants.h>)
    #include <hip/math_constants.h>
  #elif __has_include(<math_constants.h>)
    #include <math_constants.h>
  #endif
  using cudaError_t = hipError_t;
  using cudaMemcpyKind = hipMemcpyKind;
  using cudaMemoryAdvise = hipMemoryAdvise;
  static constexpr cudaError_t cudaSuccess = hipSuccess;
  static constexpr cudaError_t cudaErrorMemoryAllocation = hipErrorMemoryAllocation;
  static constexpr cudaError_t cudaErrorNotSupported = hipErrorNotSupported;
  static constexpr unsigned int cudaMemAttachGlobal = hipMemAttachGlobal;
  static constexpr cudaMemcpyKind cudaMemcpyHostToDevice = hipMemcpyHostToDevice;
  static constexpr cudaMemcpyKind cudaMemcpyDeviceToHost = hipMemcpyDeviceToHost;
  static constexpr cudaMemcpyKind cudaMemcpyDeviceToDevice = hipMemcpyDeviceToDevice;
  static constexpr cudaMemoryAdvise cudaMemAdviseSetReadMostly = hipMemAdviseSetReadMostly;
  inline const char* cudaGetErrorString(cudaError_t err) { return hipGetErrorString(err); }
  inline cudaError_t cudaPeekAtLastError() { return hipPeekAtLastError(); }
  inline cudaError_t cudaMalloc(void** ptr, std::size_t bytes) { return hipMalloc(ptr, bytes); }
  template <class T> inline cudaError_t cudaMalloc(T** ptr, std::size_t bytes) { return hipMalloc(reinterpret_cast<void**>(ptr), bytes); }
  inline cudaError_t cudaMallocManaged(void** ptr, std::size_t bytes, unsigned int flags = cudaMemAttachGlobal) { return hipMallocManaged(ptr, bytes, flags); }
  template <class T> inline cudaError_t cudaMallocManaged(T** ptr, std::size_t bytes, unsigned int flags = cudaMemAttachGlobal) { return hipMallocManaged(reinterpret_cast<void**>(ptr), bytes, flags); }
  inline cudaError_t cudaFree(void* ptr) { return hipFree(ptr); }
  inline cudaError_t cudaMemcpy(void* dst, const void* src, std::size_t bytes, cudaMemcpyKind kind) { return hipMemcpy(dst, src, bytes, kind); }
  inline cudaError_t cudaMemset(void* dst, int value, std::size_t bytes) { return hipMemset(dst, value, bytes); }
  inline cudaError_t cudaDeviceSynchronize() { return hipDeviceSynchronize(); }
  inline cudaError_t cudaGetDevice(int* dev) { return hipGetDevice(dev); }
  inline cudaError_t cudaMemAdvise(const void* ptr, std::size_t bytes, cudaMemoryAdvise advice, int device) { return hipMemAdvise(ptr, bytes, advice, device); }
  #ifndef CUDART_PI_F
    #define CUDART_PI_F 3.14159265358979323846f
  #endif
  #define CHGPU_DEVICE_ABORT() __builtin_trap()
#elif defined(CHRONO_USE_HIP) || defined(__HIP_PLATFORM_AMD__) || defined(__HIP_PLATFORM_NVIDIA__)
  #ifndef __HIP_PLATFORM_AMD__
    #define __HIP_PLATFORM_AMD__
  #endif
  #include <hip/hip_runtime_api.h>
  using cudaError_t = hipError_t;
  using cudaMemcpyKind = hipMemcpyKind;
  using cudaMemoryAdvise = hipMemoryAdvise;
  static constexpr cudaError_t cudaSuccess = hipSuccess;
  static constexpr cudaError_t cudaErrorMemoryAllocation = hipErrorMemoryAllocation;
  static constexpr cudaError_t cudaErrorNotSupported = hipErrorNotSupported;
  static constexpr unsigned int cudaMemAttachGlobal = hipMemAttachGlobal;
  static constexpr cudaMemcpyKind cudaMemcpyHostToDevice = hipMemcpyHostToDevice;
  static constexpr cudaMemcpyKind cudaMemcpyDeviceToHost = hipMemcpyDeviceToHost;
  static constexpr cudaMemcpyKind cudaMemcpyDeviceToDevice = hipMemcpyDeviceToDevice;
  static constexpr cudaMemoryAdvise cudaMemAdviseSetReadMostly = hipMemAdviseSetReadMostly;
  inline const char* cudaGetErrorString(cudaError_t err) { return hipGetErrorString(err); }
  inline cudaError_t cudaPeekAtLastError() { return hipPeekAtLastError(); }
  inline cudaError_t cudaMalloc(void** ptr, std::size_t bytes) { return hipMalloc(ptr, bytes); }
  template <class T> inline cudaError_t cudaMalloc(T** ptr, std::size_t bytes) { return hipMalloc(reinterpret_cast<void**>(ptr), bytes); }
  inline cudaError_t cudaMallocManaged(void** ptr, std::size_t bytes, unsigned int flags = cudaMemAttachGlobal) { return hipMallocManaged(ptr, bytes, flags); }
  template <class T> inline cudaError_t cudaMallocManaged(T** ptr, std::size_t bytes, unsigned int flags = cudaMemAttachGlobal) { return hipMallocManaged(reinterpret_cast<void**>(ptr), bytes, flags); }
  inline cudaError_t cudaFree(void* ptr) { return hipFree(ptr); }
  inline cudaError_t cudaMemcpy(void* dst, const void* src, std::size_t bytes, cudaMemcpyKind kind) { return hipMemcpy(dst, src, bytes, kind); }
  inline cudaError_t cudaMemset(void* dst, int value, std::size_t bytes) { return hipMemset(dst, value, bytes); }
  inline cudaError_t cudaDeviceSynchronize() { return hipDeviceSynchronize(); }
  inline cudaError_t cudaGetDevice(int* dev) { return hipGetDevice(dev); }
  inline cudaError_t cudaMemAdvise(const void* ptr, std::size_t bytes, cudaMemoryAdvise advice, int device) { return hipMemAdvise(ptr, bytes, advice, device); }
  #ifndef CUDART_PI_F
    #define CUDART_PI_F 3.14159265358979323846f
  #endif
  #define CHGPU_DEVICE_ABORT() std::abort()
#else
  using cudaError_t = int;
  enum cudaMemcpyKind { cudaMemcpyHostToDevice = 1, cudaMemcpyDeviceToHost = 2, cudaMemcpyDeviceToDevice = 3 };
  enum cudaMemoryAdvise { cudaMemAdviseSetReadMostly = 1 };
  static constexpr cudaError_t cudaSuccess = 0;
  static constexpr cudaError_t cudaErrorMemoryAllocation = 2;
  static constexpr cudaError_t cudaErrorNotSupported = 801;
  static constexpr unsigned int cudaMemAttachGlobal = 1u;
  inline const char* cudaGetErrorString(cudaError_t) { return "GPU runtime unavailable in host-only translation unit"; }
  #define CHGPU_DEVICE_ABORT() std::abort()
#endif
