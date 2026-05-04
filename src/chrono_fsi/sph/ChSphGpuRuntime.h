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
// Author: Florian Reinle
// =============================================================================
// Use: Adaption for HIP/CUDA dual-backend support
// =============================================================================

#pragma once

#include "chrono_fsi/sph/ChSphGpuPrimitives.h"

#include <cstddef>
#include <utility>

#if defined(__CUDACC__) || defined(CHRONO_USE_CUDA)
  #include <cuda_runtime.h>
  #include <cuda_runtime_api.h>
  #include <cuda/std/limits>
#elif defined(__HIPCC__) || defined(__HIP_DEVICE_COMPILE__)
  #ifndef __HIP_PLATFORM_AMD__
    #define __HIP_PLATFORM_AMD__
  #endif
  #include <hip/hip_runtime.h>
  #include <hip/hip_runtime_api.h>
  #define CHRONO_FSI_SPH_RUNTIME_IS_HIP 1
  using cudaStream_t = hipStream_t;
  using cudaEvent_t = hipEvent_t;
  using cudaError_t = int;
  using cudaDeviceProp = hipDeviceProp_t;
  using cudaMemcpyKind = hipMemcpyKind;
  static constexpr cudaError_t cudaSuccess = static_cast<int>(hipSuccess);
  static constexpr cudaError_t cudaErrorMemoryAllocation = static_cast<int>(hipErrorMemoryAllocation);
  static constexpr cudaError_t cudaErrorNotSupported = static_cast<int>(hipErrorNotSupported);
  static constexpr cudaMemcpyKind cudaMemcpyHostToDevice = hipMemcpyHostToDevice;
  static constexpr cudaMemcpyKind cudaMemcpyDeviceToHost = hipMemcpyDeviceToHost;
  static constexpr cudaMemcpyKind cudaMemcpyDeviceToDevice = hipMemcpyDeviceToDevice;
  inline const char* cudaGetErrorString(cudaError_t err) { return hipGetErrorString(static_cast<hipError_t>(err)); }
  inline cudaError_t cudaGetLastError() { return hipGetLastError(); }
  inline cudaError_t cudaPeekAtLastError() { return hipPeekAtLastError(); }
  inline cudaError_t cudaMalloc(void** ptr, std::size_t bytes) { return hipMalloc(ptr, bytes); }
  template <class T> inline cudaError_t cudaMalloc(T** ptr, std::size_t bytes) { return hipMalloc(reinterpret_cast<void**>(ptr), bytes); }
  inline cudaError_t cudaFree(void* ptr) { return hipFree(ptr); }
  inline cudaError_t cudaMemcpy(void* dst, const void* src, std::size_t bytes, cudaMemcpyKind kind) { return hipMemcpy(dst, src, bytes, kind); }
  inline cudaError_t cudaMemcpyAsync(void* dst, const void* src, std::size_t bytes, cudaMemcpyKind kind, cudaStream_t stream = 0) { return hipMemcpyAsync(dst, src, bytes, kind, stream); }
  // Important: HIP_SYMBOL must be applied at the call site to the actual device symbol token.
  // Wrapping hipMemcpyTo/FromSymbol in a C++ function template would apply HIP_SYMBOL to the
  // function parameter `symbol`, not to the original device symbol (e.g., paramsD/countersD),
  // which can compile but route symbol copies incorrectly at runtime.
  // Current SPH code only uses the 3-argument CUDA forms, so map exactly those here.
  #define cudaMemcpyToSymbolAsync(symbol, src, bytes) hipMemcpyToSymbolAsync(HIP_SYMBOL(symbol), src, bytes, 0, hipMemcpyHostToDevice, 0)
  #define cudaMemcpyFromSymbol(dst, symbol, bytes)    hipMemcpyFromSymbol(dst, HIP_SYMBOL(symbol), bytes, 0, hipMemcpyDeviceToHost)
  inline cudaError_t cudaMemset(void* dst, int value, std::size_t bytes) { return hipMemset(dst, value, bytes); }
  inline cudaError_t cudaDeviceSynchronize() { return hipDeviceSynchronize(); }
  inline cudaError_t cudaGetDevice(int* dev) { return hipGetDevice(dev); }
  inline cudaError_t cudaGetDeviceProperties(cudaDeviceProp* prop, int device) { return hipGetDeviceProperties(prop, device); }
  inline cudaError_t cudaStreamCreate(cudaStream_t* stream) { return hipStreamCreate(stream); }
  inline cudaError_t cudaStreamDestroy(cudaStream_t stream) { return hipStreamDestroy(stream); }
  inline cudaError_t cudaStreamSynchronize(cudaStream_t stream) { return hipStreamSynchronize(stream); }
  inline cudaError_t cudaEventCreate(cudaEvent_t* event) { return hipEventCreate(event); }
  inline cudaError_t cudaEventDestroy(cudaEvent_t event) { return hipEventDestroy(event); }
  inline cudaError_t cudaEventRecord(cudaEvent_t event, cudaStream_t stream = 0) { return hipEventRecord(event, stream); }
  inline cudaError_t cudaEventSynchronize(cudaEvent_t event) { return hipEventSynchronize(event); }
  inline cudaError_t cudaEventElapsedTime(float* ms, cudaEvent_t start, cudaEvent_t stop) { return hipEventElapsedTime(ms, start, stop); }
#else
  struct cudaDeviceProp { int _chrono_placeholder = 0; };
  using cudaStream_t = void*;
  using cudaEvent_t = void*;
  using cudaError_t = int;
  enum cudaMemcpyKind { cudaMemcpyHostToDevice = 1, cudaMemcpyDeviceToHost = 2, cudaMemcpyDeviceToDevice = 3 };
  static constexpr cudaError_t cudaSuccess = 0;
  static constexpr cudaError_t cudaErrorMemoryAllocation = 2;
  static constexpr cudaError_t cudaErrorNotSupported = 801;
  inline const char* cudaGetErrorString(cudaError_t) { return "GPU runtime unavailable in host-only translation unit"; }
  inline cudaError_t cudaGetLastError() { return 0; }
  inline cudaError_t cudaPeekAtLastError() { return 0; }
#endif
