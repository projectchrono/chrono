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

#include <cstddef>
#include <cstdint>
#include <limits>

#if defined(__CUDACC__) || defined(CHRONO_USE_CUDA)
  #include <cuda_runtime.h>
#elif defined(__HIPCC__) || defined(__HIP_DEVICE_COMPILE__)
  #ifndef __HIP_PLATFORM_AMD__
    #define __HIP_PLATFORM_AMD__
  #endif
  #include <hip/hip_runtime.h>
#elif defined(CHRONO_USE_HIP) || defined(__HIP_PLATFORM_AMD__) || defined(__HIP_PLATFORM_NVIDIA__)
  // Host-side HIP build: use HIP-provided vector aliases/functions so later HIP
  // runtime includes do not collide with local fallback declarations.
  #if !defined(__HIP_PLATFORM_AMD__) && !defined(__HIP_PLATFORM_NVIDIA__)
    #define __HIP_PLATFORM_AMD__
  #endif
  #include <hip/hip_runtime_api.h>
#else
  #ifndef __host__
    #define __host__
  #endif
  #ifndef __device__
    #define __device__
  #endif
  #ifndef __global__
    #define __global__
  #endif
  #ifndef __shared__
    #define __shared__
  #endif
  #ifndef __constant__
    #define __constant__
  #endif
  #ifndef __forceinline__
    #define __forceinline__ inline
  #endif

struct int2 { int x, y; };
struct int3 { int x, y, z; };
struct int4 { int x, y, z, w; };
struct uint2 { unsigned int x, y; };
struct uint3 { unsigned int x, y, z; };
struct uint4 { unsigned int x, y, z, w; };
struct float2 { float x, y; };
struct float3 { float x, y, z; };
struct float4 { float x, y, z, w; };
struct double2 { double x, y; };
struct double3 { double x, y, z; };
struct double4 { double x, y, z, w; };

constexpr inline int2 make_int2(int x, int y) { return {x, y}; }
constexpr inline int3 make_int3(int x, int y, int z) { return {x, y, z}; }
constexpr inline int4 make_int4(int x, int y, int z, int w) { return {x, y, z, w}; }
constexpr inline uint2 make_uint2(unsigned int x, unsigned int y) { return {x, y}; }
constexpr inline uint3 make_uint3(unsigned int x, unsigned int y, unsigned int z) { return {x, y, z}; }
constexpr inline uint4 make_uint4(unsigned int x, unsigned int y, unsigned int z, unsigned int w) { return {x, y, z, w}; }
constexpr inline float2 make_float2(float x, float y) { return {x, y}; }
constexpr inline float3 make_float3(float x, float y, float z) { return {x, y, z}; }
constexpr inline float4 make_float4(float x, float y, float z, float w) { return {x, y, z, w}; }
constexpr inline double2 make_double2(double x, double y) { return {x, y}; }
constexpr inline double3 make_double3(double x, double y, double z) { return {x, y, z}; }
constexpr inline double4 make_double4(double x, double y, double z, double w) { return {x, y, z, w}; }
#endif

// Provide cuda::std as a lightweight host fallback only when we are not
// compiling against the real CUDA CCCL/libcudacxx headers. In CUDA builds,
// cuda::std is provided by <cuda/std/...>; aliasing it to ::std in ordinary
// C++ translation units conflicts with CUDA 12/13 CCCL headers.
#if !defined(__CUDACC__) && !defined(CHRONO_USE_CUDA)
namespace cuda {
namespace std = ::std;
}
#endif
