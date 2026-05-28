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
// Author: Florian Reinle, Radu Serban
// =============================================================================
//
// Use: Adaption for HIP/CUDA dual-backend support
//
// =============================================================================

#pragma once

#include "chrono/gpu/ChGpuPrimitives.h"

#include <cstddef>
#include <utility>

#if defined(__CUDACC__) || defined(CHRONO_USE_CUDA)

    #include <cuda_runtime.h>
    #include <cuda_runtime_api.h>

// Type aliases
using gpuStream = cudaStream_t;
using gpuEvent = cudaEvent_t;
using gpuError = cudaError_t;
using gpuDeviceProp = cudaDeviceProp;
using gpuMemcpyKind = cudaMemcpyKind;

// Enumerator aliases
static constexpr gpuError gpuSuccess = cudaError::cudaSuccess;
static constexpr gpuError gpuErrorMemoryAllocation = cudaError::cudaErrorMemoryAllocation;
static constexpr gpuError gpuErrorNotSupported = cudaError::cudaErrorNotSupported;
static constexpr gpuMemcpyKind gpuMemcpyHostToDevice = cudaMemcpyKind::cudaMemcpyHostToDevice;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToHost = cudaMemcpyKind::cudaMemcpyDeviceToHost;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToDevice = cudaMemcpyKind::cudaMemcpyDeviceToDevice;

// Function wrappers
inline const char* gpuGetErrorString(gpuError err) {
    return cudaGetErrorString(static_cast<cudaError_t>(err));
}
inline gpuError gpuGetLastError() {
    return cudaGetLastError();
}
inline gpuError gpuPeekAtLastError() {
    return cudaPeekAtLastError();
}
inline gpuError gpuMalloc(void** ptr, std::size_t bytes) {
    return cudaMalloc(ptr, bytes);
}
template <class T>
inline gpuError gpuMalloc(T** ptr, std::size_t bytes) {
    return cudaMalloc(reinterpret_cast<void**>(ptr), bytes);
}
inline gpuError gpuFree(void* ptr) {
    return cudaFree(ptr);
}
inline gpuError gpuMemcpy(void* dst, const void* src, std::size_t bytes, gpuMemcpyKind kind) {
    return cudaMemcpy(dst, src, bytes, kind);
}
inline gpuError gpuMemcpyAsync(void* dst, const void* src, std::size_t bytes, gpuMemcpyKind kind, gpuStream stream = 0) {
    return cudaMemcpyAsync(dst, src, bytes, kind, stream);
}
    // cudaMemcpyTo/FromSymbol need the actual device symbol token, not the address of that
    // token. Keep these as macros so CUDA and HIP call sites use the same spelling.
    // Current Chrono GPU code only uses the 3-argument forms, so map exactly those here.
    #define gpuMemcpyToSymbolAsync(symbol, src, bytes) \
        cudaMemcpyToSymbolAsync(symbol, src, bytes, 0, cudaMemcpyHostToDevice, 0)
    #define gpuMemcpyFromSymbol(dst, symbol, bytes) \
        cudaMemcpyFromSymbol(dst, symbol, bytes, 0, cudaMemcpyDeviceToHost)
inline gpuError gpuMemset(void* dst, int value, std::size_t bytes) {
    return cudaMemset(dst, value, bytes);
}
inline gpuError gpuDeviceSynchronize() {
    return cudaDeviceSynchronize();
}
inline gpuError gpuGetDevice(int* dev) {
    return cudaGetDevice(dev);
}
inline gpuError gpuGetDeviceProperties(gpuDeviceProp* prop, int device) {
    return cudaGetDeviceProperties(prop, device);
}
inline gpuError gpuStreamCreate(gpuStream* stream) {
    return cudaStreamCreate(stream);
}
inline gpuError gpuStreamDestroy(gpuStream stream) {
    return cudaStreamDestroy(stream);
}
inline gpuError gpuStreamSynchronize(gpuStream stream) {
    return cudaStreamSynchronize(stream);
}
inline gpuError gpuEventCreate(gpuEvent* event) {
    return cudaEventCreate(event);
}
inline gpuError gpuEventDestroy(gpuEvent event) {
    return cudaEventDestroy(event);
}
inline gpuError gpuEventRecord(gpuEvent event, gpuStream stream = 0) {
    return cudaEventRecord(event, stream);
}
inline gpuError gpuEventSynchronize(gpuEvent event) {
    return cudaEventSynchronize(event);
}
inline gpuError gpuEventElapsedTime(float* ms, gpuEvent start, gpuEvent stop) {
    return cudaEventElapsedTime(ms, start, stop);
}

#elif defined(__HIPCC__) || defined(__HIP_DEVICE_COMPILE__)

    #ifndef __HIP_PLATFORM_AMD__
        #define __HIP_PLATFORM_AMD__
    #endif
    #include <hip/hip_runtime.h>
    #include <hip/hip_runtime_api.h>
    #define CHRONO_FSI_SPH_RUNTIME_IS_HIP 1

// Type aliases
using gpuStream = hipStream_t;
using gpuEvent = hipEvent_t;
using gpuError = int;
using gpuDeviceProp = hipDeviceProp_t;
using gpuMemcpyKind = hipMemcpyKind;

// Enumerator aliases
static constexpr gpuError gpuSuccess = static_cast<int>(hipSuccess);
static constexpr gpuError gpuErrorMemoryAllocation = static_cast<int>(hipErrorMemoryAllocation);
static constexpr gpuError gpuErrorNotSupported = static_cast<int>(hipErrorNotSupported);
static constexpr gpuMemcpyKind gpuMemcpyHostToDevice = hipMemcpyHostToDevice;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToHost = hipMemcpyDeviceToHost;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToDevice = hipMemcpyDeviceToDevice;

// Function wrappers
inline const char* gpuGetErrorString(gpuError err) {
    return hipGetErrorString(static_cast<hipError_t>(err));
}
inline gpuError gpuGetLastError() {
    return hipGetLastError();
}
inline gpuError gpuPeekAtLastError() {
    return hipPeekAtLastError();
}
inline gpuError gpuMalloc(void** ptr, std::size_t bytes) {
    return hipMalloc(ptr, bytes);
}
template <class T>
inline gpuError gpuMalloc(T** ptr, std::size_t bytes) {
    return hipMalloc(reinterpret_cast<void**>(ptr), bytes);
}
inline gpuError gpuFree(void* ptr) {
    return hipFree(ptr);
}
inline gpuError gpuMemcpy(void* dst, const void* src, std::size_t bytes, gpuMemcpyKind kind) {
    return hipMemcpy(dst, src, bytes, kind);
}
inline gpuError gpuMemcpyAsync(void* dst, const void* src, std::size_t bytes, gpuMemcpyKind kind, gpuStream stream = 0) {
    return hipMemcpyAsync(dst, src, bytes, kind, stream);
}
    // Important: HIP_SYMBOL must be applied at the call site to the actual device symbol token.
    // Wrapping hipMemcpyTo/FromSymbol in a C++ function template would apply HIP_SYMBOL to the
    // function parameter `symbol`, not to the original device symbol (e.g., paramsD/countersD),
    // which can compile but route symbol copies incorrectly at runtime.
    // Current Chrono GPU code only uses the 3-argument forms, so map exactly those here.
    #define gpuMemcpyToSymbolAsync(symbol, src, bytes) hipMemcpyToSymbolAsync(HIP_SYMBOL(symbol), src, bytes, 0, hipMemcpyHostToDevice, 0)
    #define gpuMemcpyFromSymbol(dst, symbol, bytes) hipMemcpyFromSymbol(dst, HIP_SYMBOL(symbol), bytes, 0, hipMemcpyDeviceToHost)
inline gpuError gpuMemset(void* dst, int value, std::size_t bytes) {
    return hipMemset(dst, value, bytes);
}
inline gpuError gpuDeviceSynchronize() {
    return hipDeviceSynchronize();
}
inline gpuError gpuGetDevice(int* dev) {
    return hipGetDevice(dev);
}
inline gpuError gpuGetDeviceProperties(gpuDeviceProp* prop, int device) {
    return hipGetDeviceProperties(prop, device);
}
inline gpuError gpuStreamCreate(gpuStream* stream) {
    return hipStreamCreate(stream);
}
inline gpuError gpuStreamDestroy(gpuStream stream) {
    return hipStreamDestroy(stream);
}
inline gpuError gpuStreamSynchronize(gpuStream stream) {
    return hipStreamSynchronize(stream);
}
inline gpuError gpuEventCreate(gpuEvent* event) {
    return hipEventCreate(event);
}
inline gpuError gpuEventDestroy(gpuEvent event) {
    return hipEventDestroy(event);
}
inline gpuError gpuEventRecord(gpuEvent event, gpuStream stream = 0) {
    return hipEventRecord(event, stream);
}
inline gpuError gpuEventSynchronize(gpuEvent event) {
    return hipEventSynchronize(event);
}
inline gpuError gpuEventElapsedTime(float* ms, gpuEvent start, gpuEvent stop) {
    return hipEventElapsedTime(ms, start, stop);
}

#else

struct gpuDeviceProp {
    int _chrono_placeholder = 0;
};

using gpuStream = void*;
using gpuEvent = void*;
using gpuError = int;

enum gpuMemcpyKind { gpuMemcpyHostToDevice = 1, gpuMemcpyDeviceToHost = 2, gpuMemcpyDeviceToDevice = 3 };
static constexpr gpuError gpuSuccess = 0;
static constexpr gpuError gpuErrorMemoryAllocation = 2;
static constexpr gpuError gpuErrorNotSupported = 801;

inline const char* gpuGetErrorString(gpuError) {
    return "GPU runtime unavailable in host-only translation unit";
}
inline gpuError gpuGetLastError() {
    return 0;
}
inline gpuError gpuPeekAtLastError() {
    return 0;
}

#endif
