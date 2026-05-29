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
// Use: Adaptation for HIP/CUDA dual-backend support
//
// =============================================================================

#pragma once

#include "chrono/gpu/ChGpuPrimitives.h"

#include <cstddef>
#include <cstdlib>
#include <utility>

// Single and double precision definitions of PI
#define GPU_PI_F 3.141592654f
#define GPU_PI 3.14159265358979323846

// Device-side abort helper used by DEM macros such as ABORTABORTABORT.
// Keep this in the shared runtime header so CUDA/HIP device code and host-only
// fallback code see one consistent definition.
#ifndef CHGPU_DEVICE_ABORT
    #if defined(__HIPCC__) || defined(__HIP_DEVICE_COMPILE__)
        #define CHGPU_DEVICE_ABORT() __builtin_trap()
    #elif defined(__CUDA_ARCH__)
        #define CHGPU_DEVICE_ABORT() asm("trap;")
    #else
        #define CHGPU_DEVICE_ABORT() std::abort()
    #endif
#endif

#if defined(__CUDACC__) || defined(CHRONO_USE_CUDA)

    #include <cuda_runtime.h>
    #include <cuda_runtime_api.h>

// cudaMemcpyTo/FromSymbol need the actual device symbol token, not the address
// of that token. Current Chrono GPU code only uses the 3-argument forms, so map
// exactly those here.
    #define gpuMemcpyToSymbolAsync(symbol, src, bytes) cudaMemcpyToSymbolAsync(symbol, src, bytes, 0, cudaMemcpyHostToDevice, 0)
    #define gpuMemcpyFromSymbol(dst, symbol, bytes) cudaMemcpyFromSymbol(dst, symbol, bytes, 0, cudaMemcpyDeviceToHost)

// Type aliases
using gpuStream = cudaStream_t;
using gpuEvent = cudaEvent_t;
using gpuError = cudaError_t;
using gpuDeviceProp = cudaDeviceProp;
using gpuMemcpyKind = cudaMemcpyKind;
using gpuMemoryAdvise = cudaMemoryAdvise;

// Enumerator aliases
static constexpr gpuError gpuSuccess = cudaSuccess;
static constexpr gpuError gpuErrorMemoryAllocation = cudaErrorMemoryAllocation;
static constexpr gpuError gpuErrorNotSupported = cudaErrorNotSupported;
static constexpr unsigned int gpuMemAttachGlobal = cudaMemAttachGlobal;
static constexpr gpuMemcpyKind gpuMemcpyHostToDevice = cudaMemcpyHostToDevice;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToHost = cudaMemcpyDeviceToHost;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToDevice = cudaMemcpyDeviceToDevice;
static constexpr gpuMemoryAdvise gpuMemAdviseSetReadMostly = cudaMemAdviseSetReadMostly;

// Function wrappers
inline const char* gpuGetErrorString(gpuError err) {
    return cudaGetErrorString(err);
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

inline gpuError gpuMallocManaged(void** ptr, std::size_t bytes, unsigned int flags = gpuMemAttachGlobal) {
    return cudaMallocManaged(ptr, bytes, flags);
}

template <class T>
inline gpuError gpuMallocManaged(T** ptr, std::size_t bytes, unsigned int flags = gpuMemAttachGlobal) {
    return cudaMallocManaged(reinterpret_cast<void**>(ptr), bytes, flags);
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

inline gpuError gpuMemAdvise(const void* ptr, std::size_t bytes, gpuMemoryAdvise advice, int device) {
    #if defined(CUDART_VERSION) && CUDART_VERSION >= 13000
    cudaMemLocation location{};
    location.type = cudaMemLocationTypeDevice;
    location.id = device;
    return cudaMemAdvise(ptr, bytes, advice, location);
    #else
    return cudaMemAdvise(ptr, bytes, advice, device);
    #endif
}

    #if defined(CUDART_VERSION) && CUDART_VERSION >= 13000
inline gpuError gpuMemAdvise(const void* ptr, std::size_t bytes, gpuMemoryAdvise advice, cudaMemLocation location) {
    return cudaMemAdvise(ptr, bytes, advice, location);
}
    #endif

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

#elif defined(__HIPCC__) || defined(__HIP_DEVICE_COMPILE__) || defined(CHRONO_USE_HIP) || defined(__HIP_PLATFORM_AMD__) || defined(__HIP_PLATFORM_NVIDIA__)

    #ifndef __HIP_PLATFORM_AMD__
        #define __HIP_PLATFORM_AMD__
    #endif
    #include <hip/hip_runtime.h>
    #include <hip/hip_runtime_api.h>
    #define CHRONO_FSI_SPH_RUNTIME_IS_HIP 1

// Important: HIP_SYMBOL must be applied at the call site to the actual device
// symbol token. Wrapping hipMemcpyTo/FromSymbol in a C++ function template would
// apply HIP_SYMBOL to the function parameter `symbol` and not to the original
// device symbol; that code would still compile but route symbol copies
// incorrectly at runtime. Current Chrono GPU code only uses the 3-argument
// forms, so map exactly those here.
    #define gpuMemcpyToSymbolAsync(symbol, src, bytes) hipMemcpyToSymbolAsync(HIP_SYMBOL(symbol), src, bytes, 0, hipMemcpyHostToDevice, 0)
    #define gpuMemcpyFromSymbol(dst, symbol, bytes) hipMemcpyFromSymbol(dst, HIP_SYMBOL(symbol), bytes, 0, hipMemcpyDeviceToHost)

// Type aliases
using gpuStream = hipStream_t;
using gpuEvent = hipEvent_t;
using gpuError = hipError_t;
using gpuDeviceProp = hipDeviceProp_t;
using gpuMemcpyKind = hipMemcpyKind;
using gpuMemoryAdvise = hipMemoryAdvise;

// Enumerator aliases
static constexpr gpuError gpuSuccess = hipSuccess;
static constexpr gpuError gpuErrorMemoryAllocation = hipErrorMemoryAllocation;
static constexpr gpuError gpuErrorNotSupported = hipErrorNotSupported;
static constexpr unsigned int gpuMemAttachGlobal = hipMemAttachGlobal;
static constexpr gpuMemcpyKind gpuMemcpyHostToDevice = hipMemcpyHostToDevice;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToHost = hipMemcpyDeviceToHost;
static constexpr gpuMemcpyKind gpuMemcpyDeviceToDevice = hipMemcpyDeviceToDevice;
static constexpr gpuMemoryAdvise gpuMemAdviseSetReadMostly = hipMemAdviseSetReadMostly;

// Function wrappers
inline const char* gpuGetErrorString(gpuError err) {
    return hipGetErrorString(err);
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

inline gpuError gpuMallocManaged(void** ptr, std::size_t bytes, unsigned int flags = gpuMemAttachGlobal) {
    return hipMallocManaged(ptr, bytes, flags);
}

template <class T>
inline gpuError gpuMallocManaged(T** ptr, std::size_t bytes, unsigned int flags = gpuMemAttachGlobal) {
    return hipMallocManaged(reinterpret_cast<void**>(ptr), bytes, flags);
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

inline gpuError gpuMemAdvise(const void* ptr, std::size_t bytes, gpuMemoryAdvise advice, int device) {
    return hipMemAdvise(ptr, bytes, advice, device);
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
enum gpuMemoryAdvise { gpuMemAdviseSetReadMostly = 1 };

static constexpr gpuError gpuSuccess = 0;
static constexpr gpuError gpuErrorMemoryAllocation = 2;
static constexpr gpuError gpuErrorNotSupported = 801;
static constexpr unsigned int gpuMemAttachGlobal = 1u;

inline const char* gpuGetErrorString(gpuError) {
    return "GPU runtime unavailable in host-only translation unit";
}

inline gpuError gpuGetLastError() {
    return gpuSuccess;
}

inline gpuError gpuPeekAtLastError() {
    return gpuSuccess;
}

inline gpuError gpuMalloc(void**, std::size_t) {
    return gpuErrorNotSupported;
}

template <class T>
inline gpuError gpuMalloc(T**, std::size_t) {
    return gpuErrorNotSupported;
}

inline gpuError gpuMallocManaged(void**, std::size_t, unsigned int = gpuMemAttachGlobal) {
    return gpuErrorNotSupported;
}

template <class T>
inline gpuError gpuMallocManaged(T**, std::size_t, unsigned int = gpuMemAttachGlobal) {
    return gpuErrorNotSupported;
}

inline gpuError gpuFree(void*) {
    return gpuSuccess;
}

inline gpuError gpuMemcpy(void*, const void*, std::size_t, gpuMemcpyKind) {
    return gpuErrorNotSupported;
}

inline gpuError gpuMemcpyAsync(void*, const void*, std::size_t, gpuMemcpyKind, gpuStream = 0) {
    return gpuErrorNotSupported;
}

inline gpuError gpuMemset(void*, int, std::size_t) {
    return gpuErrorNotSupported;
}

inline gpuError gpuDeviceSynchronize() {
    return gpuSuccess;
}

inline gpuError gpuGetDevice(int*) {
    return gpuErrorNotSupported;
}

inline gpuError gpuGetDeviceProperties(gpuDeviceProp*, int) {
    return gpuErrorNotSupported;
}

inline gpuError gpuMemAdvise(const void*, std::size_t, gpuMemoryAdvise, int) {
    return gpuErrorNotSupported;
}

inline gpuError gpuStreamCreate(gpuStream*) {
    return gpuErrorNotSupported;
}

inline gpuError gpuStreamDestroy(gpuStream) {
    return gpuSuccess;
}

inline gpuError gpuStreamSynchronize(gpuStream) {
    return gpuSuccess;
}

inline gpuError gpuEventCreate(gpuEvent*) {
    return gpuErrorNotSupported;
}

inline gpuError gpuEventDestroy(gpuEvent) {
    return gpuSuccess;
}

inline gpuError gpuEventRecord(gpuEvent, gpuStream = 0) {
    return gpuErrorNotSupported;
}

inline gpuError gpuEventSynchronize(gpuEvent) {
    return gpuSuccess;
}

inline gpuError gpuEventElapsedTime(float*, gpuEvent, gpuEvent) {
    return gpuErrorNotSupported;
}

#endif
