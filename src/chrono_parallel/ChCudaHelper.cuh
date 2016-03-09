// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
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
// Description: Helper functions for cuda
// =============================================================================

#pragma once

#define num_threads_per_block 128

#include <cuda_runtime_api.h>
#include <cuda.h>
#include "thirdparty/cub/cub.cuh"

#include <cassert>
#include <vector>
#include "chrono_parallel/math/real.h"
#include "chrono_parallel/math/real3.h"
//
namespace chrono {
#define BLOCKS(x) (x + num_threads_per_block - 1) / num_threads_per_block
#define CONFIG(x) BLOCKS(x), num_threads_per_block

// Macro that will print an error statement if cuda returns and error
#define cudaCheck(x)                                                                \
    {                                                                               \
        cudaError_t err = x;                                                        \
        if (err != cudaSuccess) {                                                   \
            printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); \
            assert(0);                                                              \
        }                                                                           \
    }

#define Check(x)                      \
    x;                                \
    cudaCheck(cudaPeekAtLastError()); \
    cudaCheck(cudaDeviceSynchronize());

template <typename T>
class gpu_vector {
  public:
    // Construct a empty vector
    gpu_vector() : size_d(0), data_d(0) {}
    // Construct and allocate a gpu_vector with a certain size
    gpu_vector(size_t size) { allocate(size); }
    // Clear the vector on destruct
    ~gpu_vector() { clear(); }
    // Allocate a vector if it has zero size, otherwise resize it
    void allocate(const size_t& size) {
        if (size_d == 0) {
            cudaCheck(cudaMalloc(&data_d, sizeof(T) * size));
            size_d = size;
        } else {
            resize(size);
        }
    }
    // Only resize a vector if it was already allocated
    void resize(const size_t& size) {
        if (size_d == 0) {
            allocate(size);
        } else if (size_d != size) {
            cudaCheck(cudaFree(data_d));
            cudaCheck(cudaMalloc(&data_d, sizeof(T) * size));
            size_d = size;
        }
    }
    // Free allocated memory
    void clear() {
        cudaCheck(cudaFree(data_d));
        size_d = 0;
    }
    // Set the contents of a gpu_vector to a certain value
    template <typename U>
    void set(U value) {
        cudaMemsetAsync(data_d, value, sizeof(T) * size_d);
    }
    // return the size of a gpu vector
    size_t size() const { return size_d; }
    // Get a pointer to the gpu memory, useful when passing to a kernel
    T* operator()() { return data_d; }
    // Get a constant pointer to the gpu memory,  useful when passing to a kernel
    const T* operator()() const { return data_d; }

    // Used to read back a specific element from a gpu vector
    T operator[](const size_t index) {
        T temp;
        cudaCheck(cudaMemcpy(&temp, &data_d[index], sizeof(T), cudaMemcpyDeviceToHost));
        return temp;
    }

    // Host data is stored in an STL vector
    // Note that the data is not explicitly mirrored.
    // It's up to the user to resize the host data as needed.

    std::vector<T>& getHost() { return data_h; }

    // Copy data from host stl vector to device memory. Resize memory as needed
    void copyHostToDevice() {
        if (data_h.size() == 0) {
            return;
        }
        if (size() < data_h.size()) {
            resize(data_h.size());
        }
        size_t count = data_h.size() * sizeof(T);
        cudaCheck(cudaMemcpy((void*)data_d, data_h.data(), count, cudaMemcpyHostToDevice));
    }
    // Copy data from device memory to host stl vector. Resize memory as needed
    void copyDeviceToHost() {
        if (size() == 0) {
            return;
        }
        if (data_h.size() < size()) {
            data_h.resize(size());
        }
        size_t count = size() * sizeof(T);
        cudaCheck(cudaMemcpy((void*)data_h.data(), data_d, count, cudaMemcpyDeviceToHost));
    }
    void operator=(const gpu_vector& rhs) {
        if (rhs.size() == 0) {
            return;
        }
        if (size() < rhs.size()) {
            resize(rhs.size());
        }
        size_t count = size() * sizeof(T);
        cudaCheck(cudaMemcpy((void*)data_d, rhs.data_d, count, cudaMemcpyDeviceToDevice));
    }
    template <typename U>
    void operator=(const U rhs) {
        set(rhs);
    }

    T* data_d;
    std::vector<T> data_h;
    size_t size_d;
};

// Used by cub
struct real3Min {
    inline CUDA_HOST_DEVICE real3 operator()(const real3& a, const real3& b) {
        return real3(chrono::Min(a[0], b[0]), chrono::Min(a[1], b[1]), chrono::Min(a[2], b[2]));
    }
};

struct real3Max {
    inline CUDA_HOST_DEVICE real3 operator()(const real3& a, const real3& b) {
        return real3(chrono::Max(a[0], b[0]), chrono::Max(a[1], b[1]), chrono::Max(a[2], b[2]));
    }
};

// code adopted from http://stackoverflow.com/questions/17371275/implementing-max-reduce-in-cuda
// ========================================================================================

static CUDA_DEVICE double atomicAdd_d(double* address, double value) {
    unsigned long long int* address_as_ull = (unsigned long long int*)address;
    unsigned long long int old = *address_as_ull, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(value + __longlong_as_double(assumed)));
    } while (assumed != old);
    return __longlong_as_double(old);
}

static CUDA_DEVICE double AtomicMax(double* address, double value) {
    unsigned long long int* address_as_ull = (unsigned long long int*)address;
    unsigned long long int old = *address_as_ull, assumed;

    while (value > __longlong_as_double(old)) {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(value));
    }
    return __longlong_as_double(old);
}

static CUDA_DEVICE double AtomicMin(double* address, double value) {
    unsigned long long int* address_as_ull = (unsigned long long int*)address;
    unsigned long long int old = *address_as_ull, assumed;
    while (value < __longlong_as_double(old)) {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(value));
    }
    return __longlong_as_double(old);
}

static inline CUDA_DEVICE float AtomicMax(float* address, float value) {
    int* address_as_int = (int*)address;
    int old = *address_as_int, assumed;

    while (value > __int_as_float(old)) {
        assumed = old;
        old = atomicCAS(address_as_int, assumed, __float_as_int(value));
    }

    return __int_as_float(old);
}

static inline CUDA_DEVICE float AtomicMin(float* address, float value) {
    int* address_as_int = (int*)address;
    int old = *address_as_int, assumed;

    while (value < __int_as_float(old)) {
        assumed = old;
        old = atomicCAS(address_as_int, assumed, __float_as_int(value));
    }

    return __int_as_float(old);
}

static inline CUDA_DEVICE void AtomicAdd(real3* pointer, real3 val) {
#ifdef CHRONO_PARALLEL_USE_DOUBLE
    atomicAdd_d(&pointer->x, val.x);
    atomicAdd_d(&pointer->y, val.y);
    atomicAdd_d(&pointer->z, val.z);
#else
    atomicAdd(&pointer->x, val.x);
    atomicAdd(&pointer->y, val.y);
    atomicAdd(&pointer->z, val.z);
#endif
}

static inline CUDA_DEVICE void AtomicMax(real3* pointer, real3 val) {
    AtomicMax(&pointer->x, val.x);
    AtomicMax(&pointer->y, val.y);
    AtomicMax(&pointer->z, val.z);
}

static inline CUDA_DEVICE void AtomicMin(real3* pointer, real3 val) {
    AtomicMin(&pointer->x, val.x);
    AtomicMin(&pointer->y, val.y);
    AtomicMin(&pointer->z, val.z);
}
}
