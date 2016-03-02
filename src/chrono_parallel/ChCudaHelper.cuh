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
    void set(T value) { cudaMemsetAsync(data_d, value, sizeof(T) * size_d); }
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
    }

    T* data_d;
    std::vector<T> data_h;
    size_t size_d;
};

// Used by cub
struct real3Min {
    inline CUDA_DEVICE real3 operator()(const real3& a, const real3& b) { return Min(a, b); }
};

struct real3Max {
    inline CUDA_DEVICE real3 operator()(const real3& a, const real3& b) { return Max(a, b); }
};
