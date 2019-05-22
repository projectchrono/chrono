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
// Description: Helper functions for cuda
// =============================================================================

#pragma once

#include <cuda.h>
#include <vector>

namespace chrono {
// Macro that will print an error statement if cuda returns and error

template <typename T>
class gpu_vector {
  public:
    // Construct a empty vector
    gpu_vector() : size_d(0), data_d(0) {}
    // Construct and allocate a gpu_vector with a certain size
    gpu_vector(size_t size) { allocate(size); }
    // Provide pointer to device data and size
    gpu_vector(T* ptr, size_t size) : size_d(size), data_d(ptr) {}
    // Clear the vector on destruct
    ~gpu_vector() { clear(); }
    // Allocate a vector if it has zero size, otherwise resize it
    void allocate(const size_t& size) {
        if (size_d == 0) {
            cudaMalloc(&data_d, sizeof(T) * size);
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
            cudaFree(data_d);
            cudaMalloc(&data_d, sizeof(T) * size);
            size_d = size;
        }
    }
    // Free allocated memory
    void clear() {
        if (size_d > 0) {
            cudaFree(data_d);
        }
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
        cudaMemcpy(&temp, &data_d[index], sizeof(T), cudaMemcpyDeviceToHost);
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
        cudaMemcpy((void*)data_d, data_h.data(), count, cudaMemcpyHostToDevice);
    }
    // Copy data from device memory to host stl vector. Resize memory as needed
    void copyDeviceToHost() {
		size_t new_size = size();
        if (new_size == 0) {
            return;
        }
        if (data_h.size() < new_size) {
            data_h.resize(new_size);
        }
        size_t count = new_size * sizeof(T);
        cudaMemcpy((void*)data_h.data(), data_d, count, cudaMemcpyDeviceToHost);
    }
    void operator=(const gpu_vector& rhs) {
        if (rhs.size() == 0) {
            return;
        }
        if (size() < rhs.size()) {
            resize(rhs.size());
        }
        size_t count = size() * sizeof(T);
        cudaMemcpy((void*)data_d, rhs.data_d, count, cudaMemcpyDeviceToDevice);
    }
    template <typename U>
    void operator=(const U rhs) {
        set(rhs);
    }

    T* data_d;
    std::vector<T> data_h;
    size_t size_d;
};
}
