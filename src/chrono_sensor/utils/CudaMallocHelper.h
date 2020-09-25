// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Eric Brandt
// =============================================================================
//
//
// =============================================================================

#ifndef CUDAMALLOCHELPER_H
#define CUDAMALLOCHELPER_H

#include <iostream>
#include <sstream>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_utils
/// @{

/// Function for creating a chunk of memory that will implicitely desconstruct itself.
/// @param size The number of values for which we should have space. Full memory length will be size*sizeof(T)
template <class T>
inline T* cudaMallocHelper(unsigned int size) {
    void* ret;
    cudaError_t err = cudaMalloc(&ret, size * sizeof(T));
    if (err != cudaSuccess) {
        std::stringstream s;
        s << "cudaMalloc failed with error " << err;
        throw std::runtime_error(s.str());
    }
    return (T*)ret;
}

/// The desconstructor that will be called to free memory from the device pointer.
/// @param ptr The pointer to the object that should be freed.
template <class T>
inline void cudaFreeHelper(T* ptr) {
    if (ptr)
        cudaFree((void*)ptr);
}

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
