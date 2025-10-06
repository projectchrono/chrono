// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Radu Serban
// =============================================================================
//
// Utilities for changing device arrays in non-cuda files
// =============================================================================

#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

GpuTimer::GpuTimer(cudaStream_t stream) : m_stream(stream) {
    cudaEventCreate(&m_start);
    cudaEventCreate(&m_stop);
}

GpuTimer::~GpuTimer() {
    cudaEventDestroy(m_start);
    cudaEventDestroy(m_stop);
}

void GpuTimer::Start() {
    cudaEventRecord(m_start, m_stream);
}

void GpuTimer::Stop() {
    cudaEventRecord(m_stop, m_stream);
}

float GpuTimer::Elapsed() {
    float elapsed;
    cudaEventSynchronize(m_stop);
    cudaEventElapsedTime(&elapsed, m_start, m_stop);
    return elapsed;
}

void computeGridSize(uint n, uint blockSize, uint& numBlocks, uint& numThreads) {
    uint n2 = (n == 0) ? 1 : n;
    numThreads = min(blockSize, n2);
    numBlocks = (n2 % numThreads != 0) ? (n2 / numThreads + 1) : (n2 / numThreads);
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
