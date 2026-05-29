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
// Utilities for GPU error testing and GPU timing
//
// =============================================================================

#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

GpuTimer::GpuTimer(gpuStream stream) : m_stream(stream) {
    gpuEventCreate(&m_start);
    gpuEventCreate(&m_stop);
}

GpuTimer::~GpuTimer() {
    gpuEventDestroy(m_start);
    gpuEventDestroy(m_stop);
}

void GpuTimer::Start() {
    gpuEventRecord(m_start, m_stream);
}

void GpuTimer::Stop() {
    gpuEventRecord(m_stop, m_stream);
}

float GpuTimer::Elapsed() {
    float elapsed;
    gpuEventSynchronize(m_stop);
    gpuEventElapsedTime(&elapsed, m_start, m_stop);
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
