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
// Author: Milad Rakhsha, Arman Pazouki, RFadu Serban
// =============================================================================
//
// Utilities for changing device arrays in non-cuda files
// =============================================================================

#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

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

void ChUtilsDevice::FillVector(thrust::device_vector<Real3>& vector, const Real3& value) {
    thrust::fill(vector.begin(), vector.end(), value);
}

void ChUtilsDevice::FillVector(thrust::device_vector<Real4>& vector, const Real4& value) {
    thrust::fill(vector.begin(), vector.end(), value);
}

void ChUtilsDevice::Sync_CheckError(bool* isErrorH, bool* isErrorD, std::string carshReport) {
    cudaDeviceSynchronize();
    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true) {
        throw std::runtime_error("Error! program crashed after " + carshReport + " !\n");
    }
    cudaError_t e = cudaGetLastError();
    if (e != cudaSuccess) {
        throw std::runtime_error("Error! program crashed after " + carshReport + " !\n");
    }
    cudaCheckError();
}

}  // end namespace fsi
}  // end namespace chrono
