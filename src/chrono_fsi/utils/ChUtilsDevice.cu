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
// Author: Milad Rakhsha, Arman Pazouki
// =============================================================================
//
// Base class for changing device arrays in non-cuda files
// =============================================================================
/**
 * @brief See collideSphereSphere.cuh for documentation.
 */

#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

void ChUtilsDevice::ResizeMyThrust3(thrust::device_vector<Real3>& mThrustVec, int mSize) {
    mThrustVec.resize(mSize);
}
void ChUtilsDevice::ResizeMyThrust4(thrust::device_vector<Real4>& mThrustVec, int mSize) {
    mThrustVec.resize(mSize);
}
void ChUtilsDevice::FillMyThrust3(thrust::device_vector<Real3>& mThrustVec, Real3 v) {  //
    thrust::fill(mThrustVec.begin(), mThrustVec.end(), v);
}
void ChUtilsDevice::FillMyThrust4(thrust::device_vector<Real4>& mThrustVec, Real4 v) {
    thrust::fill(mThrustVec.begin(), mThrustVec.end(), v);
}
void ChUtilsDevice::ClearMyThrustR3(thrust::device_vector<Real3>& mThrustVec) {
    mThrustVec.clear();
}
void ChUtilsDevice::ClearMyThrustR4(thrust::device_vector<Real4>& mThrustVec) {
    mThrustVec.clear();
}
void ChUtilsDevice::ClearMyThrustU1(thrust::device_vector<uint>& mThrustVec) {
    mThrustVec.clear();
}
void ChUtilsDevice::PushBackR3(thrust::device_vector<Real3>& mThrustVec, Real3 a3) {
    mThrustVec.push_back(a3);
}
void ChUtilsDevice::PushBackR4(thrust::device_vector<Real4>& mThrustVec, Real4 a4) {
    mThrustVec.push_back(a4);
}
void ChUtilsDevice::ResizeR3(thrust::device_vector<Real3>& mThrustVec, int size) {
    mThrustVec.resize(size);
}
void ChUtilsDevice::ResizeR4(thrust::device_vector<Real4>& mThrustVec, int size) {
    mThrustVec.resize(size);
}
void ChUtilsDevice::ResizeU1(thrust::device_vector<uint>& mThrustVec, int size) {
    mThrustVec.resize(size);
}

Real3 ChUtilsDevice::FetchElement(const thrust::device_vector<Real3>& DevVec, size_t i) {
    return DevVec[i];
}

void ChUtilsDevice::CopyD2H(thrust::device_vector<Real4>& DevVec, thrust::host_vector<Real4>& HostVec) {
    thrust::copy(DevVec.begin(), DevVec.end(), HostVec.begin());
}
void ChUtilsDevice::CopyD2H(thrust::device_vector<Real3>& DevVec, thrust::host_vector<Real3>& HostVec) {
    thrust::copy(DevVec.begin(), DevVec.end(), HostVec.begin());
}
void ChUtilsDevice::CopyD2H(thrust::device_vector<Real>& DevVec, thrust::host_vector<Real>& HostVec) {
    thrust::copy(DevVec.begin(), DevVec.end(), HostVec.begin());
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
