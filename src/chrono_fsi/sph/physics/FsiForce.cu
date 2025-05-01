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
// Author: Milad Rakhsha
// =============================================================================
//
// Base class for processing sph force in fsi system.
// =============================================================================

#include <thrust/extrema.h>
#include <thrust/sort.h>

#include "chrono_fsi/sph/physics/FsiForce.cuh"
#include "chrono_fsi/sph/utils/UtilsDevice.cuh"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"

namespace chrono {
namespace fsi {
namespace sph {

FsiForce::FsiForce(FsiDataManager& data_mgr, BceManager& bce_mgr, bool verbose)
    : m_data_mgr(data_mgr), m_bce_mgr(bce_mgr), m_verbose(verbose) {
    cudaMallocErrorFlag(m_errflagD);
}

FsiForce::~FsiForce() {
    cudaFreeErrorFlag(m_errflagD);
}

void FsiForce::Initialize() {
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(ChFsiParamsSPH));
    cudaMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));
}

// Use invasive to avoid one extra copy.
// However, keep in mind that sorted is changed.
void FsiForce::CopySortedToOriginal_Invasive_R3(thrust::device_vector<Real3>& original,
                                                  thrust::device_vector<Real3>& sorted,
                                                  const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<uint> dummyMarkerIndex = gridMarkerIndex;
    thrust::sort_by_key(dummyMarkerIndex.begin(), dummyMarkerIndex.end(), sorted.begin());
    dummyMarkerIndex.clear();
    thrust::copy(sorted.begin(), sorted.end(), original.begin());
}

void FsiForce::CopySortedToOriginal_NonInvasive_R3(thrust::device_vector<Real3>& original,
                                                     const thrust::device_vector<Real3>& sorted,
                                                     const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<Real3> dummySorted = sorted;
    CopySortedToOriginal_Invasive_R3(original, dummySorted, gridMarkerIndex);
}

// Use invasive to avoid one extra copy.
// However, keep in mind that sorted is changed.
void FsiForce::CopySortedToOriginal_Invasive_R4(thrust::device_vector<Real4>& original,
                                                  thrust::device_vector<Real4>& sorted,
                                                  const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<uint> dummyMarkerIndex = gridMarkerIndex;
    thrust::sort_by_key(dummyMarkerIndex.begin(), dummyMarkerIndex.end(), sorted.begin());
    dummyMarkerIndex.clear();
    thrust::copy(sorted.begin(), sorted.end(), original.begin());
}

void FsiForce::CopySortedToOriginal_NonInvasive_R4(thrust::device_vector<Real4>& original,
                                                     thrust::device_vector<Real4>& sorted,
                                                     const thrust::device_vector<uint>& gridMarkerIndex) {
    thrust::device_vector<Real4> dummySorted = sorted;
    CopySortedToOriginal_Invasive_R4(original, dummySorted, gridMarkerIndex);
}

}  // namespace sph
}  // namespace fsi
}  // namespace chrono
