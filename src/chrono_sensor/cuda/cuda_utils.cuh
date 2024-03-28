// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nevindu M. Batagoda
// =============================================================================
//
// =============================================================================

#ifdef USE_SENSOR_NVDB

//#include <nanovdb/NanoVDB.h>
//#include <nanovdb/util/cuda/CudaDeviceBuffer.h>
//#include <nanovdb/util/GridHandle.h>

#include <openvdb/tools/LevelSetSphere.h> // replace with your own dependencies for generating the OpenVDB grid
#include <nanovdb/util/CreateNanoGrid.h> // converter from OpenVDB to NanoVDB (includes NanoVDB.h and GridManager.h)
#include <nanovdb/util/cuda/CudaDeviceBuffer.h>
#include <nanovdb/util/NodeManager.h>

namespace chrono {
namespace sensor {


void cuda_convert_float_buffer_to_NVDBVec3f(void* input, void* output, int n);

nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> createNanoVDBGridHandle(void* h_points_buffer, int n);

}  // namespace sensor
}  // namespace chrono

#endif

