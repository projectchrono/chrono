// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Utilities for processing FEA mesh data on the device.
//
// =============================================================================

////#define DEBUG_LOG

#include <cstdio>

#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/transform.h>

#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/physics/SphFsiInterface.cuh"
#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

__global__ void calcDir1D(const int2* nodes, const Real3* pos, int* key, Real3* dir, uint num_seg) {
    uint tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_seg)
        return;

    int n0 = nodes[tid].x;
    int n1 = nodes[tid].y;
    Real3 p0 = pos[n0];
    Real3 p1 = pos[n1];
    Real3 d = get_normalized(p1 - p0);
    key[2 * tid + 0] = n0;
    key[2 * tid + 1] = n1;
    dir[2 * tid + 0] = d;
    dir[2 * tid + 1] = d;
}

__global__ void calcDir2D(const int3* nodes, const Real3* pos, int* key, Real3* dir, uint num_seg) {
    uint tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_seg)
        return;

    int n0 = nodes[tid].x;
    int n1 = nodes[tid].y;
    int n2 = nodes[tid].z;
    Real3 p0 = pos[n0];
    Real3 p1 = pos[n1];
    Real3 p2 = pos[n2];
    Real3 d = calc_triangle_normal(p0, p1, p2);
    key[3 * tid + 0] = n0;
    key[3 * tid + 1] = n1;
    key[3 * tid + 2] = n2;
    dir[3 * tid + 0] = d;
    dir[3 * tid + 1] = d;
    dir[3 * tid + 2] = d;
}

struct add_dir {
    __host__ __device__ Real3 operator()(const Real3& d1, const Real3& d2) { return d1 + d2; }
};

struct normalize_dir {
    __host__ __device__ Real3 operator()(const Real3& d) { return get_normalized(d); }
};

void calculateDirectionsMesh1D(FsiDataManager& data_mgr) {
    const auto& nodes = data_mgr.flex1D_Nodes_D;
    const auto& pos = data_mgr.fsiMesh1DState_D->pos;
    auto& dir = data_mgr.fsiMesh1DState_D->dir;

    uint num_seg = (uint)nodes.size();
    thrust::device_vector<Real3> ext_dir(2 * num_seg);
    thrust::device_vector<int> key(2 * num_seg);
    thrust::device_vector<int> out_key(2 * num_seg);

    uint numThreads, numBlocks;
    computeGridSize(num_seg, 1024, numBlocks, numThreads);
    calcDir1D<<<numBlocks, numThreads>>>(mI2CAST(nodes), mR3CAST(pos), I1CAST(key), mR3CAST(ext_dir), num_seg);

    thrust::sort_by_key(key.begin(), key.end(), ext_dir.begin());

    thrust::equal_to<int> binary_pred;
    add_dir binary_op;
    thrust::reduce_by_key(key.begin(), key.end(), ext_dir.begin(), out_key.begin(), dir.begin(), binary_pred,
                          binary_op);

    normalize_dir unary_op;
    thrust::transform(dir.begin(), dir.end(), dir.begin(), unary_op);

    //// TODO unsort
}

void calculateDirectionsMesh2D(FsiDataManager& data_mgr) {
    const auto& nodes = data_mgr.flex2D_Nodes_D;
    const auto& pos = data_mgr.fsiMesh2DState_D->pos;
    auto& dir = data_mgr.fsiMesh2DState_D->dir;

    uint num_tri = (uint)nodes.size();
    thrust::device_vector<Real3> ext_dir(3 * num_tri);
    thrust::device_vector<int> key(3 * num_tri);
    thrust::device_vector<int> out_key(3 * num_tri);

    uint numThreads, numBlocks;
    computeGridSize(num_tri, 1024, numBlocks, numThreads);
    calcDir2D<<<numBlocks, numThreads>>>(mI3CAST(nodes), mR3CAST(pos), I1CAST(key), mR3CAST(ext_dir), num_tri);

    thrust::sort_by_key(key.begin(), key.end(), ext_dir.begin());

    thrust::equal_to<int> binary_pred;
    add_dir binary_op;
    thrust::reduce_by_key(key.begin(), key.end(), ext_dir.begin(), out_key.begin(), dir.begin(), binary_pred,
                          binary_op);

    normalize_dir unary_op;
    thrust::transform(dir.begin(), dir.end(), dir.begin(), unary_op);

    //// TODO unsort
}

void printDirectionsMesh1D(FsiDataManager& data_mgr) {
    thrust::host_vector<Real3> pos = data_mgr.fsiMesh1DState_D->pos;
    thrust::host_vector<Real3> dir = data_mgr.fsiMesh1DState_D->dir;

    auto n = pos.size();
    std::cout << "-------" << std::endl;
    for (size_t i = 0; i < n; i++) {
        const Real3& p = pos[i];
        const Real3& d = dir[i];
        std::cout << p.x << " " << p.y << " " << p.z << "   |   ";
        std::cout << d.x << " " << d.y << " " << d.z << std::endl;
    }
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
