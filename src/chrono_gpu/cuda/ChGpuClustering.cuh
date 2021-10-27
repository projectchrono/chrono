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
//
// Contains collision helper functions from ChNarrowphaseR
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include "chrono_thirdparty/cub/cub.cuh"

#include <cuda.h>
#include <cassert>
#include <cstdio>
#include <fstream>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdint>
#include <algorithm>

#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/cuda/ChCudaMathUtils.cuh"
#include "chrono_gpu/cuda/ChGpuHelpers.cuh"

using chrono::gpu::CHGPU_TIME_INTEGRATOR;
using chrono::gpu::CHGPU_FRICTION_MODE;
using chrono::gpu::CHGPU_ROLLING_MODE;
using chrono::gpu::SPHERE_GROUP;
using chrono::gpu::CLUSTER_GRAPH_METHOD;
using chrono::gpu::CLUSTER_SEARCH_METHOD;


/// @addtogroup gpu_cuda
/// @{

/// All spheres with > minPts contacts are core, other points maybe be border points.
// InitSphereGroup
static __global__ void init_sphere_group_gdbscan(unsigned int nSpheres,
                                           unsigned int* adj_num,
                                           SPHERE_GROUP* sphere_group,
                                           unsigned int minPts) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    sphere_group[mySphereID] = adj_num[mySphereID] > minPts ? SPHERE_GROUP::CORE : SPHERE_GROUP::NOISE;
}

/// G-DBSCAN; density-based h_clustering algorithm. Identifies core, border and noise points in h_clusters.
/// SEARCH STEP
/// min_pts: minimal number of points for a h_cluster
/// radius: proximity radius, points inside can form a h_cluster
/// GdbscanConstructGraph
static __host__ void gdbscan_construct_graph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts, float radius) {
    // printf("gdbscan_construct_graph");
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // /// 2 steps:
    // ///     1- compute all adjacent spheres inside radius
    // construct_adj_num_start_by_proximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
    //         radius, adj_num, adj_start);

    // ///     2- compute adjacency list
    // construct_adj_list_by_proximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
    //         radius, adj_num, adj_start, adj_list);
}

// compute adj_start from adj_num. assumes memory was allocated
// ComputeAdjStartFromAdjNum
static __host__ void cluster_adj_num2start(unsigned int nSpheres, unsigned int * adj_num, unsigned int * adj_start) {
    memcpy(adj_start, adj_num, sizeof(*adj_start) * nSpheres);
    /// all start indices after mySphereID depend on it -> inclusive sum
    void * d_temp_storage = NULL;
    size_t bytesize = 0;
    /// with d_temp_storage = NULL, InclusiveSum computes necessary bytesize
    cub::DeviceScan::ExclusiveSum(d_temp_storage, bytesize,
    adj_start, adj_start, nSpheres);
    gpuErrchk(cudaMalloc(&d_temp_storage, bytesize));
    /// Actually perform IncluseSum
    cub::DeviceScan::ExclusiveSum(d_temp_storage, bytesize,
    adj_start, adj_start, nSpheres);
    gpuErrchk(cudaFree(d_temp_storage));
}


/// InitSphereCluster
static __global__ void init_sphere_cluster(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int cluster_id) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    sphere_data->sphere_cluster[mySphereID] = cluster_id;
}

/// InitAdj
static __global__ void init_adj(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    if (mySphereID < nSpheres) {
        sphere_data->adj_num[mySphereID] = 0;
        sphere_data->adj_start[mySphereID] = 0;
        for (unsigned int i = mySphereID; i < mySphereID + MAX_SPHERES_TOUCHED_BY_SPHERE ; i++) {
            sphere_data->adj_list[i] = 0;
        }
    }
}

/// compute adj_num from chrono contact_active_map
/// need to be separate from ComputeAdjListByContact to compute adj_start
// FIND BETTER NAMES
// ComputeAdjNumByContact
static __global__ void cluster_contact_adj_num(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // don't overrun the array
    if (mySphereID < nSpheres) {
        size_t body_A_offset = MAX_SPHERES_TOUCHED_BY_SPHERE * mySphereID;

        // count the number of contacts.
        unsigned char numActiveContacts = 0;
        for (unsigned char body_B_offset = 0; body_B_offset < MAX_SPHERES_TOUCHED_BY_SPHERE; body_B_offset++) {
            bool active_contact = sphere_data->contact_active_map[body_A_offset + body_B_offset];
            if (active_contact) {
                numActiveContacts++;
            }
        }
        sphere_data->adj_num[mySphereID] = numActiveContacts;
    }
}

/// compute adj_num from chrono contact_active_map
// FIND BETTER NAMES
// ComputeAdjNumByContact
static __global__ void cluster_contact_adj_list(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // don't overrun the array
    if (mySphereID < nSpheres) {
        size_t body_A_offset = MAX_SPHERES_TOUCHED_BY_SPHERE * mySphereID;

        unsigned char numActiveContacts = 0;
        for (unsigned char body_B_offset = 0; body_B_offset < MAX_SPHERES_TOUCHED_BY_SPHERE; body_B_offset++) {
            bool active_contact = sphere_data->contact_active_map[body_A_offset + body_B_offset];
            if (active_contact) {
                sphere_data->adj_list[sphere_data->adj_start[mySphereID] + numActiveContacts] = sphere_data->contact_partners_map[body_A_offset + body_B_offset];;
                numActiveContacts++;
            }
        }
    }
}

/// GdbscanSearchGraph
__host__ void gdbscan_search_graph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts);


/// @} gpu_cuda
