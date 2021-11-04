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
// Contains functions that separated soil particles into clusters
// Reference: Andrade, Guilherme, et al. "G-dbscan: A gpu accelerated algorithm
// for density-based clustering." Procedia Computer Science 18 (2013): 369-378.
// =============================================================================
// Authors: Gabriel Taillon
// =============================================================================

#pragma once

#include <cassert>
#include <cstdio>
#include <fstream>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdint>
#include <algorithm>
#include <cuda.h>

#include "chrono_thirdparty/cub/cub.cuh"
#include "chrono/core/ChMathematics.h"
#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"
#include "chrono_thirdparty/cub/device/device_reduce.cuh"

using chrono::gpu::CHGPU_TIME_INTEGRATOR;
using chrono::gpu::CHGPU_FRICTION_MODE;
using chrono::gpu::CHGPU_ROLLING_MODE;
using chrono::gpu::SPHERE_TYPE;
using chrono::gpu::CLUSTER_INDEX;
using chrono::gpu::CLUSTER_GRAPH_METHOD;
using chrono::gpu::CLUSTER_SEARCH_METHOD;


/// @addtogroup gpu_cuda
/// @{

/// spheres with > minPts contacts are CORE, others are NOISE
/// Only NOISE spheres may be changed to BORDER later
static __global__ void GdbscanInitSphereGroup(unsigned int nSpheres,
                                              unsigned int* adj_num,
                                              SPHERE_TYPE* sphere_type,
                                              unsigned int minPts) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        sphere_type[mySphereID] = adj_num[mySphereID] > minPts ?
        SPHERE_TYPE::CORE : SPHERE_TYPE::NOISE;
    }
}

// Tag spheres found inside mesh to type VOLUME
// must run AFTER interactionGranMat_TriangleSoup
static __global__ void SetVolumeSphereGroup(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                            unsigned int nSpheres) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if (sphere_data->sphere_inside_mesh[mySphereID]) {
           sphere_data->sphere_type[mySphereID] = SPHERE_TYPE::VOLUME;
       }
    }
}

/// Any particle with sphere_type == NOISE are not part of any cluster
/// -> sphere_cluster = INVALID
/// Should be run at the end of search step
static __global__ void GdbscanFinalClusterFromGroup(unsigned int nSpheres,
                                                    unsigned int* sphere_cluster,
                                                    SPHERE_TYPE* sphere_type) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if (sphere_type[mySphereID] == SPHERE_TYPE::NOISE) {
            sphere_cluster[mySphereID] = static_cast<unsigned int>(CLUSTER_INDEX::INVALID);
        }
    }
}

// Find if any particle is in the volume cluster.
// If any sphere in cluster sphere_type == VOLUME -> sphere_cluster = VOLUME
// UNLESS it is the biggest cluster -> sphere_cluster = GROUND
static __global__ void FindVolumeCluster(unsigned int nSpheres,
                                         bool * visited,
                                         bool * in_volume,
                                         SPHERE_TYPE* sphere_type) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if ((sphere_type[mySphereID] == SPHERE_TYPE::VOLUME) && (visited[mySphereID])) {
            in_volume[mySphereID] = true;
        }
    }
}

/// Compute adj_start from adj_num with ExclusiveSum
/// needs fully known adj_num
/// call BEFORE ComputeAdjList___
static __host__ void ComputeAdjStartFromAdjNum(unsigned int nSpheres,
                                               unsigned int * adj_num,
                                               unsigned int * adj_start) {
    memcpy(adj_start, adj_num, sizeof(*adj_start) * nSpheres);
    /// all start indices AFTER mySphereID depend on it -> exclusive sum
    void * d_temp_storage = NULL;
    size_t bytesize = 0;
    /// with d_temp_storage = NULL, ExclusiveSum computes necessary bytesize
    cub::DeviceScan::ExclusiveSum(d_temp_storage, bytesize,
    adj_start, adj_start, nSpheres);
    gpuErrchk(cudaMalloc(&d_temp_storage, bytesize));
    /// Actually perform ExcluseSum
    cub::DeviceScan::ExclusiveSum(d_temp_storage, bytesize,
    adj_start, adj_start, nSpheres);
    gpuErrchk(cudaFree(d_temp_storage));

    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}

/// Compute adj_num from chrono contact_active_map
/// adj_start needs fully known adj_num
/// adl_list needs fully known adj_start
static __global__ void ComputeAdjNumByContact(
        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
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
            unsigned int contact_id = body_A_offset + body_B_offset;
            bool active_contact = sphere_data->contact_active_map[contact_id];
            if (active_contact) {
                numActiveContacts++;
            }
        }
        sphere_data->adj_num[mySphereID] = numActiveContacts;
    }
}

/// Compute adj_list from chrono contact_active_map.
/// call AFTER ComputeAdjNumByContact and ComputeAdjStartFromAdjNum
/// adj_list needs fully known ajd_start, which needs fully known adj_num
static __global__ void ComputeAdjListByContact(
        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
        ChSystemGpu_impl::GranParamsPtr gran_params,
        unsigned int nSpheres) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // don't overrun the array
    if (mySphereID < nSpheres) {
        size_t body_A_offset = MAX_SPHERES_TOUCHED_BY_SPHERE * mySphereID;

        unsigned char numActiveContacts = 0;
        for (unsigned char body_B_offset = 0; body_B_offset < MAX_SPHERES_TOUCHED_BY_SPHERE; body_B_offset++) {
            unsigned int contact_id = body_A_offset + body_B_offset;
            bool active_contact = sphere_data->contact_active_map[contact_id];
            if (active_contact) {
                unsigned int adj_list_index = sphere_data->adj_start[mySphereID] + numActiveContacts;
                sphere_data->adj_list[adj_list_index] = sphere_data->contact_partners_map[contact_id];
                numActiveContacts++;
            }
        }
    }
}

__host__ void ConstructGraphByContact(
        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
        ChSystemGpu_impl::GranParamsPtr gran_params,
        unsigned int nSpheres);

__host__ void ConstructGraphByProximity(
        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
        ChSystemGpu_impl::GranParamsPtr gran_params,
        unsigned int nSpheres, size_t min_pts, float radius);

__host__ void GdbscanSearchGraph(
        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
        ChSystemGpu_impl::GranParamsPtr gran_params,
        unsigned int nSpheres, size_t min_pts);

/// @} gpu_cuda
