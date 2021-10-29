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

#include "chrono/core/ChMathematics.h"
#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"
#include "chrono_thirdparty/cub/device/device_reduce.cuh"

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
static __global__ void GdbscanInitSphereGroup(unsigned int nSpheres,
                                           unsigned int* adj_num,
                                           SPHERE_GROUP* sphere_group,
                                           unsigned int minPts) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        sphere_group[mySphereID] = adj_num[mySphereID] > minPts ? SPHERE_GROUP::CORE : SPHERE_GROUP::NOISE;
    }
}

static __global__ void GdbscanInitSphereCluster(unsigned int nSpheres,
                                            unsigned int* sphere_cluster,
                                           unsigned int cluster_id) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        sphere_cluster[mySphereID] = cluster_id;
    }
}

// set spheres found inside mesh to group VOLUME
// must de run AFTER interactionGranMat_TriangleSoup()
static __global__ void SetVolumeSphereGroup(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                            unsigned int nSpheres) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if (sphere_data->sphere_inside_mesh[mySphereID]) {
           sphere_data->sphere_group[mySphereID] = SPHERE_GROUP::VOLUME;
       }
    }
}

static __global__ void GdbscanFinalClusterFromGroup(unsigned int nSpheres,
                                           unsigned int* sphere_cluster,
                                           SPHERE_GROUP* sphere_group) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if (sphere_group[mySphereID] == SPHERE_GROUP::NOISE) {
            sphere_cluster[mySphereID] = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::INVALID);
        }
    }
}

// find if any particle is in the bucket cluster.
// if any of bucket is true, this is the bucket cluster UNLESS
// it is the biggest cluster -> becomes the ground cluster.
static __global__ void FindBucketCluster(unsigned int nSpheres,
                                           bool * visited,
                                           bool * in_bucket,
                                           SPHERE_GROUP* sphere_group) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if ((sphere_group[mySphereID] == SPHERE_GROUP::VOLUME) && (visited[mySphereID])) {
            in_bucket[mySphereID] = true;
        }
    }
}

// compute adj_start from adj_num. assumes memory was allocated
static __host__ void ComputeAdjStartFromAdjNum(unsigned int nSpheres, 
                                                unsigned int * adj_num,
                                                unsigned int * adj_start) {
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

static __global__ void InitAdj(unsigned int nSpheres,
                                unsigned int * adj_num,
                                unsigned int * adj_start,
                                unsigned int * adj_list) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    if (mySphereID < nSpheres) {
        adj_num[mySphereID] = 0;
        adj_start[mySphereID] = 0;
        for (unsigned int i = mySphereID; i < mySphereID + MAX_SPHERES_TOUCHED_BY_SPHERE ; i++) {
            adj_list[i] = 0;
        }
    }
}

/// compute adj_num from chrono contact_active_map
/// need to be separate from ComputeAdjListByContact to compute adj_start
static __global__ void ComputeAdjNumByContact(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
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

/// compute adj_list from chrono contact_active_map
static __global__ void ComputeAdjListByContact(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
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

__host__ void GdbscanConstructGraph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts, float radius);

__host__ void GdbscanSearchGraph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts);

/// @} gpu_cuda
