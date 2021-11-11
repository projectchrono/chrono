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
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/cuda/ChCudaMathUtils.cuh"
#include "chrono_gpu/cuda/ChGpuHelpers.cuh"
#include "chrono_gpu/cuda/ChGpu_SMC.cuh"


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
static __global__ void GdbscanInitSphereType(unsigned int nSpheres,
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
static __global__ void SetVolumeSphereType(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
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
static __global__ void GdbscanFinalClusterFromType(unsigned int nSpheres,
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

/// Switch cluster index of spheres.
/// Mostly used to switch index of a certain cluster to CLUSTER_INDEX::GROUND
static __global__ void SwitchClusterIndex(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                          ChSystemGpu_impl::GranParamsPtr gran_params,
                                          unsigned int nSpheres,
                                          unsigned int cluster_from,
                                          unsigned int cluster_to) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if(sphere_data->sphere_cluster[mySphereID] == cluster_from) {
            sphere_data->sphere_cluster[mySphereID] = cluster_to;
        }
    }
}


/// Find if any particle in a cluster are below a certain z_lim
/// if input param cluster == NONE checks all spheres.
/// VOLUME cluster is set before ground cluster
static __global__ void AreSpheresBelowZLim(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres,
                                           bool * d_below,
                                           unsigned int cluster,
                                           float z_lim) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // don't overrun the array
    if (mySphereID < nSpheres) {
        int3 mySphere_pos_local;
        double3 mySphere_pos_global;
        unsigned int ownerSD = sphere_data->sphere_owner_SDs[mySphereID];

        mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID],
                                       sphere_data->sphere_local_pos_Y[mySphereID],
                                       sphere_data->sphere_local_pos_Z[mySphereID]);
        mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(ownerSD, mySphere_pos_local, gran_params));

        if ((sphere_data->sphere_cluster[mySphereID] == cluster) ||
            (cluster == static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::NONE))) {
            if (static_cast<float>(mySphere_pos_global.z * gran_params->LENGTH_UNIT) < z_lim) {
                d_below[mySphereID] = true;
            }
        }
    }
}

// Find if any particle is in the volume cluster.
// If any sphere in cluster sphere_type == VOLUME -> sphere_cluster = VOLUME
// UNLESS it is the biggest cluster -> sphere_cluster = GROUND
static __global__ void FindVolumeTypeInCluster(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                               unsigned int nSpheres,
                                               bool * in_volume,
                                               unsigned int cluster) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // don't overrun the array
    if (mySphereID < nSpheres) {
        if ((sphere_data->sphere_type[mySphereID] == SPHERE_TYPE::VOLUME) &&
            (sphere_data->sphere_cluster[mySphereID] == cluster)){
            in_volume[mySphereID] = true;
        }
    }
}

/// Compute adj_offset from adj_num with ExclusiveSum
/// needs fully known adj_num
/// call BEFORE ComputeAdjList___
static __host__ void ComputeAdjOffsetFromAdjNum(unsigned int nSpheres,
                                               unsigned int * adj_num,
                                               unsigned int * adj_offset) {
    memcpy(adj_offset, adj_num, sizeof(*adj_offset) * nSpheres);
    /// all start indices AFTER mySphereID depend on it -> exclusive sum
    void * d_temp_storage = NULL;
    size_t bytesize = 0;
    /// with d_temp_storage = NULL, ExclusiveSum computes necessary bytesize
    cub::DeviceScan::ExclusiveSum(d_temp_storage, bytesize,
    adj_offset, adj_offset, nSpheres);
    gpuErrchk(cudaMalloc(&d_temp_storage, bytesize));
    /// Actually perform ExcluseSum
    cub::DeviceScan::ExclusiveSum(d_temp_storage, bytesize,
    adj_offset, adj_offset, nSpheres);
    gpuErrchk(cudaFree(d_temp_storage));

    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}

/// Compute adj_num from chrono contact_active_map
/// adj_offset needs fully known adj_num
/// adl_list needs fully known adj_offset
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
/// call AFTER ComputeAdjNumByContact and ComputeAdjOffsetFromAdjNum
/// adj_list needs fully known ajd_start, which needs fully known adj_num
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
            unsigned int contact_id = body_A_offset + body_B_offset;
            bool active_contact = sphere_data->contact_active_map[contact_id];
            if (active_contact) {
                unsigned int adj_list_index = sphere_data->adj_offset[mySphereID] + numActiveContacts;
                sphere_data->adj_list[adj_list_index] = sphere_data->contact_partners_map[contact_id];
                numActiveContacts++;
            }
        }
    }
}

/// ClusterSearchBFSKernel: GPU part of Breadth First Search (BFS) algorithm
/// Visits all border spheres (d_borders) in parallel, finds neighbors
/// Neighbors put into d_borders, visited on next call
/// Call until no border spheres i.e. nothing true in d_borders
static __global__ void ClusterSearchBFSKernel(unsigned int nSpheres,
                                              unsigned int * adj_num,
                                              unsigned int * adj_offset,
                                              unsigned int * adj_list,
                                              bool * d_borders,
                                              bool * d_visited) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    if (mySphereID < nSpheres) {
        if (d_borders[mySphereID]) {
            d_visited[mySphereID] = true;
            d_borders[mySphereID] = false;

            unsigned int start = adj_offset[mySphereID];
            unsigned int end = adj_offset[mySphereID] + adj_num[mySphereID];
            for (size_t i = start; i < end; i++) {
                unsigned int neighborSphereID = adj_list[i];
                if (!d_visited[neighborSphereID]) {
                    d_borders[neighborSphereID] = true;
                }
            }
        }
    }
}

/// UNTESTED
/// counts number of adjacent spheres by proximity.
static __global__ void ComputeAdjNumByProximity(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                                ChSystemGpu_impl::GranParamsPtr gran_params,
                                                unsigned int nSpheres,
                                                float radius) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int otherSphereID;

    unsigned int thisSD = blockIdx.x;
    unsigned int mySD = sphere_data->sphere_owner_SDs[mySphereID];
    unsigned int otherSD;

    int3 mySphere_pos_local, otherSphere_pos_local;
    double3 mySphere_pos_global, otherSphere_pos_global, distance;

    mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID],
                                   sphere_data->sphere_local_pos_Y[mySphereID],
                                   sphere_data->sphere_local_pos_Z[mySphereID]);
    mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, mySphere_pos_local, gran_params));

    if (mySphereID < nSpheres) {
    /// find all spheres inside the radius around mySphere
        for (size_t i = 0; (i < nSpheres); i++) {
            otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i],
                                              sphere_data->sphere_local_pos_Y[i],
                                              sphere_data->sphere_local_pos_Z[i]);
            otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(
                thisSD,
                otherSphere_pos_local,
                gran_params));
            distance = mySphere_pos_global - otherSphere_pos_global;
            if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
                sphere_data->adj_num[i]++;
                sphere_data->adj_num[mySphereID]++;
            }
        }
    }
}

/// UNTESTED
/// computes adj_list from proximity using known adj_num and adj_offset
static __global__ void ComputeAdjListByProximity(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                                 ChSystemGpu_impl::GranParamsPtr gran_params,
                                                 unsigned int nSpheres,
                                                 float radius) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int otherSphereID;

    // sphere_pos_local is relative to thisSD
    unsigned int thisSD = blockIdx.x;
    unsigned int mySD = sphere_data->sphere_owner_SDs[mySphereID];
    unsigned int otherSD;

    unsigned int vertex_start = sphere_data->adj_offset[mySphereID];
    unsigned int adjacency_num = 0;

    int3 mySphere_pos_local, otherSphere_pos_local;
    double3 mySphere_pos_global, otherSphere_pos_global, distance;

    mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID],
                                   sphere_data->sphere_local_pos_Y[mySphereID],
                                   sphere_data->sphere_local_pos_Z[mySphereID]);
    mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(
        thisSD,
        mySphere_pos_local,
        gran_params));

    if (mySphereID < nSpheres) {
        for (size_t i = 0; (i < nSpheres); i++) {
            otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i],
                                              sphere_data->sphere_local_pos_Y[i],
                                              sphere_data->sphere_local_pos_Z[i]);
            otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(
                thisSD,
                otherSphere_pos_local,
                gran_params));
            distance = mySphere_pos_global - otherSphere_pos_global;
            if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
                sphere_data->adj_list[vertex_start + adjacency_num] = i;
            }
        }
    }
    assert(adjacency_num == sphere_data->adj_num[mySphereID]);
}

__host__ void ConstructGraphByContact(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                      ChSystemGpu_impl::GranParamsPtr gran_params,
                                      unsigned int nSpheres);

__host__ void ConstructGraphByProximity(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                        ChSystemGpu_impl::GranParamsPtr gran_params,
                                        unsigned int nSpheres,
                                        size_t min_pts,
                                        float radius);

__host__ unsigned int ** GdbscanSearchGraphByBFS(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                                 ChSystemGpu_impl::GranParamsPtr gran_params,
                                                 unsigned int nSpheres,
                                                 size_t min_pts);

__host__ void IdentifyGroundCluster(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                    ChSystemGpu_impl::GranParamsPtr gran_params,
                                    unsigned int nSpheres,
                                    unsigned int ** h_clusters);

__host__ void IdentifyVolumeCluster(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                    ChSystemGpu_impl::GranParamsPtr gran_params,
                                    unsigned int nSpheres,
                                    unsigned int ** h_clusters);

__host__ void FreeClusters(unsigned int ** h_clusters);

/// @} gpu_cuda
