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


/// counts number of adjacent spheres by proximity.
static __global__ void construct_adj_num_start_by_proximity(
                        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                        ChSystemGpu_impl::GranParamsPtr gran_params,
                        unsigned int nSpheres, float radius) {

    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int otherSphereID;

    unsigned int thisSD = blockIdx.x; // sphere_pos_local is relative to this SD
    unsigned int mySD = sphere_data->sphere_owner_SDs[mySphereID];
    unsigned int otherSD;

    int3 mySphere_pos_local, otherSphere_pos_local;
    double3 mySphere_pos_global, otherSphere_pos_local, distance;

    mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID], 
            sphere_data->sphere_local_pos_Y[mySphereID], sphere_data->sphere_local_pos_Z[mySphereID]);
    mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, mySphere_pos_local, gran_params));   

    if (mySphereID < nSpheres) {
    /// find all spheres inside the radius around mySphere
        for (size_t i = 0; (i < nSpheres); i++) {
            otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i], 
            sphere_data->sphere_local_pos_Y[i], sphere_data->sphere_local_pos_Z[i]);
            otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, otherSphere_pos_local, gran_params));   
        distance = mySphere_pos_global - otherSphere_pos_global;
            if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
                adj_num[i]++;
                adj_num[mySphereID]++;
        adj_start[i+1]++;
        /// perform an inclusive sum. start index of subsequent vertices depend on all previous indices.
        void * d_temp_storage = NULL;
        size_t temp_storage_bytesize = 0;
        cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytesize, adj_start + i, adj_start + i, nSpheres - i);
        gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytesize));
        cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytesize, adj_start + i, adj_start + i, nSpheres - i);
        }
    }
    }
}

/// computes adj_list from proximity using known adj_num and adj_start
static __global__ void construct_adj_list_by_proximity(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, float radius,
                                           unsigned int* adj_num,
                                           unsigned int* adj_start,
                                           unsigned int* adj_list) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int otherSphereID;

    unsigned int thisSD = blockIdx.x; // sphere_pos_local is relative to this SD
    unsigned int mySD = sphere_data->sphere_owner_SDs[mySphereID];
    unsigned int otherSD;

    unsigned int vertex_start = adj_start[mySphereID];
    unsigned int adjacency_num = 0;

    int3 mySphere_pos_local, otherSphere_pos_local;
    double3 mySphere_pos_global, otherSphere_pos_local;

    mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID], 
            sphere_data->sphere_local_pos_Y[mySphereID], sphere_data->sphere_local_pos_Z[mySphereID]);
    mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, mySphere_pos_local, gran_params));   

    if (mySphereID < nSpheres) {
        for (size_t i = 0; (i < nSpheres); i++) {
            otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i], 
            sphere_data->sphere_local_pos_Y[i], sphere_data->sphere_local_pos_Z[i]);
            otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, otherSphere_pos_local, gran_params));   
            distance = mySphere_pos_global - otherSphere_pos_global;
            if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
                adj_list[vertex_start + adjacency_num] = i;
        }
        }                  
    }
    assert(adjacency_num == vertex_adjacency_num[mySphereID]);
}

static __global__ void initial_sphere_group_gdbscan(unsigned int nSpheres,
                                           unsigned int* adj_num,
                                           SPHERE_GROUP* sphere_group,
                                           unsigned int minPts) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    sphere_group[mySphereID] = adj_num[mySphereID] > minPts ? SPHERE_GROUP::CORE : SPHERE_GROUP::NOISE;
}

/// computes cluster by breadth first search
/// return pointer to clusters: array of pointers to arrays of variable length
/// cluster[0][0] -> number of pointers
/// cluster[M][0] -> size of the Mth cluster
/// cluster[M][N] -> Nth point in Mth cluster
static __host__ unsigned int * cluster_search_BFS(unsigned int nSpheres,
                        unsigned int* adj_num,
                        unsigned int* adj_start,
                        unsigned int* adj_list,
                        SPHERE_GROUP* sphere_group) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    unsigned int ** clusters;
    // how to use vectors in CUDA?

    unsigned int cluster_index = 1; /// cluster 0 is reserved for ground.

    bool * borders; // [mySphereID] -> is vertex a border?
    bool * visited; // [mySphereID] -> was vertex visited?
    bool * visited_host; // [mySphereID] -> was vertex visited?
    gpuErrchk(cudaMalloc(&borders, sizeof(*borders) * nSpheres));
    gpuErrchk(cudaMalloc(&visited, sizeof(*visited) * nSpheres));
    visited_host = (bool *)calloc(sizeof(*visited_host), nSpheres);
    

    for (size_t i = 0; i < nSpheres; i++) {
        if (!visited_host[i]) {
            gpuErrchk(cudaMemset(&borders, false, sizeof(*borders) * nSpheres));
            gpuErrchk(cudaMemset(&visited, false, sizeof(*visited) * nSpheres));
            visited_host[i] = true;
            borders[i] = true;
            clustering_search_BFS_kernel<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(nSPheres,
                radius, adj_num, adj_start, adj_list,
                borders, visited);

            gpuErrchk(cudaMemcpy(visited_host, visited, sizeof(*visited) * nSpheres, cudaMemcpyDeeviceToHost));

            // cluster[0] is its size, so it length nSpheres + 1
            for (size_t i = 0; i < nSpheres; i++) {
                if (visited_host[i]) {
                   cluster[cluster[0]++] = i;
                }
            }
            cluster_index++;
        }
    gpuErrchk(cudaFree(borders));
    gpuErrchk(cudaFree(visited));
}

/// computes cluster by breadth first search 
static __global__ void cluster_search_BFS_kernel(unsigned int nSpheres,
                        unsigned int * adj_num
                        unsigned int * adj_start
                        unsigned int * adj_list,
                        bool * borders, bool * visited) {

    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int neighborSphereID;

    if (visited[mySphereID]) {
        visited[mySphereID] = true;
    borders[mySphereID] = false;
    unsigned int start = adj_start[mySphereID];
    unsigned int end = adj_start[mySphereID] + vertex_adjacency_num[mySphereID];
    for (size_t i = start; i < end; i++) {
        unsigned int neighborSphereID = adj_list[i];
        border[neighborSphereID] = !visited[neighborSphereID];
        }
    }
}


/// G-DBSCAN; density-based clustering algorithm. Identifies core, border and noise points in clusters.
/// min_pts: minimal number of points for a cluster
/// radius: proximity radius, points inside can form a cluster
static __host__ void clustering_gdbscan(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts, float radius) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    initial_sphere_group_gdbscan<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(nSPheres,
                radius, adj_num, adj_start, adj_list,
                borders, visited);
    /// 3 steps:
    ///     1- compute all adjacent spheres inside radius
    construct_adj_num_start_by_proximity<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSPheres,
            radius, adj_num, adj_start);

    ///     2- compute adjacency list
    construct_adj_list_by_proximity<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSPheres,
            radius, adj_num, adj_start, adj_list);


    ///     3- build clusters by breadth-first search (BFS)
    unsigned int cluster = malloc((nSpheres + 1) * sizeof(*cluster)); 
    bool * points_visited = calloc(snSpheres, sizeof(*point_visited)); 
    unsigned int * cluster;
    for (size_t i = 0; i < nSpheres; i++) {
        if (!point_visited[i]) {
            memset(cluster, 0, sizeof(*cluster) * (nSPheres + 1));
            cluster = clustering_search_BFS(sphere_data, gran_params, nSPheres,
                radius, adj_num, adj_start, adj_list, cluster);
            if (cluster[0] < min_pts) {
                sphere_data->sphere_group[cluster[j]] = SPHERE_GROUP::NOISE;
            } else {
                for (size_t j = 0; j < cluster[0]; j++) {
                    points_visited[cluster[j]] = true;
                    if (adj_num[cluster[j]] >= min_pts) {
                        sphere_data->sphere_group[cluster[j]] = SPHERE_GROUP::CORE;
                    } else {
                        sphere_data->sphere_group[cluster[j]] = SPHERE_GROUP::BORDER;
                    }
                }
            }
            points_visited[i] = true;
        }
    }
    free(cluster);
}

/// @} gpu_cuda
