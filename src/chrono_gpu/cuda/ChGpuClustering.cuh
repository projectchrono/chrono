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

/// UNTESTED
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
                }
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

/// UNTESTED
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

static __global__ void init_sphere_group_gdbscan(unsigned int nSpheres,
                                           unsigned int* adj_num,
                                           SPHERE_GROUP* sphere_group,
                                           unsigned int minPts) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    sphere_group[mySphereID] = adj_num[mySphereID] > minPts ? SPHERE_GROUP::CORE : SPHERE_GROUP::NOISE;
}

/// computes h_cluster by breadth first search
/// return pointer to h_clusters: array of pointers to arrays of variable length

static __host__ unsigned int * cluster_search_BFS(unsigned int nSpheres,
                        unsigned int* adj_num,
                        unsigned int* adj_start,
                        unsigned int* adj_list,
                        SPHERE_GROUP* sphere_group) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// CLUSTERS REPRESENTATION IN MEMORY
    /// ALTERNATIVE 1:
    /// clusters, array of points to arrays of variable lengths, with lengths at [0]
    unsigned int * h_cluster;
    unsigned int h_cluster_num = 1;
    unsigned int ** h_clusters;
    h_clusters = (unsigned int *)malloc(sizeof(*h_clusters) * (nSpheres+1)); // at worst, there will be nSpheres h_clusters
    h_clusters[0] = (unsigned int *)malloc(sizeof(**h_clusters)); // number of h_clusters at h_clusters[0][0]
    /// h_clusters[0][0] -> number of pointers/h_clusters
    /// h_clusters[M][0] -> size of the Mth h_cluster
    /// h_clusters[M][N] -> Nth point in Mth h_cluster

    /// CLUSTERS REPRESENTATION IN MEMORY
    /// ALTERNATIVE 2:
    // /// cluster_list/cluster_num/sphere_in_cluster_num similar to adj_num, adj_list (annoying to ouput)
    // unsigned int * h_cluster_list;
    // unsigned int h_cluster_num; /// number of spheres in each cluster
    // unsigned int * h_sphere_num_in_cluster; /// number of spheres in each cluster
    // unsigned int * h_cluster_start; /// index where cluster list starts in h_cluster_list
    // h_cluster_list = (unsigned int *)malloc(sizeof(*h_cluster_list) * nSpheres); // there is only nSpheres in ALL clusters
    // h_cluster_start = (unsigned int *)malloc(sizeof(*h_cluster_start) * nSpheres); // at worst, there will be nSpheres h_clusters
    // h_sphere_num_in_cluster = (unsigned int *)malloc(sizeof(*h_sphere_num_in_cluster) * nSpheres); // at worst, there will be nSpheres h_clusters

    bool * d_borders; // [mySphereID] -> is vertex a border?
    bool * d_visited; // [mySphereID] -> was vertex d_visited during BFS_kernel?
    bool * h_visited; // [mySphereID] -> host of d_visited
    bool * h_searched; // [mySphereID] -> was vertex searched before?
    unsigned int * d_border_num; // number of remaining border vertices to search in BFS_kernel
    unsigned int * h_border_num; // number of remaining border vertices to search in BFS_kernl
    gpuErrchk(cudaMalloc((void**)&d_borders, sizeof(*d_borders) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_visited, sizeof(*d_visited) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_border_num, sizeof(*d_border_num));
    h_visited = (bool *)calloc(sizeof(*h_visited), nSpheres);
    h_searched = (bool *)calloc(sizeof(*h_searched), nSpheres);
    SPHERE_GROUP h_current_group;

    for (size_t i = 0; i < nSpheres; i++) {
        gpuErrchk(cudaMemcpy(&h_current_group, &(sphere_data->sphere_group + i), sizeof(SPHERE_GROUP)), cudaMemcpyDeviceToHost);
        /// find the next h_cluster, at the first sphere not yet searched.
        if ((!h_searched[i]) && (h_current_group == SPHERE_GROUP::CORE)) {
            gpuErrchk(cudaMemset(&d_borders, false, sizeof(*d_borders) * nSpheres));
            gpuErrchk(cudaMemset(&d_visited, false, sizeof(*d_visited) * nSpheres));
            gpuErrchk(cudaMemset(&(d_border_num), 1, sizeof(*d_border_num)));
            gpuErrchk(cudaMemset(&(d_borders + i), true, sizeof(*d_borders)));
            h_searched[i] = true;

            do {
            h_clustering_search_BFS_kernel<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                radius, adj_num, adj_start, adj_list,
                d_borders, d_visited, d_border_num);
                gpuErrchk(cudaMemcpy(&h_border_num, &d_border_num, sizeof(*d_border_num)), cudaMemcpyDeviceToHost);
            } while (h_border_num > 0);
            
            gpuErrchk(cudaMemcpy(&h_visited, &d_visited, sizeof(*d_visited)* nSpheres, cudaMemcpyDeviceToHost);
            h_cluster = (unsigned int *)malloc(sizeof(*h_cluster) * (nSpheres + 1));
            // h_cluster[0] is its size, so it length nSpheres + 1
            for (size_t j = 0; i < nSpheres; j++) {
                if (h_visited[j]) {
                    h_searched[j] = true;
                    h_cluster[h_cluster[0]++] = j;
                    gpuErrchk(cudaMemcpy(&h_current_group, &(sphere_data->sphere_group + j), sizeof(SPHERE_GROUP)), cudaMemcpyDeviceToHost);
                    if (h_current_group != SPHERE_GROUP::CORE) {
                       gpuErrchk(cudaMemset(&(sphere_data->sphere_group + j), SPHERE_GROUP::BORDER, sizeof(*sphere_data->sphere_group)));
                    }
                }
            }
            h_cluster = (unsigned int *)realloc(h_cluster, sizeof(*h_cluster) * (h_cluster[0]));
            h_clusters[h_cluster_num] = h_cluster;
            h_clusters[0][0] = ++h_cluster_num; // h_clusters size is number of clusters + 1 cause it includes its length 
        }
    }
    h_clusters = (unsigned int *)realloc(h_clusters, sizeof(*h_clusters) * (h_clusters[0][0]));

    free(h_visited);
    free(h_searched);
    free(h_border_num);
    free(h_current_group);
    gpuErrchk(cudaFree(d_borders));
    gpuErrchk(cudaFree(d_border_num));
    gpuErrchk(cudaFree(d_visited));
    return(h_clusters);
}

/// computes h_cluster by breadth first search
static __global__ void cluster_search_BFS_kernel(unsigned int nSpheres,
                        unsigned int * adj_num
                        unsigned int * adj_start
                        unsigned int * adj_list,
                        bool * d_borders, 
                        bool * d_visited,
                        unsigned int * d_border_num) {

    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int neighborSphereID;

    if (d_visited[mySphereID]) {
        d_visited[mySphereID] = true;
        d_borders[mySphereID] = false;
        atomicAdd(d_border_num, -1);
        unsigned int start = adj_start[mySphereID];
        unsigned int end = adj_start[mySphereID] + vertex_adjacency_num[mySphereID];
        for (size_t i = start; i < end; i++) {
            unsigned int neighborSphereID = adj_list[i];
            if (!d_visited[neighborSphereID]) {
                border[neighborSphereID] = true;
                atomicAdd(d_border_num, 1);
            }
        }
    }
}

/// UNTESTED
/// G-DBSCAN; density-based h_clustering algorithm. Identifies core, border and noise points in h_clusters.
/// SEARCH STEP
/// min_pts: minimal number of points for a h_cluster
/// radius: proximity radius, points inside can form a h_cluster
static __host__ void gdbscan_construct_graph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts, float radius) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// 2 steps:
    ///     1- compute all adjacent spheres inside radius
    construct_adj_num_start_by_proximity<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
            radius, adj_num, adj_start);

    ///     2- compute adjacency list
    construct_adj_list_by_proximity<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
            radius, adj_num, adj_start, adj_list);
}


/// G-DBSCAN; density-based h_clustering algorithm. Identifies core, border and noise points in h_clusters.
/// min_pts: minimal number of points for a h_cluster
/// radius: proximity radius, points inside can form a h_cluster
/// return h_clusters array of pointers to arrays of different sizes
static __host__ unsigned int* gdbscan_search_graph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    init_sphere_group_gdbscan<<<nBLocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                sphere_data->adj_num, sphere_data->sphere_group, minPts);

    unsigned int h_clusters * cluster_search_BFS(nSpheres,
                                        sphere_data->adj_num,
                                        sphere_data->adj_start,
                                        sphere_data->adj_list,
                                        sphere_data->sphere_group) {

    unsigned int cluster_num = h_clusters[0][0];
    unsigned int * sphere_num_in_cluster = (unsigned int *);
    for (size_t i = 0; i < h_clusters[0][0]) { // expect low number of clusters, 1 most of the time, maybe 2-3 otherwise.
        sphere_num_in_cluster = h_clusters[i][0];
    }    

    for (size_t i = 0; i < h_clusters[0][0]) {
        free(h_clusters[i]);
    }
    free(h_clusters);
}

/// @} gpu_cuda
