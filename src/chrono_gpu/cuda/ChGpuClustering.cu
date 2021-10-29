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

#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/cuda/ChCudaMathUtils.cuh"
#include "chrono_gpu/cuda/ChGpuHelpers.cuh"
#include "chrono_gpu/cuda/ChGpu_SMC.cuh"
#include "chrono_gpu/cuda/ChGpuClustering.cuh"

namespace chrono {
namespace gpu {

/// ClusterSearchBFSKernel: GPU part of Breadth First Search (BFS) algorithm
/// Visits all border spheres (d_borders) in parallel, finds neighbors
/// Neighbors set in d_borders, visited on next call
/// Call until no border spheres i.e. nothing true in d_borders
static __global__ void ClusterSearchBFSKernel(unsigned int nSpheres,
                        unsigned int * adj_num,
                        unsigned int * adj_start,
                        unsigned int * adj_list,
                        bool * d_borders, 
                        bool * d_visited) {

    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    if (mySphereID < nSpheres) { 
        if (d_borders[mySphereID]) {
            d_visited[mySphereID] = true;
            d_borders[mySphereID] = false;

            unsigned int start = adj_start[mySphereID];
            unsigned int end = adj_start[mySphereID] + adj_num[mySphereID];
            for (size_t i = start; i < end; i++) {
                unsigned int neighborSphereID = adj_list[i];
                if (!d_visited[neighborSphereID]) {
                    d_borders[neighborSphereID] = true;
                }
            }
        }
    }
}

/// computes h_cluster by breadth first search
/// h_clusters[0][0] -> number of pointers/h_clusters
/// h_clusters[M][0] -> size of the Mth h_cluster
/// h_clusters[M][N] -> Nth point in Mth h_cluster
/// return pointer to h_clusters: array of pointers to arrays of variable length
static __host__ unsigned int ** ClusterSearchBFS(unsigned int nSpheres,
                        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                        unsigned int* adj_num,
                        unsigned int* adj_start,
                        unsigned int* adj_list,
                        SPHERE_GROUP* sphere_group) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// CLUSTERS REPRESENTATION IN MEMORY
    /// clusters, array of points to arrays of variable lengths, with lengths at [0]
    unsigned int * h_cluster;
    unsigned int h_cluster_num = 0;
    unsigned int ** h_clusters;
    h_clusters = (unsigned int **)malloc(sizeof(*h_clusters) * (nSpheres+1)); // at worst, there will be nSpheres h_clusters
    h_clusters[0] = (unsigned int *)malloc(sizeof(**h_clusters)); // number of h_clusters at h_clusters[0][0]
    h_clusters[0][0] = h_cluster_num;


    unsigned int * d_border_num; // number of remaining border vertices to search in BFS_kernel
    unsigned int * d_in_volume_num; // number of spheres inside the volume
    unsigned int * h_border_num = (unsigned int *)malloc(sizeof(*h_border_num)); // number of remaining border vertices to search in BFS_kernl
    unsigned int * h_in_volume_num = (unsigned int *)malloc(sizeof(*h_in_volume_num)); // number of remaining border vertices to search in BFS_kernl
    gpuErrchk(cudaMalloc((void**)&d_border_num, sizeof(*d_border_num)));
    gpuErrchk(cudaMalloc((void**)&d_in_volume_num, sizeof(*d_in_volume_num)));
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    bool * d_borders; // [mySphereID] -> is vertex a border?
    bool * d_visited; // [mySphereID] -> was vertex d_visited during BFS_kernel?
    bool * h_visited; // [mySphereID] -> host of d_visited
    bool * h_searched; // [mySphereID] -> was vertex searched before?
    bool * d_in_volume; // [mySphereID] -> is particle inside the volume?
    gpuErrchk(cudaMalloc((void**)&d_borders, sizeof(*d_borders) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_visited, sizeof(*d_visited) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_in_volume, sizeof(*d_in_volume) * nSpheres));
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    h_visited = (bool *)calloc(sizeof(*h_visited), nSpheres);
    h_searched = (bool *)calloc(sizeof(*h_searched), nSpheres);
    SPHERE_GROUP h_current_group;

    for (size_t i = 0; i < nSpheres; i++) {
        h_current_group = sphere_group[i];
        /// find the next h_cluster, at the first sphere not yet searched.
        if ((!h_searched[i]) && (h_current_group == SPHERE_GROUP::CORE)) {
            cudaMemset(d_borders, false, sizeof(*d_borders) * nSpheres);
            cudaMemset(d_visited, false, sizeof(*d_visited) * nSpheres);
            cudaMemset(d_in_volume, false, sizeof(*d_in_volume) * nSpheres);
            cudaMemset(&d_borders[i], true, sizeof(*d_borders));
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            h_cluster_num++;

            // visit all spheres connected to sphere i in parallel
            do {
                ClusterSearchBFSKernel<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                adj_num, adj_start, adj_list,
                d_borders, d_visited);
                gpuErrchk(cudaPeekAtLastError());
                gpuErrchk(cudaDeviceSynchronize());
                // Allocate temporary storage
                void *d_temp_storage = NULL;
                size_t temp_storage_bytes = 0;
                // Determine temporary device storage requirements
                cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, d_borders, d_border_num, nSpheres);
                // find and visit border points, establishing the cluster
                gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
                gpuErrchk(cudaPeekAtLastError());
                gpuErrchk(cudaDeviceSynchronize());

                cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, d_borders, d_border_num, nSpheres);
                cudaMemcpy(h_border_num, d_border_num, sizeof(*d_border_num), cudaMemcpyDeviceToHost);
                gpuErrchk(cudaFree(d_temp_storage));
                gpuErrchk(cudaPeekAtLastError());
                gpuErrchk(cudaDeviceSynchronize());
            } while ((*h_border_num) > 0);

            // find if any sphere was tagged in the VOLUME group i.e. in the volume.
            FindVolumeCluster<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                                           d_visited, d_in_volume,
                                           sphere_data->sphere_group);
            // Sum number of particles in d_in_volume into h_in_volume_num
            void *d_temp_storage = NULL;
            size_t temp_storage_bytes = 0;
            // Determine temporary device storage requirements
            cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, d_in_volume, d_in_volume_num, nSpheres);
            // find and visit border points, establishing the cluster
            gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, d_in_volume, d_in_volume_num, nSpheres);
            cudaMemcpy(h_in_volume_num, d_in_volume_num, sizeof(*d_in_volume_num), cudaMemcpyDeviceToHost);
            gpuErrchk(cudaFree(d_temp_storage));
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());

            cudaMemcpy(h_visited, d_visited, sizeof(*d_visited) * nSpheres, cudaMemcpyDeviceToHost);
            h_cluster = (unsigned int *)calloc((nSpheres + 1), sizeof(*h_cluster));
            // h_cluster[0] is its size, so it length nSpheres + 1
            assert(h_cluster[0] == 0);

            unsigned int cluster_index;
            // if any sphere is in the volume, the cluster is VOLUME
            // gets overwritten by the biggest cluster in GdbscanSearchGraph
            if (*h_in_volume_num > 0) {
                cluster_index = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::VOLUME);
            }  else {
                cluster_index = h_cluster_num + static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::START);
            }

            for (size_t j = 0; j < nSpheres; j++) {
                if (h_visited[j]) {
                    if ((sphere_group[j] != SPHERE_GROUP::CORE) && (sphere_group[j] != SPHERE_GROUP::VOLUME)) {
                        sphere_group[j] = SPHERE_GROUP::BORDER;
                    }

                    h_searched[j] = true;
                    h_cluster[++h_cluster[0]] = j;
                    sphere_data->sphere_cluster[j] = cluster_index;
                }
            }
            h_clusters[h_cluster_num] = h_cluster;
            assert(h_cluster[0] <= nSpheres);
            assert(h_clusters[h_cluster_num][0] <= nSpheres);
            h_clusters[0][0] = h_cluster_num; // h_clusters size is number of clusters + 1 cause it includes its length 
            h_searched[i] = true;
        }
    }

    h_clusters = (unsigned int **)realloc(h_clusters, sizeof(*h_clusters) * (h_clusters[0][0]+1));

    free(h_visited);
    free(h_searched);
    free(h_border_num);
    free(h_in_volume_num);
    gpuErrchk(cudaFree(d_borders));
    gpuErrchk(cudaFree(d_border_num));
    gpuErrchk(cudaFree(d_in_volume_num));
    gpuErrchk(cudaFree(d_in_volume));
    gpuErrchk(cudaFree(d_visited));
    assert(h_clusters[0][0] == h_cluster_num);
    assert(h_clusters[0][0] <= nSpheres);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    return(h_clusters);
}

/// UNTESTED
/// counts number of adjacent spheres by proximity.
static __global__ void ComputeAdjNumByProximity(
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
    double3 mySphere_pos_global, otherSphere_pos_global, distance;

    mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID],
            sphere_data->sphere_local_pos_Y[mySphereID], sphere_data->sphere_local_pos_Z[mySphereID]);
    mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, mySphere_pos_local, gran_params));

    if (mySphereID < nSpheres) {
    /// find all spheres inside the radius around mySphere
        for (size_t i = 0; (i < nSpheres); i++) {
            otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i],
            sphere_data->sphere_local_pos_Y[i], sphere_data->sphere_local_pos_Z[i]);
            otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD,
                otherSphere_pos_local, gran_params));
            distance = mySphere_pos_global - otherSphere_pos_global;
            if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
                sphere_data->adj_num[i]++;
                sphere_data->adj_num[mySphereID]++;
            }
        }
    }
}

/// UNTESTED
/// computes adj_list from proximity using known adj_num and adj_start
static __global__ void ComputeAdjListByProximity(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, float radius) {
    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int otherSphereID;

    unsigned int thisSD = blockIdx.x; // sphere_pos_local is relative to this SD
    unsigned int mySD = sphere_data->sphere_owner_SDs[mySphereID];
    unsigned int otherSD;

    unsigned int vertex_start = sphere_data->adj_start[mySphereID];
    unsigned int adjacency_num = 0;

    int3 mySphere_pos_local, otherSphere_pos_local;
    double3 mySphere_pos_global, otherSphere_pos_global, distance;

    mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID],
            sphere_data->sphere_local_pos_Y[mySphereID], sphere_data->sphere_local_pos_Z[mySphereID]);
    mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, mySphere_pos_local, gran_params));

    if (mySphereID < nSpheres) {
        for (size_t i = 0; (i < nSpheres); i++) {
            otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i],
            sphere_data->sphere_local_pos_Y[i], sphere_data->sphere_local_pos_Z[i]);
            otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD,
                otherSphere_pos_local, gran_params));
            distance = mySphere_pos_global - otherSphere_pos_global;
            if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
                sphere_data->adj_list[vertex_start + adjacency_num] = i;
            }
        }
    }
    assert(adjacency_num == sphere_data->adj_num[mySphereID]);
}


}  // namespace gpu
}  // namespace chrono



/// UNTESTED
/// G-DBSCAN; density-based h_clustering algorithm. Identifies core, border and noise points in h_clusters.
/// min_pts: minimal number of points for a h_cluster
/// radius: proximity radius, points inside can form a h_cluster
__host__ void GdbscanConstructGraph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts, float radius) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// compute all adjacent spheres inside radius
    ComputeAdjNumByProximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
            radius);
    /// compute all adjacent spheres inside radius
    ComputeAdjStartFromAdjNum(nSpheres, sphere_data->adj_num, sphere_data->adj_start);
    /// compute adjacency list
    ComputeAdjListByProximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
            radius);
}

/// G-DBSCAN; density-based h_clustering algorithm. Identifies core, border and noise points in h_clusters.<
/// Searches using a parallel Breadth-First search
/// min_pts: minimal number of points for a cluster
__host__ void GdbscanSearchGraph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    unsigned int ** h_clusters = ClusterSearchBFS(nSpheres, sphere_data,
                                        sphere_data->adj_num,
                                        sphere_data->adj_start,
                                        sphere_data->adj_list,
                                        sphere_data->sphere_group);
    unsigned int cluster_num = h_clusters[0][0];
    unsigned int sphere_num_in_cluster;
    unsigned int biggest_cluster_size = 1;
    unsigned int biggest_cluster_id = 1;

    if (cluster_num > 0) {
        // find biggest cluster id
        for (size_t i = 1; i < cluster_num; i++) {// expect low number of clusters, 1 most of the time, maybe 2-3 otherwise.
            sphere_num_in_cluster = h_clusters[i][0];
            assert(sphere_num_in_cluster <= nSpheres);
            if (sphere_num_in_cluster > biggest_cluster_size) {
                biggest_cluster_size = sphere_num_in_cluster;
                biggest_cluster_id = i;
            }
        }    

        // tag each sphere to a biggest cluster_group
        // for (size_t i = 1; i < cluster_num; i++) {// expect low number of clusters, 1 most of the time, maybe 2-3 otherwise.
        sphere_num_in_cluster = h_clusters[biggest_cluster_id][0];
        assert(h_clusters[biggest_cluster_id][0] <= nSpheres);
        for (size_t j = 1; j < (sphere_num_in_cluster + 1); j++) {
            unsigned int CurrentSphereID = h_clusters[biggest_cluster_id][j];
            sphere_data->sphere_cluster[CurrentSphereID] = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::GROUND);
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
    }

    GdbscanFinalClusterFromGroup<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres, sphere_data->sphere_cluster, sphere_data->sphere_group);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    for (size_t i = 0; i < cluster_num; i++) {
        free(h_clusters[i]);
    }
    free(h_clusters);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}


