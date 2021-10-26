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

#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/cuda/ChCudaMathUtils.cuh"
#include "chrono_gpu/cuda/ChGpuHelpers.cuh"
#include "chrono_gpu/cuda/ChGpuClustering.cuh"

namespace chrono {
namespace gpu {

// /// UNTESTED
// /// counts number of adjacent spheres by proximity.
// static __global__ void construct_adj_num_start_by_proximity(
//                         ChSystemGpu_impl::GranSphereDataPtr sphere_data,
//                         ChSystemGpu_impl::GranParamsPtr gran_params,
//                         unsigned int nSpheres, float radius) {

//     // my sphere ID, we're using a 1D thread->sphere map
//     unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
//     unsigned int otherSphereID;

//     unsigned int thisSD = blockIdx.x; // sphere_pos_local is relative to this SD
//     unsigned int mySD = sphere_data->sphere_owner_SDs[mySphereID];
//     unsigned int otherSD;

//     int3 mySphere_pos_local, otherSphere_pos_local;
//     double3 mySphere_pos_global, otherSphere_pos_local, distance;

//     mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID], 
//             sphere_data->sphere_local_pos_Y[mySphereID], sphere_data->sphere_local_pos_Z[mySphereID]);
//     mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, mySphere_pos_local, gran_params));   

//     if (mySphereID < nSpheres) {
//     /// find all spheres inside the radius around mySphere
//         for (size_t i = 0; (i < nSpheres); i++) {
//             otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i], 
//             sphere_data->sphere_local_pos_Y[i], sphere_data->sphere_local_pos_Z[i]);
//             otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, otherSphere_pos_local, gran_params));   
//             distance = mySphere_pos_global - otherSphere_pos_global;
//                 if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
//                     adj_num[i]++;
//                     adj_num[mySphereID]++;
//                 }
//             adj_start[i+1]++;
//             /// perform an inclusive sum. start index of subsequent vertices depend on all previous indices.
//             void * d_temp_storage = NULL;
//             size_t temp_storage_bytesize = 0;
//             cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytesize, adj_start + i, adj_start + i, nSpheres - i);
//             gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytesize));
//             cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytesize, adj_start + i, adj_start + i, nSpheres - i);
//         }
//     }
// }

// /// UNTESTED
// /// computes adj_list from proximity using known adj_num and adj_start
// static __global__ void construct_adj_list_by_proximity(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
//                                            ChSystemGpu_impl::GranParamsPtr gran_params,
//                                            unsigned int nSpheres, float radius,
//                                            unsigned int* adj_num,
//                                            unsigned int* adj_start,
//                                            unsigned int* adj_list) {
//     // my sphere ID, we're using a 1D thread->sphere map
//     unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
//     unsigned int otherSphereID;

//     unsigned int thisSD = blockIdx.x; // sphere_pos_local is relative to this SD
//     unsigned int mySD = sphere_data->sphere_owner_SDs[mySphereID];
//     unsigned int otherSD;

//     unsigned int vertex_start = adj_start[mySphereID];
//     unsigned int adjacency_num = 0;

//     int3 mySphere_pos_local, otherSphere_pos_local;
//     double3 mySphere_pos_global, otherSphere_pos_global;

//     mySphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[mySphereID], 
//             sphere_data->sphere_local_pos_Y[mySphereID], sphere_data->sphere_local_pos_Z[mySphereID]);
//     mySphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, mySphere_pos_local, gran_params));   

//     if (mySphereID < nSpheres) {
//         for (size_t i = 0; (i < nSpheres); i++) {
//             otherSphere_pos_local = make_int3(sphere_data->sphere_local_pos_X[i], 
//             sphere_data->sphere_local_pos_Y[i], sphere_data->sphere_local_pos_Z[i]);
//             otherSphere_pos_global = int64_t3_to_double3(convertPosLocalToGlobal(thisSD, otherSphere_pos_local, gran_params));   
//             distance = mySphere_pos_global - otherSphere_pos_global;
//             if ((i != mySphereID) && (Dot(distance, distance) < (radius*radius))) {
//                 adj_list[vertex_start + adjacency_num] = i;
//             }
//         }                  
//     }
//     assert(adjacency_num == vertex_adjacency_num[mySphereID]);
// }

/// All spheres with > min_pts contacts are core, other points maybe be border points.
static __global__ void init_sphere_group(unsigned int nSpheres,
                                           unsigned int* adj_num,
                                           SPHERE_GROUP* sphere_group,
                                           unsigned int min_pts) {
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    sphere_group[mySphereID] = adj_num[mySphereID] > min_pts ? SPHERE_GROUP::CORE : SPHERE_GROUP::NOISE;
}

/// computes h_cluster by breadth first search
static __global__ void cluster_search_BFS_kernel(unsigned int nSpheres,
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
/// return pointer to h_clusters: array of pointers to arrays of variable length
static __host__ unsigned int ** cluster_search_BFS(unsigned int nSpheres,
                        ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                        unsigned int* adj_num,
                        unsigned int* adj_start,
                        unsigned int* adj_list,
                        SPHERE_GROUP* sphere_group) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// CLUSTERS REPRESENTATION IN MEMORY
    /// ALTERNATIVE 1:
    /// clusters, array of points to arrays of variable lengths, with lengths at [0]
    unsigned int * h_cluster;
    unsigned int h_cluster_num = 0;
    unsigned int ** h_clusters;
    h_clusters = (unsigned int **)malloc(sizeof(*h_clusters) * (nSpheres+1)); // at worst, there will be nSpheres h_clusters
    h_clusters[0] = (unsigned int *)malloc(sizeof(**h_clusters)); // number of h_clusters at h_clusters[0][0]
    h_clusters[0][0] = h_cluster_num;
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
    unsigned int * d_adj_num, * d_adj_start, * d_adj_list;
    bool * d_borders; // [mySphereID] -> is vertex a border?
    bool * d_visited; // [mySphereID] -> was vertex d_visited during BFS_kernel?
    bool * h_visited; // [mySphereID] -> host of d_visited
    bool * h_searched; // [mySphereID] -> was vertex searched before?
    unsigned int * d_border_num; // number of remaining border vertices to search in BFS_kernel
    unsigned int * h_border_num = (unsigned int *)malloc(sizeof(*h_border_num)); // number of remaining border vertices to search in BFS_kernl
    gpuErrchk(cudaMalloc((void**)&d_adj_num, sizeof(*d_adj_num) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_adj_start, sizeof(*d_adj_start) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_adj_list, sizeof(*d_adj_list) * nSpheres * MAX_SPHERES_TOUCHED_BY_SPHERE));
    cudaMemcpy(d_adj_num, adj_num, sizeof(*d_adj_num) * nSpheres, cudaMemcpyHostToDevice);
    cudaMemcpy(d_adj_start, adj_start, sizeof(*d_adj_start) * nSpheres, cudaMemcpyHostToDevice);
    cudaMemcpy(d_adj_list, adj_list, sizeof(*d_adj_list) * nSpheres, cudaMemcpyHostToDevice);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    gpuErrchk(cudaMalloc((void**)&d_borders, sizeof(*d_borders) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_visited, sizeof(*d_visited) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_border_num, sizeof(*d_border_num)));
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    h_visited = (bool *)calloc(sizeof(*h_visited), nSpheres);
    h_searched = (bool *)calloc(sizeof(*h_searched), nSpheres);
    SPHERE_GROUP h_current_group;

    for (size_t i = 0; i < nSpheres; i++) {
        h_current_group = sphere_group[i];
        /// find the next h_cluster, at the first sphere not yet searched.
        // printf("i %d %d \n", i, nSpheres);
        if ((!h_searched[i]) && (h_current_group == SPHERE_GROUP::CORE)) {
            cudaMemset(d_borders, false, sizeof(*d_borders) * nSpheres);
            cudaMemset(d_visited, false, sizeof(*d_visited) * nSpheres);
            cudaMemset(&d_borders[i], true, sizeof(*d_borders));
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            h_cluster_num++;
            printf("h_cluster_num %d \n", h_cluster_num);

            do {
                cluster_search_BFS_kernel<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                d_adj_num, d_adj_start, d_adj_list,
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
                // printf("h_border_num %d \n", *h_border_num);
            } while ((*h_border_num) > 0);

            cudaMemcpy(h_visited, d_visited, sizeof(*d_visited)* nSpheres, cudaMemcpyDeviceToHost);
            h_cluster = (unsigned int *)calloc((nSpheres + 1), sizeof(*h_cluster));
            assert(h_cluster[0] == 0);
            // h_cluster[0] is its size, so it length nSpheres + 1
            for (size_t j = 0; j < nSpheres; j++) {
                if (h_visited[j]) {
                    h_searched[j] = true;
                    h_cluster[++h_cluster[0]] = j;
                    sphere_data->sphere_cluster[j] = h_cluster_num;
                    if (sphere_group[j] != SPHERE_GROUP::CORE) {
                        sphere_group[j] = SPHERE_GROUP::BORDER;
                    }
                }
            }
            // assert(h_cluster[0] <= nSpheres);
            // h_clusters[h_cluster_num] = (unsigned int *)realloc(h_cluster, sizeof(*h_cluster) * (h_cluster[0] + 1));;
            h_clusters[h_cluster_num] = h_cluster;
            assert(h_cluster[0] <= nSpheres);
            printf("spheres in cluster %d\n", h_cluster[0]);
            printf("spheres in cluster %d\n", h_clusters[h_cluster_num][0]);
            assert(h_clusters[h_cluster_num][0] <= nSpheres);
            h_clusters[0][0] = h_cluster_num; // h_clusters size is number of clusters + 1 cause it includes its length 
            h_searched[i] = true;
        }
    }
    printf("out");
    getchar();

    h_clusters = (unsigned int **)realloc(h_clusters, sizeof(*h_clusters) * (h_clusters[0][0]+1));

    free(h_visited);
    free(h_searched);
    free(h_border_num);
    gpuErrchk(cudaFree(d_borders));
    gpuErrchk(cudaFree(d_border_num));
    gpuErrchk(cudaFree(d_visited));
    gpuErrchk(cudaFree(d_adj_num));
    gpuErrchk(cudaFree(d_adj_list));
    gpuErrchk(cudaFree(d_adj_start));
    assert(h_clusters[0][0] == h_cluster_num);
    assert(h_clusters[0][0] <= nSpheres);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    return(h_clusters);
}

// /// UNTESTED
// /// G-DBSCAN; density-based h_clustering algorithm. Identifies core, border and noise points in h_clusters.
// /// SEARCH STEP
// /// min_pts: minimal number of points for a h_cluster
// /// radius: proximity radius, points inside can form a h_cluster
// static __host__ void gdbscan_construct_graph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
//                                            ChSystemGpu_impl::GranParamsPtr gran_params,
//                                            unsigned int nSpheres, size_t min_pts, float radius) {
//     unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

//     /// 2 steps:
//     ///     1- compute all adjacent spheres inside radius
//     construct_adj_num_start_by_proximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
//             radius, adj_num, adj_start);

//     ///     2- compute adjacency list
//     construct_adj_list_by_proximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, nSpheres,
//             radius, adj_num, adj_start, adj_list);
// }

// static __global__ void update_cluster_group(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
//                                            ChSystemGpu_impl::GranParamsPtr gran_params,
//                                            unsigned int sphere_num_in_cluster,
//                                            unsigned int cluster_id,
//                                            unsigned int * cluster) {
//     unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
//     for (size_t i = 0; i < sphere_num_in_cluster; i++) { 
//         if (cluster[i] == mySphereID) {
//             sphere_data->sphere_cluster = cluster_id;
//         }
//     }
// }

}  // namespace gpu
}  // namespace chrono


/// G-DBSCAN; density-based h_clustering algorithm. Identifies core, border and noise points in h_clusters.
/// min_pts: minimal number of points for a cluster
__host__ void gdbscan_search_graph(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                           ChSystemGpu_impl::GranParamsPtr gran_params,
                                           unsigned int nSpheres, size_t min_pts) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    chrono::gpu::init_sphere_group<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                sphere_data->adj_num, sphere_data->sphere_group, min_pts);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    init_sphere_cluster<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, gran_params, static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::GROUND));
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    unsigned int ** h_clusters = cluster_search_BFS(nSpheres, sphere_data,
                                        sphere_data->adj_num,
                                        sphere_data->adj_start,
                                        sphere_data->adj_list,
                                        sphere_data->sphere_group);
    unsigned int * d_cluster;
    unsigned int cluster_num = h_clusters[0][0];
    unsigned int sphere_num_in_cluster;
    unsigned int biggest_cluster_size = 1;
    unsigned int biggest_cluster_id = 1;

    printf("cluster_num %d", cluster_num);
    if (cluster_num > 0) {
        // find biggest cluster id
        for (size_t i = 1; i < cluster_num; i++) {// expect low number of clusters, 1 most of the time, maybe 2-3 otherwise.
            // printf("cluster_i %d\n", i);
            sphere_num_in_cluster = h_clusters[i][0];
            assert(sphere_num_in_cluster <= nSpheres);
            if (sphere_num_in_cluster > biggest_cluster_size) {
                biggest_cluster_size = sphere_num_in_cluster;
                biggest_cluster_id = i;
            }
        }    

        // tag each sphere to a biggest cluster_group
        // for (size_t i = 1; i < cluster_num; i++) {// expect low number of clusters, 1 most of the time, maybe 2-3 otherwise.
            // printf("cluster_i %d\n", i);

        sphere_num_in_cluster = h_clusters[biggest_cluster_id][0];
        assert(h_clusters[biggest_cluster_id][0] <= nSpheres);
        for (size_t j = 1; j < (sphere_num_in_cluster + 1); j++) {
            unsigned int CurrentSphereID = h_clusters[biggest_cluster_id][j];
            sphere_data->sphere_cluster[CurrentSphereID] = 0;
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
    }

    for (size_t i = 0; i < cluster_num; i++) {
        free(h_clusters[i]);
    }
    free(h_clusters);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}

