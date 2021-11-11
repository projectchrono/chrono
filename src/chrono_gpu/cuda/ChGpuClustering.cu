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

/// Identifies all clusters by breadth first search and outputs
/// Returns h_clusters: array of pointers to arrays of variable length
/// h_clusters[0][0] -> number of pointers/cluster in h_clusters
/// h_clusters[M][0] -> size of the Mth cluster
/// h_clusters[M][N] -> Nth point in Mth cluster
/// at worst, there will be nSpheres h_clusters
static __host__ unsigned int ** ClusterSearchBFS(unsigned int nSpheres,
                                                 ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                                 unsigned int* adj_num,
                                                 unsigned int* adj_offset,
                                                 unsigned int* adj_list,
                                                 SPHERE_TYPE* sphere_type) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    cudaMemset(sphere_data->sphere_cluster,
               static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::GROUND),
               sizeof(*sphere_data->sphere_cluster) * nSpheres);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    unsigned int ** h_clusters; /* all identified clusters */
    unsigned int * h_cluster;
    unsigned int h_cluster_num = 0;
    h_clusters = (unsigned int **)malloc(sizeof(*h_clusters) * (nSpheres+1));
    h_clusters[0] = (unsigned int *)malloc(sizeof(**h_clusters));
    h_clusters[0][0] = h_cluster_num;  // number of h_clusters

    // border_num: number of remaining border vertices to search in BFS_kernel
    unsigned int * d_border_num;
    gpuErrchk(cudaMalloc((void**)&d_border_num, sizeof(*d_border_num)));
    unsigned int * h_border_num = (unsigned int *)malloc(sizeof(*h_border_num)); 
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    bool * d_borders;  // [mySphereID] -> is vertex a border?
    bool * d_visited;  // [mySphereID] -> was vertex d_visited in BFS_kernel?
    bool * h_visited;  // [mySphereID] -> host of d_visited
    bool * h_searched;  // [mySphereID] -> was vertex searched before?
    gpuErrchk(cudaMalloc((void**)&d_borders, sizeof(*d_borders) * nSpheres));
    gpuErrchk(cudaMalloc((void**)&d_visited, sizeof(*d_visited) * nSpheres));
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    h_visited = (bool *)calloc(sizeof(*h_visited), nSpheres);
    h_searched = (bool *)calloc(sizeof(*h_searched), nSpheres);
    SPHERE_TYPE h_current_type;

    // Search for clusters at every sphere.
    for (size_t i = 0; i < nSpheres; i++) {
        h_current_type = sphere_type[i];
        /// find the next h_cluster, at the first sphere not yet searched
        /// all spheres connected to it are part of this cluster
        if ((!h_searched[i]) && (h_current_type == SPHERE_TYPE::CORE)) {
            cudaMemset(d_borders, false, sizeof(*d_borders) * nSpheres);
            cudaMemset(d_visited, false, sizeof(*d_visited) * nSpheres);
            cudaMemset(&d_borders[i], true, sizeof(*d_borders));
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            h_cluster_num++;

            // visit all spheres connected to sphere i in parallel
            do {
                // find and visit border points, establishing the cluster
                ClusterSearchBFSKernel<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                    nSpheres,
                    adj_num,
                    adj_offset,
                    adj_list,
                    d_borders,
                    d_visited);
                gpuErrchk(cudaPeekAtLastError());
                gpuErrchk(cudaDeviceSynchronize());
                // Allocate temporary storage
                void *d_temp_storage = NULL;
                size_t temp_storage_bytes = 0;
                // Determine temporary device storage requirements
                cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes,
                                       d_borders, d_border_num, nSpheres);
                gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
                gpuErrchk(cudaPeekAtLastError());
                gpuErrchk(cudaDeviceSynchronize());
                // sum d_borders to find remainder borders to visit
                cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes,
                                       d_borders, d_border_num, nSpheres);
                cudaMemcpy(h_border_num, d_border_num,
                           sizeof(*d_border_num), cudaMemcpyDeviceToHost);
                gpuErrchk(cudaFree(d_temp_storage));
                gpuErrchk(cudaPeekAtLastError());
                gpuErrchk(cudaDeviceSynchronize());
            } while ((*h_border_num) > 0);

            cudaMemcpy(h_visited, d_visited, sizeof(*d_visited) * nSpheres,
                       cudaMemcpyDeviceToHost);
            h_cluster = (unsigned int *)calloc((nSpheres + 1), sizeof(*h_cluster));
            // h_cluster[0] is its size, so it length nSpheres + 1
            assert(h_cluster[0] == 0);

            // First pass of setting sphere_cluster
            // GROUND and VOLUME clusters come later
            unsigned int cluster_index = h_cluster_num + static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::START);

            for (size_t j = 0; j < nSpheres; j++) {
                if (h_visited[j]) {
                    if ((sphere_type[j] != chrono::gpu::SPHERE_TYPE::CORE) &&
                        (sphere_type[j] != chrono::gpu::SPHERE_TYPE::VOLUME)) {
                        sphere_type[j] = chrono::gpu::SPHERE_TYPE::BORDER;
                    }

                    h_searched[j] = true;
                    h_cluster[++h_cluster[0]] = j;
                    sphere_data->sphere_cluster[j] = cluster_index;
                }
            }
            h_clusters[h_cluster_num] = h_cluster;
            assert(h_cluster[0] <= nSpheres);
            assert(h_clusters[h_cluster_num][0] <= nSpheres);
            h_clusters[0][0] = h_cluster_num;
            h_searched[i] = true;
        }
    }
    size_t h_clusters_bytesize = sizeof(*h_clusters) * (h_clusters[0][0]+1);
    h_clusters = (unsigned int **)realloc(h_clusters, h_clusters_bytesize);

    free(h_visited);
    free(h_searched);
    free(h_border_num);
    gpuErrchk(cudaFree(d_borders));
    gpuErrchk(cudaFree(d_border_num));
    gpuErrchk(cudaFree(d_visited));
    assert(h_clusters[0][0] == h_cluster_num);
    assert(h_clusters[0][0] <= nSpheres);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    return(h_clusters);
}

/// Uses sphere_contact_map to construct adjacency lists for clustering
__host__ void ConstructGraphByContact(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                      ChSystemGpu_impl::GranParamsPtr gran_params,
                                      unsigned int nSpheres) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    ComputeAdjNumByContact<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                                gran_params,
                                                                nSpheres);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    ComputeAdjOffsetFromAdjNum(nSpheres,
                              sphere_data->adj_num,
                              sphere_data->adj_offset);
    
    ComputeAdjListByContact<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                                 gran_params,
                                                                 nSpheres);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}

/// UNTESTED
/// G-DBSCAN; density-based h_clustering algorithm.
/// Identifies core, border and noise points in h_clusters.
/// min_pts: minimal number of points for a h_cluster
/// radius: proximity radius, points inside can form a h_cluster
__host__ void ConstructGraphByProximity(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                        ChSystemGpu_impl::GranParamsPtr gran_params,
                                        unsigned int nSpheres,
                                        size_t min_pts,
                                        float radius) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// compute all adjacent spheres inside radius
    ComputeAdjNumByProximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                                  gran_params,
                                                                  nSpheres,
                                                                  radius);
    /// compute all adjacent spheres inside radius
    ComputeAdjOffsetFromAdjNum(nSpheres,
                              sphere_data->adj_num,
                              sphere_data->adj_offset);
    /// compute adjacency list
    ComputeAdjListByProximity<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                                   gran_params,
                                                                   nSpheres,
                                                                   radius);
}

/// identify ground cluster by spheres lower than a certain plane
__host__ void IdentifyGroundClusterByLowest(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                            ChSystemGpu_impl::GranParamsPtr gran_params,
                                            unsigned int ** h_clusters,
                                            unsigned int nSpheres) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    unsigned int cluster_num = h_clusters[0][0];
    unsigned int cluster_index;

    unsigned int * h_below_num = (unsigned int *)malloc(sizeof(*h_below_num));
    bool * d_below;
    gpuErrchk(cudaMalloc((void**)&d_below, sizeof(*d_below) * nSpheres));
    unsigned int * d_below_num;
    gpuErrchk(cudaMalloc((void**)&d_below_num, sizeof(*d_below_num)));
    unsigned int ground_cluster = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::GROUND);
    ///　Find ALL clusters with any sphere center below plane box_z + 1*sphere_radius
    for (size_t i = 1; i < (cluster_num + 1); i++) {
        cudaMemset(d_below, false, sizeof(*d_below) * nSpheres);

        // if any sphere is in VOLUME cluster, cluster index should be VOLUME
        if (sphere_data->sphere_cluster[h_clusters[i][1]] == static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::VOLUME)) {
            cluster_index = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::VOLUME);
        } else {
            cluster_index = i + static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::START);
        }
        AreSpheresBelowZLim<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                                 gran_params,
                                                                 nSpheres,
                                                                 d_below,
                                                                 cluster_index,
                                                                 gran_params->ground_z_lim);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        // Sum number of particles in below z_lim
        void *d_temp_storage = NULL;
        size_t temp_storage_bytes = 0;
        // Determine temporary device storage requirements
        cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes,
                               d_below, d_below_num, nSpheres);
        // find and visit border points, establishing the cluster
        gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes,
                               d_below, d_below_num, nSpheres);
        cudaMemcpy(h_below_num, d_below_num,
                   sizeof(*d_below_num), cudaMemcpyDeviceToHost);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaFree(d_temp_storage));

        if ((*h_below_num) > 0) {
            SwitchClusterIndex<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                                    gran_params,
                                                                    nSpheres,
                                                                    cluster_index,
                                                                    ground_cluster);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
    }

    gpuErrchk(cudaFree(d_below));
    gpuErrchk(cudaFree(d_below_num));
    free(h_below_num);
}

__host__ void FreeClusters(unsigned int ** h_clusters) {
    unsigned int cluster_num = h_clusters[0][0];
    for (size_t i = 0; i < (cluster_num + 1); i++) {
        free(h_clusters[i]);
    }
    free(h_clusters);
}

/// Finds cluster in h_clusters with most sphres (biggest cluster)
/// sets sphere_cluster of all those spheres to GROUND
__host__ void IdentifyGroundClusterByBiggest(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                             ChSystemGpu_impl::GranParamsPtr gran_params,
                                             unsigned int ** h_clusters,
                                             unsigned int nSpheres) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    unsigned int ground_cluster = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::GROUND);
    unsigned int cluster_num = h_clusters[0][0];
    unsigned int sphere_num_in_cluster = 1;
    unsigned int biggest_cluster_size = 1;
    unsigned int biggest_cluster;

    // find which cluster is the biggest
    for (size_t i = 1; i < (cluster_num + 1); i++) {
        sphere_num_in_cluster = h_clusters[i][0];
        assert(sphere_num_in_cluster <= nSpheres);
        if (sphere_num_in_cluster > biggest_cluster_size) {
            biggest_cluster_size = sphere_num_in_cluster;
            if (sphere_data->sphere_cluster[h_clusters[i][1]] == static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::VOLUME)) {
                biggest_cluster = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::VOLUME);
            } else {
                biggest_cluster = i + static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::START);
            }
        }
    }

    SwitchClusterIndex<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                            gran_params,
                                                            nSpheres,
                                                            biggest_cluster,
                                                            ground_cluster);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}

__host__ void IdentifyVolumeCluster(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                    ChSystemGpu_impl::GranParamsPtr gran_params,
                                    unsigned int nSpheres,
                                    unsigned int ** h_clusters) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// sets sphere_type to VOLUME if inside a mesh
    /// must be set AFTER GdbscanInitSphereType,
    /// and AFTER interactionGranMat_TriangleSoup,
    /// which is AFTER AdvanceSimulation
    SetVolumeSphereType<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                             nSpheres);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    // in_volume_num: number of spheres inside the volume mesh
    unsigned int * d_in_volume_num;
    unsigned int * h_in_volume_num = (unsigned int *)malloc(sizeof(*h_in_volume_num));
    gpuErrchk(cudaMalloc((void**)&d_in_volume_num, sizeof(*d_in_volume_num)));
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    bool * d_in_volume;  // [mySphereID] -> is particle inside the volume?
    gpuErrchk(cudaMalloc((void**)&d_in_volume, sizeof(*d_in_volume) * nSpheres));
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    unsigned int cluster_num = h_clusters[0][0];

    for (size_t i = 1; i < (cluster_num + 1); i++) {
        cudaMemset(d_in_volume, false, sizeof(*d_in_volume) * nSpheres);
        unsigned int * h_cluster = h_clusters[i];
        unsigned int cluster_index = i + static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::START);

        // find if any sphere in cluster was tagged in the VOLUME type
        FindVolumeTypeInCluster<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data,
                                                               nSpheres,
                                                               d_in_volume,
                                                               cluster_index);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        // Sum number of particles in d_in_volume into h_in_volume_num
        void *d_temp_storage = NULL;
        size_t temp_storage_bytes = 0;
        // Determine temporary device storage requirements
        cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes,
                               d_in_volume, d_in_volume_num, nSpheres);
        // find and visit border points, establishing the cluster
        gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes,
                               d_in_volume, d_in_volume_num, nSpheres);
        cudaMemcpy(h_in_volume_num, d_in_volume_num,
                   sizeof(*d_in_volume_num), cudaMemcpyDeviceToHost);
        gpuErrchk(cudaFree(d_temp_storage));
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        // any sphere of cluster is type  VOLUME -> sphere_cluster becomes VOLUME
        // UNLESS it is GROUND
        if (*h_in_volume_num > 0) {
            unsigned int volume_cluster = static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::VOLUME);
            unsigned int sphere_num_in_cluster = h_cluster[0];
            for (size_t j = 1; j < (sphere_num_in_cluster + 1); j++) {
                if (sphere_data->sphere_cluster[h_cluster[j]] != static_cast<unsigned int>(chrono::gpu::CLUSTER_INDEX::GROUND)) {
                    sphere_data->sphere_cluster[h_cluster[j]] = volume_cluster;
                }
            }
        }
    }
    gpuErrchk(cudaFree(d_in_volume_num));
    gpuErrchk(cudaFree(d_in_volume));
    free(h_in_volume_num);
}


/// Finds the GROUND cluster using cluster_ground_method in gran_params
__host__ void IdentifyGroundCluster(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                    ChSystemGpu_impl::GranParamsPtr gran_params,
                                    unsigned int nSpheres,
                                    unsigned int ** h_clusters) {
    unsigned int cluster_num = h_clusters[0][0];

    if (cluster_num > 0) {
        switch (gran_params->cluster_ground_method) {
            case chrono::gpu::CLUSTER_GROUND_METHOD::NONE: {
                break;
            }
            case chrono::gpu::CLUSTER_GROUND_METHOD::BIGGEST: {
                IdentifyGroundClusterByBiggest(sphere_data, gran_params, h_clusters, nSpheres);
                break;
            }
            case chrono::gpu::CLUSTER_GROUND_METHOD::LOWEST: {
                IdentifyGroundClusterByLowest(sphere_data, gran_params, h_clusters, nSpheres);
                break;
            }
            default: {
                break;
            }
        }
    }
}

/// Search part of G-DBSCAN; density-based h_clustering algorithm.
/// Identifies core, border and noise points in h_clusters.
/// Searches using a parallel Breadth-First search
/// min_pts: minimal number of points for a cluster
__host__ unsigned int ** GdbscanSearchGraphByBFS(ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                 ChSystemGpu_impl::GranParamsPtr gran_params,
                                 unsigned int nSpheres,
                                 size_t min_pts) {
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    /// sphere_type is CORE if neighbors_num > min_pts else NOISE
    GdbscanInitSphereType<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                                                               sphere_data->adj_num,
                                                               sphere_data->sphere_type,
                                                               gran_params->gdbscan_min_pts);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    unsigned int ** h_clusters = ClusterSearchBFS(nSpheres, sphere_data,
                                                  sphere_data->adj_num,
                                                  sphere_data->adj_offset,
                                                  sphere_data->adj_list,
                                                  sphere_data->sphere_type);
    unsigned int cluster_num = h_clusters[0][0];

    GdbscanFinalClusterFromType<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(nSpheres,
                                                                     sphere_data->sphere_cluster,
                                                                     sphere_data->sphere_type);
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
    return(h_clusters);
}


