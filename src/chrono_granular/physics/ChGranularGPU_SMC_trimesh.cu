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
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

// NOTE: DON'T MOVE OR CHANGE THESE THREE LINES
#include "chrono/ChConfig.h"
#undef CHRONO_HAS_SSE
#undef CHRONO_HAS_AVX

#include "chrono_granular/physics/ChGranularGPU_SMC.cuh"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_granular/ChGranularDefines.h"

// these define things that mess with cub
#include "chrono_granular/physics/ChGranularCollision.cuh"
#include "chrono_granular/physics/ChGranularBoxTriangle.cuh"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"

#include "chrono_granular/physics/ChGranularGPU_SMC_trimesh.cuh"

namespace chrono {
namespace granular {

void ChSystemGranularSMC_trimesh::resetTriangleForces() {
    gpuErrchk(cudaMemset(meshSoup->generalizedForcesPerFamily, 0, 6 * meshSoup->numTriangleFamilies * sizeof(float)));
}
// Reset triangle broadphase data structures
void ChSystemGranularSMC_trimesh::resetTriangleBroadphaseInformation() {
    gpuErrchk(cudaMemset(SD_numTrianglesTouching.data(), 0, SD_numTrianglesTouching.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(SD_TriangleCompositeOffsets.data(), NULL_GRANULAR_ID,
                         SD_TriangleCompositeOffsets.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(triangles_in_SD_composite.data(), NULL_GRANULAR_ID,
                         triangles_in_SD_composite.size() * sizeof(unsigned int)));
}

__host__ void ChSystemGranularSMC_trimesh::runTriangleBroadphase() {
    METRICS_PRINTF("Resetting broadphase info!\n");

    packSphereDataPointers();

    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_NumSDsTouching;
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsCompositeOffsets;

    Triangle_NumSDsTouching.resize(meshSoup->nTrianglesInSoup, 0);
    Triangle_SDsCompositeOffsets.resize(meshSoup->nTrianglesInSoup, 0);

    const int nthreads = CUDA_THREADS_PER_BLOCK;
    int nblocks = (meshSoup->nTrianglesInSoup + nthreads - 1) / nthreads;
    triangleSoup_CountSDsTouched<<<nblocks, nthreads>>>(meshSoup, Triangle_NumSDsTouching.data(), gran_params,
                                                        tri_params);

    unsigned int numTriangles = meshSoup->nTrianglesInSoup;

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    unsigned int num_entries = 0;

    // do prefix scan
    {
        void* d_temp_storage = NULL;
        size_t temp_storage_bytes = 0;
        unsigned int* out_ptr = Triangle_SDsCompositeOffsets.data();
        unsigned int* in_ptr = Triangle_NumSDsTouching.data();

        // copy data into the tmp array
        gpuErrchk(cudaMemcpy(out_ptr, in_ptr, numTriangles * sizeof(unsigned int), cudaMemcpyDeviceToDevice));
        cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, numTriangles);

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
        // Allocate temporary storage
        gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
        // Run exclusive prefix sum
        cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, numTriangles);

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaFree(d_temp_storage));
        num_entries = out_ptr[numTriangles - 1] + in_ptr[numTriangles - 1];
        // printf("%u entries total!\n", num_entries);
    }

    // for (unsigned int i = 0; i < Triangle_NumSDsTouching.size(); i++) {
    //     printf("Triangle %u touches %u SDs, offset is %u\n", i, Triangle_NumSDsTouching[i],
    //            Triangle_SDsCompositeOffsets[i]);
    // }
    // total number of sphere entries to record
    // to be sorted
    // produced by sort
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_SDs_out;
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_TriIDs_out;

    Triangle_SDsComposite_SDs_out.resize(num_entries, NULL_GRANULAR_ID);
    Triangle_SDsComposite_TriIDs_out.resize(num_entries, NULL_GRANULAR_ID);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // sort key-value where the key is SD id, value is triangle ID in composite array
    {
        // tmp values used for sort
        std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_SDs;
        std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_TriIDs;
        Triangle_SDsComposite_SDs.resize(num_entries, NULL_GRANULAR_ID);
        Triangle_SDsComposite_TriIDs.resize(num_entries, NULL_GRANULAR_ID);

        // printf("first run: num entries is %u, theoretical max is %u\n", num_entries, nSDs *
        // MAX_TRIANGLE_COUNT_PER_SD);
        triangleSoup_StoreSDsTouched<<<nblocks, nthreads>>>(
            meshSoup, Triangle_NumSDsTouching.data(), Triangle_SDsCompositeOffsets.data(),
            Triangle_SDsComposite_SDs.data(), Triangle_SDsComposite_TriIDs.data(), gran_params, tri_params);
        unsigned int num_items = num_entries;
        unsigned int* d_keys_in = Triangle_SDsComposite_SDs.data();
        unsigned int* d_keys_out = Triangle_SDsComposite_SDs_out.data();
        unsigned int* d_values_in = Triangle_SDsComposite_TriIDs.data();
        unsigned int* d_values_out = Triangle_SDsComposite_TriIDs_out.data();

        gpuErrchk(cudaDeviceSynchronize());

        // Determine temporary device storage requirements
        void* d_temp_storage = NULL;
        size_t temp_storage_bytes = 0;
        // Run sorting operation
        // pass null, cub tells us what it needs
        cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in,
                                        d_values_out, num_items);
        gpuErrchk(cudaDeviceSynchronize());

        // Allocate temporary storage
        gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
        gpuErrchk(cudaDeviceSynchronize());

        cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in,
                                        d_values_out, num_items);
        gpuErrchk(cudaDeviceSynchronize());

        gpuErrchk(cudaFree(d_temp_storage));
    }
    // now Triangle_SDsComposite_SDs_out has an ordered list of active SDs, with one entry for each triangle
    //
    // for (unsigned int i = 0; i < Triangle_SDsComposite_TriIDs_out.size(); i++) {
    //     printf("composite entry %u is SD %u, triangle %u\n", i, Triangle_SDsComposite_SDs_out[i],
    //            Triangle_SDsComposite_TriIDs_out[i]);
    // }

    // offsets of each SD in composite array
    std::vector<unsigned int, cudallocator<unsigned int>> SD_TriangleCompositeOffsets_tmp;
    std::vector<unsigned int, cudallocator<unsigned int>> SD_numTrianglesTouching_tmp;

    SD_TriangleCompositeOffsets_tmp.resize(nSDs, NULL_GRANULAR_ID);
    SD_numTrianglesTouching_tmp.resize(nSDs, 0);

    // if there are triangle-sd contacts, sweep through them, otherwise just move on
    if (Triangle_SDsComposite_SDs_out.size() > 0) {
        // get first active SD
        unsigned int prev_SD = Triangle_SDsComposite_SDs_out.at(0);
        // first SD has offset 0
        SD_TriangleCompositeOffsets.at(prev_SD) = 0;
        // number of triangles in current SD
        unsigned int curr_count = 0;
        // offset to current SD
        unsigned int curr_offset = 0;

        // simultaneously do a prefix scan and a store, but on host
        // TODO optimize and test
        // TODO can we do this with a weird prefix scan operation?
        for (unsigned int i = 0; i < Triangle_SDsComposite_SDs_out.size(); i++) {
            unsigned int curr_SD = Triangle_SDsComposite_SDs_out.at(i);
            // this is the start of a new SD
            if (prev_SD != curr_SD) {
                // printf("change! SD %u has curr count %u, offset %u, prev is %u\n",curr_count, curr_offset,  );
                // store the count for this SD
                SD_numTrianglesTouching.at(prev_SD) = curr_count;
                // reset count
                curr_count = 0;
                // set this SD to have offset after the previous one ends
                SD_TriangleCompositeOffsets.at(curr_SD) = curr_offset;
            }
            curr_count++;
            curr_offset++;
            // now this is the active SD to check against
            prev_SD = curr_SD;
        }

        // right now we only store counts at the end of a streak, so we need to store the last streak
        // TODO is this always right???
        SD_numTrianglesTouching.at(prev_SD) = curr_count;
    }

    // for (unsigned int i = 0; i < SD_numTrianglesTouching.size(); i++) {
    //     printf("tri count index %u is usual %u, other %u\n", i, SD_numTrianglesTouching[i],
    //            SD_numTrianglesTouching_tmp[i]);
    // }
    //
    // for (unsigned int i = 0; i < SD_TriangleCompositeOffsets.size(); i++) {
    //     printf("offset index %u is usual %u, other %u\n", i, SD_TriangleCompositeOffsets[i],
    //            SD_TriangleCompositeOffsets_tmp[i]);
    // }
    //
    // for (unsigned int i = 0; i < triangles_in_SD_composite.size(); i++) {
    //     printf("composite index %u is usual %u, other %u\n", i, triangles_in_SD_composite[i],
    //            Triangle_SDsComposite_TriIDs_out[i]);
    // }

    triangles_in_SD_composite.resize(Triangle_SDsComposite_TriIDs_out.size());

    // copy the composite data to the primary location
    gpuErrchk(cudaMemcpy(triangles_in_SD_composite.data(), Triangle_SDsComposite_TriIDs_out.data(),
                         Triangle_SDsComposite_TriIDs_out.size() * sizeof(unsigned int), cudaMemcpyDeviceToDevice));

    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}
__host__ double ChSystemGranularSMC_trimesh::advance_simulation(float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Settling simulation loop.
    float duration_SU = duration / TIME_SU2UU;
    unsigned int nsteps = std::round(duration_SU / stepSize_SU);

    packSphereDataPointers();
    // cudaMemAdvise(gran_params, sizeof(*gran_params), cudaMemAdviseSetReadMostly, dev_ID);

    METRICS_PRINTF("advancing by %f at timestep %f, %u timesteps at approx user timestep %f\n", duration_SU,
                   stepSize_SU, nsteps, duration / nsteps);

    METRICS_PRINTF("Starting Main Simulation loop!\n");

    float time_elapsed_SU = 0;  // time elapsed in this call (SU)
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (; time_elapsed_SU < stepSize_SU * nsteps; time_elapsed_SU += stepSize_SU) {
        updateBCPositions();

        resetSphereAccelerations();
        resetBCForces();
        if (meshSoup->nTrianglesInSoup != 0 && mesh_collision_enabled) {
            resetTriangleForces();
            resetTriangleBroadphaseInformation();
        }

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        METRICS_PRINTF("Starting computeSphereForces!\n");

        if (gran_params->friction_mode == FRICTIONLESS) {
            // Compute sphere-sphere forces
            computeSphereForces_frictionless<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(), BC_params_list_SU.size());
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        } else if (gran_params->friction_mode == SINGLE_STEP || gran_params->friction_mode == MULTI_STEP) {
            // figure out who is contacting
            determineContactPairs<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(sphere_data, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());

            computeSphereContactForces<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(), BC_params_list_SU.size(),
                nSpheres);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }

        if (meshSoup->nTrianglesInSoup != 0 && mesh_collision_enabled) {
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            runTriangleBroadphase();
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup->numTriangleFamilies != 0 && mesh_collision_enabled) {
            // TODO please do not use a template here
            // triangle labels come after BC labels numerically
            unsigned int triangleFamilyHistmapOffset = gran_params->nSpheres + 1 + BC_params_list_SU.size() + 1;
            // compute sphere-triangle forces
            interactionTerrain_TriangleSoup<CUDA_THREADS_PER_BLOCK><<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                meshSoup, sphere_data, triangles_in_SD_composite.data(), SD_numTrianglesTouching.data(),
                SD_TriangleCompositeOffsets.data(), gran_params, tri_params, triangleFamilyHistmapOffset);
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        METRICS_PRINTF("Resetting broadphase info!\n");

        resetBroadphaseInformation();

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        METRICS_PRINTF("Starting integrateSpheres!\n");
        integrateSpheres<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
            updateFrictionData<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }

        runSphereBroadphase();

        packSphereDataPointers();

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        elapsedSimTime += stepSize_SU * TIME_SU2UU;  // Advance current time
    }

    return time_elapsed_SU * TIME_SU2UU;  // return elapsed UU time
}
}  // namespace granular
}  // namespace chrono
