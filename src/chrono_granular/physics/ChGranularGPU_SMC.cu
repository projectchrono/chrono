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

#include <cmath>
#include <numeric>

#include "chrono_granular/physics/ChGranularGPU_SMC.cuh"
#include "chrono_granular/utils/ChGranularUtilities.h"
#include "chrono/core/ChTimer.h"

namespace chrono {
namespace granular {

__host__ double ChSystemGranularSMC::get_max_z() const {
    size_t nSpheres = sphere_local_pos_Z.size();
    std::vector<int64_t> sphere_pos_global_Z;
    sphere_pos_global_Z.resize(nSpheres);
    for (size_t index = 0; index < nSpheres; index++) {
        unsigned int ownerSD = sphere_data->sphere_owner_SDs[index];
        int3 sphere_pos_local =
            make_int3(sphere_data->sphere_local_pos_X[index], sphere_data->sphere_local_pos_Y[index],
                      sphere_data->sphere_local_pos_Z[index]);
        sphere_pos_global_Z[index] = convertPosLocalToGlobal(ownerSD, sphere_pos_local, gran_params).z;
    }

    double max_z_SU = *(std::max_element(sphere_pos_global_Z.begin(), sphere_pos_global_Z.end()));
    double max_z_UU = max_z_SU * LENGTH_SU2UU;

    return max_z_UU;
}

// Reset broadphase data structures
void ChSystemGranularSMC::resetBroadphaseInformation() {
    // Set all the offsets to zero
    gpuErrchk(cudaMemset(SD_NumSpheresTouching.data(), 0, SD_NumSpheresTouching.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(SD_SphereCompositeOffsets.data(), 0, SD_SphereCompositeOffsets.size() * sizeof(unsigned int)));
    // For each SD, all the spheres touching that SD should have their ID be NULL_GRANULAR_ID
    gpuErrchk(cudaMemset(spheres_in_SD_composite.data(), NULL_GRANULAR_ID,
                         spheres_in_SD_composite.size() * sizeof(unsigned int)));
    gpuErrchk(cudaDeviceSynchronize());
}

// Reset sphere acceleration data structures
void ChSystemGranularSMC::resetSphereAccelerations() {
    // cache past acceleration data
    if (time_integrator == GRAN_TIME_INTEGRATOR::CHUNG) {
        gpuErrchk(cudaMemcpy(sphere_acc_X_old.data(), sphere_acc_X.data(), nSpheres * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        gpuErrchk(cudaMemcpy(sphere_acc_Y_old.data(), sphere_acc_Y.data(), nSpheres * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        gpuErrchk(cudaMemcpy(sphere_acc_Z_old.data(), sphere_acc_Z.data(), nSpheres * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        // if we have multistep AND friction, cache old alphas
        if (gran_params->friction_mode != FRICTIONLESS) {
            gpuErrchk(cudaMemcpy(sphere_ang_acc_X_old.data(), sphere_ang_acc_X.data(), nSpheres * sizeof(float),
                                 cudaMemcpyDeviceToDevice));
            gpuErrchk(cudaMemcpy(sphere_ang_acc_Y_old.data(), sphere_ang_acc_Y.data(), nSpheres * sizeof(float),
                                 cudaMemcpyDeviceToDevice));
            gpuErrchk(cudaMemcpy(sphere_ang_acc_Z_old.data(), sphere_ang_acc_Z.data(), nSpheres * sizeof(float),
                                 cudaMemcpyDeviceToDevice));
        }
        gpuErrchk(cudaDeviceSynchronize());
    }

    // reset current accelerations to zero to zero
    gpuErrchk(cudaMemset(sphere_acc_X.data(), 0, nSpheres * sizeof(float)));
    gpuErrchk(cudaMemset(sphere_acc_Y.data(), 0, nSpheres * sizeof(float)));
    gpuErrchk(cudaMemset(sphere_acc_Z.data(), 0, nSpheres * sizeof(float)));

    // reset torques to zero, if applicable
    if (gran_params->friction_mode != FRICTIONLESS) {
        gpuErrchk(cudaMemset(sphere_ang_acc_X.data(), 0, nSpheres * sizeof(float)));
        gpuErrchk(cudaMemset(sphere_ang_acc_Y.data(), 0, nSpheres * sizeof(float)));
        gpuErrchk(cudaMemset(sphere_ang_acc_Z.data(), 0, nSpheres * sizeof(float)));
    }
}

__global__ void compute_absv(const unsigned int nSpheres,
                             const float* velX,
                             const float* velY,
                             const float* velZ,
                             float* d_absv) {
    unsigned int my_sphere = blockIdx.x * blockDim.x + threadIdx.x;
    if (my_sphere < nSpheres) {
        float v[3] = {velX[my_sphere], velY[my_sphere], velZ[my_sphere]};
        d_absv[my_sphere] = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    }
}

__host__ float ChSystemGranularSMC::get_max_vel() const {
    float* d_absv;
    float* d_max_vel;
    float h_max_vel;
    gpuErrchk(cudaMalloc(&d_absv, nSpheres * sizeof(float)));
    gpuErrchk(cudaMalloc(&d_max_vel, sizeof(float)));

    compute_absv<<<(nSpheres + 255) / 256, 256>>>(nSpheres, pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data(), d_absv);

    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, d_absv, d_max_vel, nSpheres);
    gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
    cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, d_absv, d_max_vel, nSpheres);
    gpuErrchk(cudaMemcpy(&h_max_vel, d_max_vel, sizeof(float), cudaMemcpyDeviceToHost));

    gpuErrchk(cudaFree(d_absv));
    gpuErrchk(cudaFree(d_max_vel));

    return h_max_vel;
}

__host__ int3 ChSystemGranularSMC::getSDTripletFromID(unsigned int SD_ID) const {
    return SDIDTriplet(SD_ID, gran_params);
}
/// Sort sphere positions by subdomain id
/// Occurs entirely on host, not intended to be efficient
/// ONLY DO AT BEGINNING OF SIMULATION
__host__ void ChSystemGranularSMC::defragment_initial_positions() {
    INFO_PRINTF("Starting defrag run!\n");
    ChTimer<> timer;
    timer.start();

    // key and value pointers
    std::vector<unsigned int, cudallocator<unsigned int>> sphere_ids;

    // load sphere indices
    sphere_ids.resize(nSpheres);
    std::iota(sphere_ids.begin(), sphere_ids.end(), 0);

    // sort sphere ids by owner SD
    std::sort(sphere_ids.begin(), sphere_ids.end(),
              [&](std::size_t i, std::size_t j) { return sphere_owner_SDs.at(i) < sphere_owner_SDs.at(j); });

    std::vector<int, cudallocator<int>> sphere_pos_x_tmp;
    std::vector<int, cudallocator<int>> sphere_pos_y_tmp;
    std::vector<int, cudallocator<int>> sphere_pos_z_tmp;
    std::vector<not_stupid_bool, cudallocator<not_stupid_bool>> sphere_fixed_tmp;
    std::vector<unsigned int, cudallocator<unsigned int>> sphere_owner_SDs_tmp;

    sphere_pos_x_tmp.resize(nSpheres);
    sphere_pos_y_tmp.resize(nSpheres);
    sphere_pos_z_tmp.resize(nSpheres);
    sphere_fixed_tmp.resize(nSpheres);
    sphere_owner_SDs_tmp.resize(nSpheres);

    // reorder values into new sorted
    for (unsigned int i = 0; i < nSpheres; i++) {
        sphere_pos_x_tmp.at(i) = sphere_local_pos_X.at(sphere_ids.at(i));
        sphere_pos_y_tmp.at(i) = sphere_local_pos_Y.at(sphere_ids.at(i));
        sphere_pos_z_tmp.at(i) = sphere_local_pos_Z.at(sphere_ids.at(i));
        sphere_fixed_tmp.at(i) = sphere_fixed.at(sphere_ids.at(i));
        sphere_owner_SDs_tmp.at(i) = sphere_owner_SDs.at(sphere_ids.at(i));
    }

    // swap into the correct data structures
    sphere_local_pos_X.swap(sphere_pos_x_tmp);
    sphere_local_pos_Y.swap(sphere_pos_y_tmp);
    sphere_local_pos_Z.swap(sphere_pos_z_tmp);
    sphere_fixed.swap(sphere_fixed_tmp);
    sphere_owner_SDs.swap(sphere_owner_SDs_tmp);

    timer.stop();
    INFO_PRINTF("finished defrag run in %f seconds!\n", timer.GetTimeSeconds());
}
__host__ void ChSystemGranularSMC::setupSphereDataStructures() {
    // Each fills user_sphere_positions with positions to be copied
    if (user_sphere_positions.size() == 0) {
        printf("ERROR: no sphere positions given!\n");
        exit(1);
    }

    nSpheres = (unsigned int)user_sphere_positions.size();
    INFO_PRINTF("%u balls added!\n", nSpheres);
    gran_params->nSpheres = nSpheres;

    TRACK_VECTOR_RESIZE(sphere_owner_SDs, nSpheres, "sphere_owner_SDs", NULL_GRANULAR_ID);

    // Allocate space for new bodies
    TRACK_VECTOR_RESIZE(sphere_local_pos_X, nSpheres, "sphere_local_pos_X", 0);
    TRACK_VECTOR_RESIZE(sphere_local_pos_Y, nSpheres, "sphere_local_pos_Y", 0);
    TRACK_VECTOR_RESIZE(sphere_local_pos_Z, nSpheres, "sphere_local_pos_Z", 0);

    TRACK_VECTOR_RESIZE(sphere_fixed, nSpheres, "sphere_fixed", 0);

    // temporarily store global positions as 64-bit, discard as soon as local positions are loaded
    {
        bool user_provided_fixed = user_sphere_fixed.size() != 0;
        if (user_provided_fixed && user_sphere_fixed.size() != nSpheres) {
            printf("Provided fixity array does not match provided particle positions\n");
            exit(1);
        }

        std::vector<int64_t, cudallocator<int64_t>> sphere_global_pos_X;
        std::vector<int64_t, cudallocator<int64_t>> sphere_global_pos_Y;
        std::vector<int64_t, cudallocator<int64_t>> sphere_global_pos_Z;

        sphere_global_pos_X.resize(nSpheres);
        sphere_global_pos_Y.resize(nSpheres);
        sphere_global_pos_Z.resize(nSpheres);

        // Copy from array of structs to 3 arrays
        for (unsigned int i = 0; i < nSpheres; i++) {
            auto vec = user_sphere_positions.at(i);
            // cast to double, convert to SU, then cast to int64_t
            sphere_global_pos_X.at(i) = (int64_t)((double)vec.x() / LENGTH_SU2UU);
            sphere_global_pos_Y.at(i) = (int64_t)((double)vec.y() / LENGTH_SU2UU);
            sphere_global_pos_Z.at(i) = (int64_t)((double)vec.z() / LENGTH_SU2UU);

            // Convert to not_stupid_bool
            sphere_fixed.at(i) = (not_stupid_bool)((user_provided_fixed) ? user_sphere_fixed[i] : false);
        }

        packSphereDataPointers();
        // Figure our the number of blocks that need to be launched to cover the box
        unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
        initializeLocalPositions<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
            sphere_data, sphere_global_pos_X.data(), sphere_global_pos_Y.data(), sphere_global_pos_Z.data(), nSpheres,
            gran_params);

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
        defragment_initial_positions();
    }

    TRACK_VECTOR_RESIZE(pos_X_dt, nSpheres, "pos_X_dt", 0);
    TRACK_VECTOR_RESIZE(pos_Y_dt, nSpheres, "pos_Y_dt", 0);
    TRACK_VECTOR_RESIZE(pos_Z_dt, nSpheres, "pos_Z_dt", 0);
    TRACK_VECTOR_RESIZE(sphere_acc_X, nSpheres, "sphere_acc_X", 0);
    TRACK_VECTOR_RESIZE(sphere_acc_Y, nSpheres, "sphere_acc_Y", 0);
    TRACK_VECTOR_RESIZE(sphere_acc_Z, nSpheres, "sphere_acc_Z", 0);

    // NOTE that this will get resized again later, this is just the first estimate
    TRACK_VECTOR_RESIZE(spheres_in_SD_composite, 2 * nSpheres, "spheres_in_SD_composite", NULL_GRANULAR_ID);

    if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
        // add rotational DOFs
        TRACK_VECTOR_RESIZE(sphere_Omega_X, nSpheres, "sphere_Omega_X", 0);
        TRACK_VECTOR_RESIZE(sphere_Omega_Y, nSpheres, "sphere_Omega_Y", 0);
        TRACK_VECTOR_RESIZE(sphere_Omega_Z, nSpheres, "sphere_Omega_Z", 0);

        // add torques
        TRACK_VECTOR_RESIZE(sphere_ang_acc_X, nSpheres, "sphere_ang_acc_X", 0);
        TRACK_VECTOR_RESIZE(sphere_ang_acc_Y, nSpheres, "sphere_ang_acc_Y", 0);
        TRACK_VECTOR_RESIZE(sphere_ang_acc_Z, nSpheres, "sphere_ang_acc_Z", 0);
    }

    if (gran_params->friction_mode == GRAN_FRICTION_MODE::MULTI_STEP ||
        gran_params->friction_mode == GRAN_FRICTION_MODE::SINGLE_STEP) {
        TRACK_VECTOR_RESIZE(contact_partners_map, 12 * nSpheres, "contact_partners_map", NULL_GRANULAR_ID);
        TRACK_VECTOR_RESIZE(contact_active_map, 12 * nSpheres, "contact_active_map", false);
    }
    if (gran_params->friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
        float3 null_history = {0., 0., 0.};
        TRACK_VECTOR_RESIZE(contact_history_map, 12 * nSpheres, "contact_history_map", null_history);
    }

    if (time_integrator == GRAN_TIME_INTEGRATOR::CHUNG) {
        TRACK_VECTOR_RESIZE(sphere_acc_X_old, nSpheres, "sphere_acc_X_old", 0);
        TRACK_VECTOR_RESIZE(sphere_acc_Y_old, nSpheres, "sphere_acc_Y_old", 0);
        TRACK_VECTOR_RESIZE(sphere_acc_Z_old, nSpheres, "sphere_acc_Z_old", 0);

        // friction and multistep means keep old ang acc
        if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
            TRACK_VECTOR_RESIZE(sphere_ang_acc_X_old, nSpheres, "sphere_ang_acc_X_old", 0);
            TRACK_VECTOR_RESIZE(sphere_ang_acc_Y_old, nSpheres, "sphere_ang_acc_Y_old", 0);
            TRACK_VECTOR_RESIZE(sphere_ang_acc_Z_old, nSpheres, "sphere_ang_acc_Z_old", 0);
        }
    }
    // make sure the right pointers are packed
    packSphereDataPointers();
}

__host__ void ChSystemGranularSMC::runSphereBroadphase() {
    METRICS_PRINTF("Resetting broadphase info!\n");

    resetBroadphaseInformation();
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    packSphereDataPointers();

    sphereBroadphase_dryrun<CUDA_THREADS_PER_BLOCK>
        <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, nSpheres, gran_params);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;

    // num spheres in last SD
    unsigned int last_SD_num_spheres = SD_NumSpheresTouching.at(nSDs - 1);

    unsigned int* out_ptr = SD_SphereCompositeOffsets.data();
    unsigned int* in_ptr = SD_NumSpheresTouching.data();

    // copy data into the tmp array
    gpuErrchk(cudaMemcpy(out_ptr, in_ptr, nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));
    cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, nSDs);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // Allocate temporary storage
    gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // Run exclusive prefix sum
    cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, nSDs);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // total number of sphere entries to record
    unsigned int num_entries = out_ptr[nSDs - 1] + in_ptr[nSDs - 1];
    spheres_in_SD_composite.resize(num_entries, NULL_GRANULAR_ID);

    // make sure the DEs pointer is updated
    packSphereDataPointers();

    // printf("first run: num entries is %u, theoretical max is %u\n", num_entries, nSDs * MAX_COUNT_OF_SPHERES_PER_SD);

    // for (unsigned int i = 0; i < nSDs; i++) {
    //     printf("SD %d has offset %u, N %u \n", i, out_ptr[i], in_ptr[i]);
    // }

    // back up the offsets
    // TODO use a cached allocator, CUB provides one
    std::vector<unsigned int, cudallocator<unsigned int>> SD_SphereCompositeOffsets_bak;
    SD_SphereCompositeOffsets_bak.resize(SD_SphereCompositeOffsets.size());
    gpuErrchk(cudaMemcpy(SD_SphereCompositeOffsets_bak.data(), SD_SphereCompositeOffsets.data(),
                         nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    sphereBroadphase<CUDA_THREADS_PER_BLOCK><<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, nSpheres, gran_params);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    //
    // for (unsigned int i = 0; i < nSDs; i++) {
    //     printf("SD %d has offset %u, N %u \n", i, out_ptr[i], in_ptr[i]);
    // }
    //
    // for (unsigned int i = 0; i < num_entries; i++) {
    //     printf("entry %u is %u\n", i, spheres_in_SD_composite[i]);
    // }

    // restore the old offsets
    gpuErrchk(cudaMemcpy(SD_SphereCompositeOffsets.data(), SD_SphereCompositeOffsets_bak.data(),
                         nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));
    gpuErrchk(cudaFree(d_temp_storage));
}

__host__ void ChSystemGranularSMC::updateBCPositions() {
    for (unsigned int i = 0; i < BC_params_list_UU.size(); i++) {
        auto bc_type = BC_type_list.at(i);
        const BC_params_t<float, float3>& params_UU = BC_params_list_UU.at(i);
        BC_params_t<int64_t, int64_t3>& params_SU = BC_params_list_SU.at(i);
        auto offset_function = BC_offset_function_list.at(i);
        setBCOffset(bc_type, params_UU, params_SU, offset_function(elapsedSimTime));
    }

    if (!BD_is_fixed) {
        double3 new_BD_offset = BDOffsetFunction(elapsedSimTime);

        int64_t3 bd_offset_SU = {0, 0, 0};
        bd_offset_SU.x = new_BD_offset.x / LENGTH_SU2UU;
        bd_offset_SU.y = new_BD_offset.y / LENGTH_SU2UU;
        bd_offset_SU.z = new_BD_offset.z / LENGTH_SU2UU;

        int64_t old_frame_X = gran_params->BD_frame_X;
        int64_t old_frame_Y = gran_params->BD_frame_Y;
        int64_t old_frame_Z = gran_params->BD_frame_Z;

        gran_params->BD_frame_X = bd_offset_SU.x + BD_rest_frame_SU.x;
        gran_params->BD_frame_Y = bd_offset_SU.y + BD_rest_frame_SU.y;
        gran_params->BD_frame_Z = bd_offset_SU.z + BD_rest_frame_SU.z;

        unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

        int64_t3 offset_delta = {0, 0, 0};

        // if the frame X increases, the local X should decrease
        offset_delta.x = old_frame_X - gran_params->BD_frame_X;
        offset_delta.y = old_frame_Y - gran_params->BD_frame_Y;
        offset_delta.z = old_frame_Z - gran_params->BD_frame_Z;

        // printf("offset is %lld, %lld, %lld\n", offset_delta.x, offset_delta.y, offset_delta.z);

        packSphereDataPointers();

        applyBDFrameChange<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(offset_delta, sphere_data, nSpheres, gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
    }
}

__host__ double ChSystemGranularSMC::advance_simulation(float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Settling simulation loop.
    float duration_SU = duration / TIME_SU2UU;
    unsigned int nsteps = std::round(duration_SU / stepSize_SU);

    METRICS_PRINTF("advancing by %f at timestep %f, %u timesteps at approx user timestep %f\n", duration_SU,
                   stepSize_SU, nsteps, duration / nsteps);
    float time_elapsed_SU = 0;  // time elapsed in this advance call

    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (; time_elapsed_SU < stepSize_SU * nsteps; time_elapsed_SU += stepSize_SU) {
        updateBCPositions();

        runSphereBroadphase();
        packSphereDataPointers();

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        resetSphereAccelerations();
        resetBCForces();

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

        METRICS_PRINTF("Starting integrateSpheres!\n");
        integrateSpheres<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
            updateFrictionData<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }

        elapsedSimTime += stepSize_SU * TIME_SU2UU;  // Advance current time
    }

    return time_elapsed_SU * TIME_SU2UU;  // return elapsed UU time
}
}  // namespace granular
}  // namespace chrono
