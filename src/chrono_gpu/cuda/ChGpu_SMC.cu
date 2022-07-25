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
// Authors: Conlain Kelly, Nic Olsen, Ruochun Zhang, Dan Negrut
// =============================================================================

#include <cmath>
#include <numeric>
#include <fstream>

#include "chrono_gpu/cuda/ChGpu_SMC.cuh"
#include "chrono_gpu/utils/ChGpuUtilities.h"

namespace chrono {
namespace gpu {

__host__ float ChSystemGpu_impl::computeArray3SquaredSum(std::vector<float, cudallocator<float>>& arrX,
                                                         std::vector<float, cudallocator<float>>& arrY,
                                                         std::vector<float, cudallocator<float>>& arrZ,
                                                         size_t nSpheres) {
    const unsigned int threadsPerBlock = 1024;
    unsigned int nBlocks = (nSpheres + threadsPerBlock - 1) / threadsPerBlock;
    elementalArray3Squared<float><<<nBlocks, threadsPerBlock>>>(sphere_data->sphere_stats_buffer, arrX.data(),
                                                                arrY.data(), arrZ.data(), nSpheres);
    gpuErrchk(cudaDeviceSynchronize());

    // Use CUB to reduce. And put the reduced result at the last element of sphere_stats_buffer array.
    size_t temp_storage_bytes = 0;
    cub::DeviceReduce::Sum(NULL, temp_storage_bytes, sphere_data->sphere_stats_buffer,
                           sphere_data->sphere_stats_buffer + nSpheres, nSpheres);
    void* d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    cub::DeviceReduce::Sum(d_scratch_space, temp_storage_bytes, sphere_data->sphere_stats_buffer,
                           sphere_data->sphere_stats_buffer + nSpheres, nSpheres);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    return *(sphere_data->sphere_stats_buffer + nSpheres);
}

__host__ double ChSystemGpu_impl::GetMaxParticleZ(bool getMax) {
    size_t nSpheres = sphere_local_pos_Z.size();
    if (nSpheres == 0)
        CHGPU_ERROR("ERROR! 0 particle in system! Please call this method after Initialize().\n");

    const unsigned int threadsPerBlock = 1024;
    unsigned int nBlocks = (nSpheres + threadsPerBlock - 1) / threadsPerBlock;
    elementalZLocalToGlobal<<<nBlocks, threadsPerBlock>>>(sphere_data->sphere_stats_buffer, sphere_data, nSpheres,
                                                          gran_params);
    gpuErrchk(cudaDeviceSynchronize());

    // Use CUB to find the max or min Z.
    size_t temp_storage_bytes = 0;
    if (getMax) {
        cub::DeviceReduce::Max(NULL, temp_storage_bytes, sphere_data->sphere_stats_buffer,
                               sphere_data->sphere_stats_buffer + nSpheres, nSpheres);
        void* d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
        cub::DeviceReduce::Max(d_scratch_space, temp_storage_bytes, sphere_data->sphere_stats_buffer,
                               sphere_data->sphere_stats_buffer + nSpheres, nSpheres);
    } else {
        cub::DeviceReduce::Min(NULL, temp_storage_bytes, sphere_data->sphere_stats_buffer,
                               sphere_data->sphere_stats_buffer + nSpheres, nSpheres);
        void* d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
        cub::DeviceReduce::Min(d_scratch_space, temp_storage_bytes, sphere_data->sphere_stats_buffer,
                               sphere_data->sphere_stats_buffer + nSpheres, nSpheres);
    }
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    return *(sphere_data->sphere_stats_buffer + nSpheres);
}

__host__ unsigned int ChSystemGpu_impl::GetNumParticleAboveZ(float ZValue) {
    size_t nSpheres = sphere_local_pos_Z.size();
    if (nSpheres == 0)
        CHGPU_ERROR("ERROR! 0 particle in system! Please call this method after Initialize().\n");

    const unsigned int threadsPerBlock = 1024;
    unsigned int nBlocks = (nSpheres + threadsPerBlock - 1) / threadsPerBlock;
    elementalZAboveValue<<<nBlocks, threadsPerBlock>>>(sphere_data->sphere_stats_buffer_int, sphere_data, nSpheres,
                                                       gran_params, ZValue);
    gpuErrchk(cudaDeviceSynchronize());

    // Use CUB to find the max or min Z.
    size_t temp_storage_bytes = 0;
    cub::DeviceReduce::Sum(NULL, temp_storage_bytes, sphere_data->sphere_stats_buffer_int,
                           sphere_data->sphere_stats_buffer_int + nSpheres, nSpheres);
    void* d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    cub::DeviceReduce::Sum(d_scratch_space, temp_storage_bytes, sphere_data->sphere_stats_buffer_int,
                           sphere_data->sphere_stats_buffer_int + nSpheres, nSpheres);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    return *(sphere_data->sphere_stats_buffer_int + nSpheres);
}

__host__ unsigned int ChSystemGpu_impl::GetNumParticleAboveX(float XValue) {
    size_t nSpheres = sphere_local_pos_X.size();
    if (nSpheres == 0)
        CHGPU_ERROR("ERROR! 0 particle in system! Please call this method after Initialize().\n");

    const unsigned int threadsPerBlock = 1024;
    unsigned int nBlocks = (nSpheres + threadsPerBlock - 1) / threadsPerBlock;
    elementalXAboveValue<<<nBlocks, threadsPerBlock>>>(sphere_data->sphere_stats_buffer_int, sphere_data, nSpheres,
                                                       gran_params, XValue);
    gpuErrchk(cudaDeviceSynchronize());

    // Use CUB to find the max or min X.
    size_t temp_storage_bytes = 0;
    cub::DeviceReduce::Sum(NULL, temp_storage_bytes, sphere_data->sphere_stats_buffer_int,
                           sphere_data->sphere_stats_buffer_int + nSpheres, nSpheres);
    void* d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    cub::DeviceReduce::Sum(d_scratch_space, temp_storage_bytes, sphere_data->sphere_stats_buffer_int,
                           sphere_data->sphere_stats_buffer_int + nSpheres, nSpheres);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    return *(sphere_data->sphere_stats_buffer_int + nSpheres);
}

// Reset broadphase data structures
void ChSystemGpu_impl::resetBroadphaseInformation() {
    // Set all the offsets to zero
    gpuErrchk(cudaMemset(SD_NumSpheresTouching.data(), 0, SD_NumSpheresTouching.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(SD_SphereCompositeOffsets.data(), 0, SD_SphereCompositeOffsets.size() * sizeof(unsigned int)));
    // For each SD, all the spheres touching that SD should have their ID be NULL_CHGPU_ID
    gpuErrchk(cudaMemset(spheres_in_SD_composite.data(), NULL_CHGPU_ID,
                         spheres_in_SD_composite.size() * sizeof(unsigned int)));
    gpuErrchk(cudaDeviceSynchronize());
}

// Reset sphere acceleration data structures
void ChSystemGpu_impl::resetSphereAccelerations() {
    // cache past acceleration data
    if (time_integrator == CHGPU_TIME_INTEGRATOR::CHUNG) {
        gpuErrchk(cudaMemcpy(sphere_acc_X_old.data(), sphere_acc_X.data(), nSpheres * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        gpuErrchk(cudaMemcpy(sphere_acc_Y_old.data(), sphere_acc_Y.data(), nSpheres * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        gpuErrchk(cudaMemcpy(sphere_acc_Z_old.data(), sphere_acc_Z.data(), nSpheres * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        // if we have multistep AND friction, cache old alphas
        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
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
    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
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
        d_absv[my_sphere] = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
}

__host__ float ChSystemGpu_impl::get_max_vel() const {
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

__host__ int3 ChSystemGpu_impl::getSDTripletFromID(unsigned int SD_ID) const {
    return SDIDTriplet(SD_ID, gran_params);
}
/// Sort sphere positions by subdomain id
/// Occurs entirely on host, not intended to be efficient
/// ONLY DO AT BEGINNING OF SIMULATION
__host__ void ChSystemGpu_impl::defragment_initial_positions() {
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

    std::vector<float, cudallocator<float>> sphere_vel_x_tmp;
    std::vector<float, cudallocator<float>> sphere_vel_y_tmp;
    std::vector<float, cudallocator<float>> sphere_vel_z_tmp;

    std::vector<float, cudallocator<float>> sphere_angv_x_tmp;
    std::vector<float, cudallocator<float>> sphere_angv_y_tmp;
    std::vector<float, cudallocator<float>> sphere_angv_z_tmp;

    std::vector<not_stupid_bool, cudallocator<not_stupid_bool>> sphere_fixed_tmp;
    std::vector<unsigned int, cudallocator<unsigned int>> sphere_owner_SDs_tmp;

    sphere_pos_x_tmp.resize(nSpheres);
    sphere_pos_y_tmp.resize(nSpheres);
    sphere_pos_z_tmp.resize(nSpheres);

    sphere_vel_x_tmp.resize(nSpheres);
    sphere_vel_y_tmp.resize(nSpheres);
    sphere_vel_z_tmp.resize(nSpheres);

    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
        sphere_angv_x_tmp.resize(nSpheres);
        sphere_angv_y_tmp.resize(nSpheres);
        sphere_angv_z_tmp.resize(nSpheres);
    }

    sphere_fixed_tmp.resize(nSpheres);
    sphere_owner_SDs_tmp.resize(nSpheres);

    // reorder values into new sorted
    for (unsigned int i = 0; i < nSpheres; i++) {
        sphere_pos_x_tmp.at(i) = sphere_local_pos_X.at(sphere_ids.at(i));
        sphere_pos_y_tmp.at(i) = sphere_local_pos_Y.at(sphere_ids.at(i));
        sphere_pos_z_tmp.at(i) = sphere_local_pos_Z.at(sphere_ids.at(i));

        sphere_vel_x_tmp.at(i) = (float)pos_X_dt.at(sphere_ids.at(i));
        sphere_vel_y_tmp.at(i) = (float)pos_Y_dt.at(sphere_ids.at(i));
        sphere_vel_z_tmp.at(i) = (float)pos_Z_dt.at(sphere_ids.at(i));

        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
            sphere_angv_x_tmp.at(i) = (float)sphere_Omega_X.at(sphere_ids.at(i));
            sphere_angv_y_tmp.at(i) = (float)sphere_Omega_Y.at(sphere_ids.at(i));
            sphere_angv_z_tmp.at(i) = (float)sphere_Omega_Z.at(sphere_ids.at(i));
        }

        sphere_fixed_tmp.at(i) = sphere_fixed.at(sphere_ids.at(i));
        sphere_owner_SDs_tmp.at(i) = sphere_owner_SDs.at(sphere_ids.at(i));
    }

    // swap into the correct data structures
    sphere_local_pos_X.swap(sphere_pos_x_tmp);
    sphere_local_pos_Y.swap(sphere_pos_y_tmp);
    sphere_local_pos_Z.swap(sphere_pos_z_tmp);

    pos_X_dt.swap(sphere_vel_x_tmp);
    pos_Y_dt.swap(sphere_vel_y_tmp);
    pos_Z_dt.swap(sphere_vel_z_tmp);

    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
        sphere_Omega_X.swap(sphere_angv_x_tmp);
        sphere_Omega_Y.swap(sphere_angv_y_tmp);
        sphere_Omega_Z.swap(sphere_angv_z_tmp);
    }

    sphere_fixed.swap(sphere_fixed_tmp);
    sphere_owner_SDs.swap(sphere_owner_SDs_tmp);
}

/// Same defragment function, but this time for the contact friction history arrays.
/// It is stand-alone because it should rarely be needed, so let us save some time by
/// not calling it in most of our simulations.
__host__ void ChSystemGpu_impl::defragment_friction_history(unsigned int history_offset) {
    // key and value pointers
    std::vector<unsigned int, cudallocator<unsigned int>> sphere_ids;

    // load sphere indices
    sphere_ids.resize(nSpheres);
    std::iota(sphere_ids.begin(), sphere_ids.end(), 0);

    // sort sphere ids by owner SD
    std::sort(sphere_ids.begin(), sphere_ids.end(),
              [&](std::size_t i, std::size_t j) { return sphere_owner_SDs.at(i) < sphere_owner_SDs.at(j); });

    std::vector<float3, cudallocator<float3>> history_tmp;
    std::vector<unsigned int, cudallocator<unsigned int>> partners_tmp;

    history_tmp.resize(history_offset * nSpheres);
    partners_tmp.resize(history_offset * nSpheres);

    // reorder values into new sorted
    for (unsigned int i = 0; i < nSpheres; i++) {
        for (unsigned int j = 0; j < history_offset; j++) {
            history_tmp.at(history_offset * i + j) = contact_history_map.at(history_offset * sphere_ids.at(i) + j);
            partners_tmp.at(history_offset * i + j) = contact_partners_map.at(history_offset * sphere_ids.at(i) + j);
        }
    }

    contact_history_map.swap(history_tmp);
    contact_partners_map.swap(partners_tmp);
}

__host__ void ChSystemGpu_impl::setupSphereDataStructures() {
    // Each fills user_sphere_positions with positions to be copied
    if (user_sphere_positions.size() == 0) {
        CHGPU_ERROR("ERROR! no sphere positions given!\n");
    }

    nSpheres = (unsigned int)user_sphere_positions.size();
    INFO_PRINTF("%u balls added!\n", nSpheres);
    gran_params->nSpheres = nSpheres;

    TRACK_VECTOR_RESIZE(sphere_owner_SDs, nSpheres, "sphere_owner_SDs", NULL_CHGPU_ID);

    // Allocate space for new bodies
    TRACK_VECTOR_RESIZE(sphere_local_pos_X, nSpheres, "sphere_local_pos_X", 0);
    TRACK_VECTOR_RESIZE(sphere_local_pos_Y, nSpheres, "sphere_local_pos_Y", 0);
    TRACK_VECTOR_RESIZE(sphere_local_pos_Z, nSpheres, "sphere_local_pos_Z", 0);

    TRACK_VECTOR_RESIZE(sphere_fixed, nSpheres, "sphere_fixed", 0);

    TRACK_VECTOR_RESIZE(pos_X_dt, nSpheres, "pos_X_dt", 0);
    TRACK_VECTOR_RESIZE(pos_Y_dt, nSpheres, "pos_Y_dt", 0);
    TRACK_VECTOR_RESIZE(pos_Z_dt, nSpheres, "pos_Z_dt", 0);

    // temporarily store global positions as 64-bit, discard as soon as local positions are loaded
    {
        bool user_provided_fixed = user_sphere_fixed.size() != 0;
        bool user_provided_vel = user_sphere_vel.size() != 0;
        if (user_provided_fixed && user_sphere_fixed.size() != nSpheres)
            CHGPU_ERROR("Provided fixity array has length %zu, but there are %u spheres!\n", user_sphere_fixed.size(),
                        nSpheres);
        if (user_provided_vel && user_sphere_vel.size() != nSpheres)
            CHGPU_ERROR("Provided velocity array has length %zu, but there are %u spheres!\n", user_sphere_vel.size(),
                        nSpheres);

        std::vector<int64_t, cudallocator<int64_t>> sphere_global_pos_X;
        std::vector<int64_t, cudallocator<int64_t>> sphere_global_pos_Y;
        std::vector<int64_t, cudallocator<int64_t>> sphere_global_pos_Z;

        sphere_global_pos_X.resize(nSpheres);
        sphere_global_pos_Y.resize(nSpheres);
        sphere_global_pos_Z.resize(nSpheres);

        // Copy from array of structs to 3 arrays
        for (unsigned int i = 0; i < nSpheres; i++) {
            float3 vec = user_sphere_positions.at(i);
            // cast to double, convert to SU, then cast to int64_t
            sphere_global_pos_X.at(i) = (int64_t)((double)vec.x / LENGTH_SU2UU);
            sphere_global_pos_Y.at(i) = (int64_t)((double)vec.y / LENGTH_SU2UU);
            sphere_global_pos_Z.at(i) = (int64_t)((double)vec.z / LENGTH_SU2UU);

            // Convert to not_stupid_bool
            sphere_fixed.at(i) = (not_stupid_bool)((user_provided_fixed) ? user_sphere_fixed[i] : false);
            if (user_provided_vel) {
                auto vel = user_sphere_vel.at(i);
                pos_X_dt.at(i) = (float)(vel.x / VEL_SU2UU);
                pos_Y_dt.at(i) = (float)(vel.y / VEL_SU2UU);
                pos_Z_dt.at(i) = (float)(vel.z / VEL_SU2UU);
            }
        }

        packSphereDataPointers();
        // Figure our the number of blocks that need to be launched to cover the box
        unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
        initializeLocalPositions<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
            sphere_data, sphere_global_pos_X.data(), sphere_global_pos_Y.data(), sphere_global_pos_Z.data(), nSpheres,
            gran_params);

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
    }

    TRACK_VECTOR_RESIZE(sphere_acc_X, nSpheres, "sphere_acc_X", 0);
    TRACK_VECTOR_RESIZE(sphere_acc_Y, nSpheres, "sphere_acc_Y", 0);
    TRACK_VECTOR_RESIZE(sphere_acc_Z, nSpheres, "sphere_acc_Z", 0);

    // The buffer array that stores any quantity that the user wish to quarry. We resize it here once instead of
    // resizing on-the-call, to save time, in case that quarry function is called with a high frequency. The last
    // element in this array is to store the reduced value.
    TRACK_VECTOR_RESIZE(sphere_stats_buffer, nSpheres + 1, "sphere_stats_buffer", 0);
    TRACK_VECTOR_RESIZE(sphere_stats_buffer_int, nSpheres + 1, "sphere_stats_buffer_int", 0);

    // NOTE that this will get resized again later, this is just the first estimate
    TRACK_VECTOR_RESIZE(spheres_in_SD_composite, 2 * nSpheres, "spheres_in_SD_composite", NULL_CHGPU_ID);

    if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
        // add rotational DOFs
        TRACK_VECTOR_RESIZE(sphere_Omega_X, nSpheres, "sphere_Omega_X", 0);
        TRACK_VECTOR_RESIZE(sphere_Omega_Y, nSpheres, "sphere_Omega_Y", 0);
        TRACK_VECTOR_RESIZE(sphere_Omega_Z, nSpheres, "sphere_Omega_Z", 0);

        // add torques
        TRACK_VECTOR_RESIZE(sphere_ang_acc_X, nSpheres, "sphere_ang_acc_X", 0);
        TRACK_VECTOR_RESIZE(sphere_ang_acc_Y, nSpheres, "sphere_ang_acc_Y", 0);
        TRACK_VECTOR_RESIZE(sphere_ang_acc_Z, nSpheres, "sphere_ang_acc_Z", 0);

        {
            bool user_provided_ang_vel = user_sphere_ang_vel.size() != 0;
            if (user_provided_ang_vel && user_sphere_ang_vel.size() != nSpheres)
                CHGPU_ERROR("Provided angular velocity array has length %zu, but there are %u spheres!\n",
                            user_sphere_ang_vel.size(), nSpheres);
            if (user_provided_ang_vel) {
                for (unsigned int i = 0; i < nSpheres; i++) {
                    auto ang_vel = user_sphere_ang_vel.at(i);
                    sphere_Omega_X.at(i) = (float)(ang_vel.x * TIME_SU2UU);
                    sphere_Omega_Y.at(i) = (float)(ang_vel.y * TIME_SU2UU);
                    sphere_Omega_Z.at(i) = (float)(ang_vel.z * TIME_SU2UU);
                }
            }
        }
    }

    if (time_integrator == CHGPU_TIME_INTEGRATOR::CHUNG) {
        TRACK_VECTOR_RESIZE(sphere_acc_X_old, nSpheres, "sphere_acc_X_old", 0);
        TRACK_VECTOR_RESIZE(sphere_acc_Y_old, nSpheres, "sphere_acc_Y_old", 0);
        TRACK_VECTOR_RESIZE(sphere_acc_Z_old, nSpheres, "sphere_acc_Z_old", 0);

        // friction and multistep means keep old ang acc
        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
            TRACK_VECTOR_RESIZE(sphere_ang_acc_X_old, nSpheres, "sphere_ang_acc_X_old", 0);
            TRACK_VECTOR_RESIZE(sphere_ang_acc_Y_old, nSpheres, "sphere_ang_acc_Y_old", 0);
            TRACK_VECTOR_RESIZE(sphere_ang_acc_Z_old, nSpheres, "sphere_ang_acc_Z_old", 0);
        }
    }

    // If this is a new-boot, we usually want to do this defragment.
    // But if this is a restart, then probably no. We do not want every time the simulation restarts,
    // we have the order of particles completely changed: it may be bad for visualization or debugging
    if (defragment_on_start) {
        defragment_initial_positions();
    }

    bool user_provided_internal_data = false;
    if (gran_params->friction_mode == CHGPU_FRICTION_MODE::MULTI_STEP ||
        gran_params->friction_mode == CHGPU_FRICTION_MODE::SINGLE_STEP) {
        TRACK_VECTOR_RESIZE(contact_partners_map, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres, "contact_partners_map",
                            NULL_CHGPU_ID);
        TRACK_VECTOR_RESIZE(contact_active_map, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres, "contact_active_map", false);

        // If the user provides a checkpointed history array, we load it here
        bool user_provided_partner_map = user_partner_map.size() != 0;
        if (user_provided_partner_map && user_partner_map.size() != MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres)
            CHGPU_ERROR("ERROR! The user provided contact partner map has size %zu. It needs to be %u * %u!\n",
                        user_partner_map.size(), MAX_SPHERES_TOUCHED_BY_SPHERE, nSpheres);

        // Hope that using .at (instead of []) gives better err msg when things go wrong,
        // at the cost of some speed which is not important in I/O
        if (user_provided_partner_map) {
            for (unsigned int i = 0; i < nSpheres; i++) {
                for (unsigned int j = 0; j < MAX_SPHERES_TOUCHED_BY_SPHERE; j++) {
                    contact_partners_map.at(MAX_SPHERES_TOUCHED_BY_SPHERE * i + j) =
                        user_partner_map.at(MAX_SPHERES_TOUCHED_BY_SPHERE * i + j);
                }
            }
        }

        user_provided_internal_data = user_provided_internal_data || user_provided_partner_map;
    }

    if (gran_params->friction_mode == CHGPU_FRICTION_MODE::MULTI_STEP) {
        float3 null_history = {0., 0., 0.};
        TRACK_VECTOR_RESIZE(contact_history_map, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres, "contact_history_map",
                            null_history);
        TRACK_VECTOR_RESIZE(contact_duration, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres, "contact_duration", 0);

        // If the user provides a checkpointed history array, we load it here
        bool user_provided_friction_history = user_friction_history.size() != 0;
        if (user_provided_friction_history && user_friction_history.size() != MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres)
            CHGPU_ERROR("ERROR! The user provided contact friction history has size %zu. It needs to be %u * %u!\n",
                        user_friction_history.size(), MAX_SPHERES_TOUCHED_BY_SPHERE, nSpheres);

        if (user_provided_friction_history) {
            for (unsigned int i = 0; i < nSpheres; i++) {
                for (unsigned int j = 0; j < MAX_SPHERES_TOUCHED_BY_SPHERE; j++) {
                    float3 history_UU = user_friction_history[MAX_SPHERES_TOUCHED_BY_SPHERE * i + j];
                    float3 history_SU = make_float3(history_UU.x / LENGTH_SU2UU, history_UU.y / LENGTH_SU2UU,
                                                    history_UU.z / LENGTH_SU2UU);
                    contact_history_map.at(MAX_SPHERES_TOUCHED_BY_SPHERE * i + j) = history_SU;
                }
            }
        }

        user_provided_internal_data = user_provided_internal_data || user_provided_friction_history;
    }

    // This if content should be executed rarely, if at all.
    // If user gives Chrono::Gpu internal data from a file then it's a restart,
    // then defragment_on_start should be set to false. But I implemented it anyway.
    if (user_provided_internal_data && defragment_on_start) {
        defragment_friction_history(MAX_SPHERES_TOUCHED_BY_SPHERE);
    }

    // record normal contact force
    if (gran_params->recording_contactInfo == true) {
        float3 null_force = {0.0f, 0.0f, 0.0f};
        TRACK_VECTOR_RESIZE(normal_contact_force, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres, "normal contact force",
                            null_force);
    }

    // record friction force
    if (gran_params->recording_contactInfo == true && gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
        float3 null_force = {0.0f, 0.0f, 0.0f};
        TRACK_VECTOR_RESIZE(tangential_friction_force, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres,
                            "tangential contact force", null_force);
    }

    // record rolling friction torque
    if (gran_params->recording_contactInfo == true && gran_params->rolling_mode != CHGPU_ROLLING_MODE::NO_RESISTANCE) {
        float3 null_force = {0.0f, 0.0f, 0.0f};
        TRACK_VECTOR_RESIZE(rolling_friction_torque, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres,
                            "rolling friction torque", null_force);
        TRACK_VECTOR_RESIZE(char_collision_time, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres,
                            "characterisitc collision time", 0);
        TRACK_VECTOR_RESIZE(v_rot_array, MAX_SPHERES_TOUCHED_BY_SPHERE * nSpheres, "v rot", null_force);
    }

    // make sure the right pointers are packed
    packSphereDataPointers();
}

/// <summary>
/// runSphereBroadphase goes through three stages. First, a kernel figures out for each SD, how many spheres touch it.
/// Then, there is a prefix scan done (which requires two CUB function calls) to figure out offsets into the big fat
/// array that contains, for SD after SD, which spheres touch the SD. This last thing is accomplished by a kernel call.
///
/// CAVEAT: in this approach, the outcome of the prefix scan operation will be canibalized during the kernel call that
/// updates the big fat composite array. As such, there is a "scratch-pad" version that is used along the way
/// </summary>
/// <returns></returns>
__host__ void ChSystemGpu_impl::runSphereBroadphase() {
    METRICS_PRINTF("Resetting broadphase info!\n");

    // reset the number of spheres per SD, the offsets in the big composite array, and the big fat composite array
    resetBroadphaseInformation();

    // Frist stage of the computation in this function: Figure out the how many spheres touch each SD.
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    getNumberOfSpheresTouchingEachSD<CUDA_THREADS_PER_BLOCK>
        <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, nSpheres, gran_params);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    // Starting the second stage of this function call - the prefix scan operation
    unsigned int* out_ptr = SD_SphereCompositeOffsets.data();
    unsigned int* in_ptr = SD_NumSpheresTouching.data();
    gpuErrchk(cudaMemcpy(out_ptr, in_ptr, nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));

    // cold run; CUB determines the amount of storage it needs (since first argument is NULL pointer)
    size_t temp_storage_bytes = 0;
    cub::DeviceScan::ExclusiveSum(NULL, temp_storage_bytes, in_ptr, out_ptr, nSDs);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    // give CUB needed temporary storage on the device
    void* d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    // Run the actual exclusive prefix sum
    cub::DeviceScan::ExclusiveSum(d_scratch_space, temp_storage_bytes, in_ptr, out_ptr, nSDs);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    // Beginning of the last stage of computation in this function: assembling the big composite array.
    // num_entries: total number of sphere entries to record in the big fat composite array
    unsigned int num_entries = out_ptr[nSDs - 1] + in_ptr[nSDs - 1];
    spheres_in_SD_composite.resize(num_entries, NULL_CHGPU_ID);
    sphere_data->spheres_in_SD_composite = spheres_in_SD_composite.data();

    // Copy the offesets in the scratch pad; the subsequent kernel call would step on the outcome of the prefix scan
    gpuErrchk(cudaMemcpy(SD_SphereCompositeOffsets_ScratchPad.data(), SD_SphereCompositeOffsets.data(),
                         nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));
    // Populate the composite array; in the process, the content of the scratch pad will be modified
    // nBlocks = (MAX_SDs_TOUCHED_BY_SPHERE * nSpheres + 2*CUDA_THREADS_PER_BLOCK - 1) / (2*CUDA_THREADS_PER_BLOCK);
    // populateSpheresInEachSD<<<nBlocks, 2*CUDA_THREADS_PER_BLOCK>>>(sphere_data, nSpheres, gran_params);
    nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / (CUDA_THREADS_PER_BLOCK);
    populateSpheresInEachSD<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, nSpheres, gran_params);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
}

__host__ void ChSystemGpu_impl::updateBCPositions() {
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
        bd_offset_SU.x = (int64_t)(new_BD_offset.x / LENGTH_SU2UU);
        bd_offset_SU.y = (int64_t)(new_BD_offset.y / LENGTH_SU2UU);
        bd_offset_SU.z = (int64_t)(new_BD_offset.z / LENGTH_SU2UU);

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

__host__ double ChSystemGpu_impl::AdvanceSimulation(float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    // Settling simulation loop.
    float duration_SU = (float)(duration / TIME_SU2UU);
    unsigned int nsteps = (unsigned int)std::round(duration_SU / stepSize_SU);
    METRICS_PRINTF("advancing by %f at timestep %f, %u timesteps at approx user timestep %f\n", duration_SU,
                   stepSize_SU, nsteps, duration / nsteps);
    float time_elapsed_SU = 0;  // time elapsed in this advance call

    packSphereDataPointers();

    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (unsigned int n = 0; n < nsteps; n++) {
        updateBCPositions();
        runSphereBroadphase();
        resetSphereAccelerations();
        resetBCForces();

        METRICS_PRINTF("Starting computeSphereForces!\n");

        if (gran_params->friction_mode == CHGPU_FRICTION_MODE::FRICTIONLESS) {
            // Compute sphere-sphere forces
            computeSphereForces_frictionless_matBased<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                (unsigned int)BC_params_list_SU.size());
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        } else if (gran_params->friction_mode == CHGPU_FRICTION_MODE::SINGLE_STEP ||
                   gran_params->friction_mode == CHGPU_FRICTION_MODE::MULTI_STEP) {
            // figure out who is contacting
            determineContactPairs<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(sphere_data, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());

            if (gran_params->use_mat_based == true) {
                computeSphereContactForces_matBased<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                    sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                    (unsigned int)BC_params_list_SU.size(), nSpheres);

            } else {
                computeSphereContactForces<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                    sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                    (unsigned int)BC_params_list_SU.size(), nSpheres);
            }

            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }

        METRICS_PRINTF("Starting integrateSpheres!\n");
        integrateSpheres<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
            const unsigned int nThreadsUpdateHist = 2 * CUDA_THREADS_PER_BLOCK;
            unsigned int fricMapSize = nSpheres * MAX_SPHERES_TOUCHED_BY_SPHERE;
            unsigned int nBlocksFricHistoryPostProcess = (fricMapSize + nThreadsUpdateHist - 1) / nThreadsUpdateHist;

            METRICS_PRINTF("Update Friction Data!\n");

            updateFrictionData<<<nBlocksFricHistoryPostProcess, nThreadsUpdateHist>>>(fricMapSize, sphere_data,
                                                                                      gran_params);

            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            METRICS_PRINTF("Update angular velocity.\n");
            updateAngVels<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }

        elapsedSimTime += (float)(stepSize_SU * TIME_SU2UU);  // Advance current time
        time_elapsed_SU += stepSize_SU;
    }

    return time_elapsed_SU * TIME_SU2UU;  // return elapsed UU time
}
}  // namespace gpu
}  // namespace chrono
