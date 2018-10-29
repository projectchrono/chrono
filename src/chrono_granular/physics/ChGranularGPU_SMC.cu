// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly, Nic Olsen
// =============================================================================

#include "../chrono_granular/physics/ChGranularGPU_SMC.cuh"

namespace chrono {
namespace granular {

__host__ double ChSystemGranular::get_max_z() const {
    int* max_z_d;
    int max_z_h;
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;

    gpuErrchk(cudaMalloc(&max_z_d, sizeof(int)));

    cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, pos_Z.data(), max_z_d, nDEs);
    gpuErrchk(cudaDeviceSynchronize());

    // Allocate temporary storage
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    // Run max-reduction
    cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, pos_Z.data(), max_z_d, nDEs);
    gpuErrchk(cudaDeviceSynchronize());

    gpuErrchk(cudaMemcpy(&max_z_h, max_z_d, sizeof(int), cudaMemcpyDeviceToHost));

    double max_z_UU = max_z_h * gran_params->LENGTH_UNIT;
    gpuErrchk(cudaFree(max_z_d));
    gpuErrchk(cudaDeviceSynchronize());

    return max_z_UU;
}

/// Copy constant sphere data to device, this should run at start
__host__ void ChSystemGranularMonodisperse_SMC::copy_const_data_to_device() {
    // Copy quantities expressed in SU units for the SD dimensions to device
    gran_params->SD_size_X_SU = SD_size_X_SU;
    gran_params->SD_size_Y_SU = SD_size_Y_SU;
    gran_params->SD_size_Z_SU = SD_size_Z_SU;
    // Copy global BD size in multiples of SDs to device
    gran_params->nSDs_X = nSDs_X;
    gran_params->nSDs_Y = nSDs_Y;
    gran_params->nSDs_Z = nSDs_Z;

    gran_params->gravAcc_X_SU = gravity_X_SU;
    gran_params->gravAcc_Y_SU = gravity_Y_SU;
    gran_params->gravAcc_Z_SU = gravity_Z_SU;
    gran_params->gravMag_SU =
        std::sqrt(gravity_X_SU * gravity_X_SU + gravity_Y_SU * gravity_Y_SU + gravity_Z_SU * gravity_Z_SU);

    gran_params->sphereRadius_SU = sphereRadius_SU;

    gran_params->Kn_s2s_SU = K_n_s2s_SU;
    gran_params->Kn_s2w_SU = K_n_s2w_SU;
    gran_params->Gamma_n_s2s_SU = Gamma_n_s2s_SU;
    gran_params->Gamma_n_s2w_SU = Gamma_n_s2w_SU;

    gran_params->K_t_s2s_SU = K_t_s2s_SU;
    gran_params->K_t_s2w_SU = K_t_s2w_SU;
    gran_params->Gamma_t_s2s_SU = Gamma_t_s2s_SU;
    gran_params->Gamma_t_s2w_SU = Gamma_t_s2w_SU;

    gran_params->cohesion_ratio = cohesion_over_gravity;

    gran_params->integrator_type = time_integrator;
    gran_params->contact_model = contact_model;
    gran_params->friction_mode = fric_mode;
}

/// Similar to the copy_const_data_to_device, but saves us a big copy
/// This can run at every timestep to allow a moving BD
__host__ void ChSystemGranularMonodisperse_SMC::copyBD_Frame_to_device() {
    // Unified memory does all the work here
    gran_params->BD_frame_X = BD_frame_X;
    gran_params->BD_frame_Y = BD_frame_Y;
    gran_params->BD_frame_Z = BD_frame_Z;
    gran_params->BD_frame_X_dot = BD_frame_X_dot;
    gran_params->BD_frame_Y_dot = BD_frame_Y_dot;
    gran_params->BD_frame_Z_dot = BD_frame_Z_dot;
}

// Check number of spheres in each SD and dump relevant info to file
void ChSystemGranularMonodisperse_SMC::checkSDCounts(std::string ofile, bool write_out = false, bool verbose = false) {
    // Count of DEs in each SD
    unsigned int* sdvals = SD_NumOf_DEs_Touching.data();
    // DEs that are in each SD
    unsigned int* sdSpheres = DEs_in_SD_composite.data();
    // # times each DE appears in some SD
    unsigned int* deCounts = new unsigned int[nDEs];

    // could use memset instead, just need to zero these out
    for (unsigned int i = 0; i < nDEs; i++) {
        deCounts[i] = 0;
    }

    unsigned int max_count = 0;
    unsigned int sum = 0;
    for (unsigned int i = 0; i < nSDs; i++) {
        // printf("count is %u for SD sd %u \n", sdvals[i], i);
        sum += sdvals[i];
        if (sdvals[i] > max_count)
            max_count = sdvals[i];
    }
    // safety checks, if these fail we were probably about to crash
    assert(sum < MAX_COUNT_OF_DEs_PER_SD * nSDs);
    assert(max_count < MAX_COUNT_OF_DEs_PER_SD);
    if (verbose) {
        printf("max DEs per SD is %u\n", max_count);
        printf("total sd/de overlaps is %u\n", sum);
        printf("theoretical total is %u\n", MAX_COUNT_OF_DEs_PER_SD * nSDs);
    }
    // Copy over occurences in SDs
    for (unsigned int i = 0; i < MAX_COUNT_OF_DEs_PER_SD * nSDs; i++) {
        // printf("de id is %d, i is %u\n", sdSpheres[i], i);
        // Check if invalid sphere
        if (sdSpheres[i] == NULL_GRANULAR_ID) {
            // printf("invalid sphere in sd");
        } else {
            assert(sdSpheres[i] < nDEs);
            deCounts[sdSpheres[i]]++;
        }
    }
    if (write_out) {
        writeFile(ofile, deCounts);
    }
    delete[] deCounts;
}
// This can belong to the superclass but does reference deCounts which may not be a thing when DVI rolls around
void ChSystemGranularMonodisperse_SMC::writeFile(std::string ofile, unsigned int* deCounts) {
    // unnecessary if called by checkSDCounts()
    // The file writes are a pretty big slowdown in CSV mode
    if (file_write_mode == GRAN_OUTPUT_MODE::BINARY) {
        // Write the data as binary to a file, requires later postprocessing that can be done in parallel, this is a
        // much faster write due to no formatting
        std::ofstream ptFile(ofile + ".raw", std::ios::out | std::ios::binary);

        for (unsigned int n = 0; n < nDEs; n++) {
            float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                              pos_Z_dt.at(n) * pos_Z_dt.at(n));

            ptFile.write((const char*)&pos_X.at(n), sizeof(int));
            ptFile.write((const char*)&pos_Y.at(n), sizeof(int));
            ptFile.write((const char*)&pos_Z.at(n), sizeof(int));
            ptFile.write((const char*)&pos_X_dt.at(n), sizeof(float));
            ptFile.write((const char*)&pos_Y_dt.at(n), sizeof(float));
            ptFile.write((const char*)&pos_Z_dt.at(n), sizeof(float));
            ptFile.write((const char*)&absv, sizeof(float));
            ptFile.write((const char*)&deCounts[n], sizeof(int));
        }
    } else if (file_write_mode == GRAN_OUTPUT_MODE::CSV) {
        // CSV is much slower but requires less postprocessing
        std::ofstream ptFile(ofile + ".csv", std::ios::out);

        // Dump to a stream, write to file only at end
        std::ostringstream outstrstream;
        outstrstream << "x,y,z,vx,vy,vz,absv,nTouched\n";

        for (unsigned int n = 0; n < nDEs; n++) {
            float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                              pos_Z_dt.at(n) * pos_Z_dt.at(n));
            outstrstream << pos_X.at(n) << "," << pos_Y.at(n) << "," << pos_Z.at(n) << "," << pos_X_dt.at(n) << ","
                         << pos_Y_dt.at(n) << "," << pos_Z_dt.at(n) << "," << absv << "," << deCounts[n] << "\n";
        }

        ptFile << outstrstream.str();
    } else if (file_write_mode == GRAN_OUTPUT_MODE::NONE) {
        // Do nothing, only here for symmetry
    }
}

// This can belong to the superclass but does reference deCounts which may not be a thing when DVI rolls around
void ChSystemGranularMonodisperse_SMC::writeFileUU(std::string ofile) {
    // The file writes are a pretty big slowdown in CSV mode
    if (file_write_mode == GRAN_OUTPUT_MODE::BINARY) {
        // TODO implement this
        // Write the data as binary to a file, requires later postprocessing that can be done in parallel, this is a
        // much faster write due to no formatting
        // std::ofstream ptFile(ofile + ".raw", std::ios::out | std::ios::binary);
        //
        // for (unsigned int n = 0; n < nDEs; n++) {
        //     float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
        //                       pos_Z_dt.at(n) * pos_Z_dt.at(n));
        //
        //     ptFile.write((const char*)&pos_X.at(n), sizeof(int));
        //     ptFile.write((const char*)&pos_Y.at(n), sizeof(int));
        //     ptFile.write((const char*)&pos_Z.at(n), sizeof(int));
        //     ptFile.write((const char*)&pos_X_dt.at(n), sizeof(float));
        //     ptFile.write((const char*)&pos_Y_dt.at(n), sizeof(float));
        //     ptFile.write((const char*)&pos_Z_dt.at(n), sizeof(float));
        //     ptFile.write((const char*)&absv, sizeof(float));
        //     ptFile.write((const char*)&deCounts[n], sizeof(int));
        // }
    } else if (file_write_mode == GRAN_OUTPUT_MODE::CSV) {
        // CSV is much slower but requires less postprocessing
        std::ofstream ptFile(ofile + ".csv", std::ios::out);

        // Dump to a stream, write to file only at end
        std::ostringstream outstrstream;
        outstrstream << "x,y,z,USU,wx,wy,wz\n";

        for (unsigned int n = 0; n < nDEs; n++) {
            // TODO convert absv into UU
            float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                              pos_Z_dt.at(n) * pos_Z_dt.at(n));

            outstrstream << pos_X.at(n) * gran_params->LENGTH_UNIT << "," << pos_Y.at(n) * gran_params->LENGTH_UNIT
                         << "," << pos_Z.at(n) * gran_params->LENGTH_UNIT << "," << absv;

            if (fric_mode != GRAN_FRICTION_MODE::FRICTIONLESS) {
                outstrstream << "," << omega_X.at(n) << "," << omega_Y.at(n) << "," << omega_Z.at(n);
            }
            outstrstream << "\n";
        }

        ptFile << outstrstream.str();
    } else if (file_write_mode == GRAN_OUTPUT_MODE::NONE) {
        // Do nothing, only here for symmetry
    }
}

// Reset broadphase data structures
void ChSystemGranularMonodisperse_SMC::resetBroadphaseInformation() {
    // Set all the offsets to zero
    gpuErrchk(cudaMemset(SD_NumOf_DEs_Touching.data(), 0, nSDs * sizeof(unsigned int)));
    // For each SD, all the spheres touching that SD should have their ID be NULL_GRANULAR_ID
    gpuErrchk(cudaMemset(DEs_in_SD_composite.data(), NULL_GRANULAR_ID,
                         MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));
}
// Reset sphere-sphere force data structures
void ChSystemGranularMonodisperse_SMC::resetSphereForces() {
    // cache past force data
    if (time_integrator == GRAN_TIME_INTEGRATOR::CHUNG) {
        gpuErrchk(cudaMemcpy(sphere_force_X_old.data(), sphere_force_X.data(), nDEs * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        gpuErrchk(cudaMemcpy(sphere_force_Y_old.data(), sphere_force_Y.data(), nDEs * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        gpuErrchk(cudaMemcpy(sphere_force_Z_old.data(), sphere_force_Z.data(), nDEs * sizeof(float),
                             cudaMemcpyDeviceToDevice));
        gpuErrchk(cudaDeviceSynchronize());
    }
    // reset forces to zero
    gpuErrchk(cudaMemset(sphere_force_X.data(), 0, nDEs * sizeof(float)));
    gpuErrchk(cudaMemset(sphere_force_Y.data(), 0, nDEs * sizeof(float)));
    gpuErrchk(cudaMemset(sphere_force_Z.data(), 0, nDEs * sizeof(float)));

    // reset torques to zero, if applicable
    if (fric_mode != FRICTIONLESS) {
        gpuErrchk(cudaMemset(sphere_torque_X.data(), 0, nDEs * sizeof(float)));
        gpuErrchk(cudaMemset(sphere_torque_Y.data(), 0, nDEs * sizeof(float)));
        gpuErrchk(cudaMemset(sphere_torque_Z.data(), 0, nDEs * sizeof(float)));
    }
}

void ChSystemGranularMonodisperse_SMC::updateBDPosition(const float stepSize_SU) {
    // Frequency of oscillation
    float frame_X_old = BD_frame_X;
    float frame_Y_old = BD_frame_Y;
    float frame_Z_old = BD_frame_Z;
    // Put the bottom-left corner of box wherever the user told us to
    BD_frame_X = (box_size_X * (BDPositionFunctionX(elapsedSimTime))) / gran_params->LENGTH_UNIT;
    BD_frame_Y = (box_size_Y * (BDPositionFunctionY(elapsedSimTime))) / gran_params->LENGTH_UNIT;
    BD_frame_Z = (box_size_Z * (BDPositionFunctionZ(elapsedSimTime))) / gran_params->LENGTH_UNIT;

    BD_frame_X_dot = (BD_frame_X - frame_X_old) / stepSize_SU;
    BD_frame_Y_dot = (BD_frame_Y - frame_Y_old) / stepSize_SU;
    BD_frame_Z_dot = (BD_frame_Z - frame_Z_old) / stepSize_SU;

    copyBD_Frame_to_device();
}

// All the information a moving sphere needs
typedef struct {
    int pos_X;
    int pos_Y;
    int pos_Z;
    float pos_X_dt;
    float pos_Y_dt;
    float pos_Z_dt;
} sphere_data_struct;

template <unsigned int CUB_THREADS>
__global__ void owner_prepack(int* d_sphere_pos_X,
                              int* d_sphere_pos_Y,
                              int* d_sphere_pos_Z,
                              float* d_sphere_pos_X_dt,
                              float* d_sphere_pos_Y_dt,
                              float* d_sphere_pos_Z_dt,
                              unsigned int nSpheres,
                              unsigned int* owners,
                              sphere_data_struct* sphere_info,
                              ParamsPtr gran_params) {
    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // Only do this for valid spheres
    if (mySphereID >= nSpheres) {
        return;
    }
    // Find this SD's owner
    owners[mySphereID] = SDTripletID(
        pointSDTriplet(d_sphere_pos_X[mySphereID], d_sphere_pos_Y[mySphereID], d_sphere_pos_Z[mySphereID], gran_params),
        gran_params);

    sphere_data_struct mydata = sphere_info[mySphereID];

    // The value is the sphere id, to be sorted with owner as the key
    mydata.pos_X = d_sphere_pos_X[mySphereID];
    mydata.pos_Y = d_sphere_pos_Y[mySphereID];
    mydata.pos_Z = d_sphere_pos_Z[mySphereID];
    mydata.pos_X_dt = d_sphere_pos_X_dt[mySphereID];
    mydata.pos_Y_dt = d_sphere_pos_Y_dt[mySphereID];
    mydata.pos_Z_dt = d_sphere_pos_Z_dt[mySphereID];
    // replace with the new data
    sphere_info[mySphereID] = mydata;
}
// unpack sorted data to global memory, very coalesced
template <unsigned int CUB_THREADS>
__global__ void owner_unpack(int* d_sphere_pos_X,
                             int* d_sphere_pos_Y,
                             int* d_sphere_pos_Z,
                             float* d_sphere_pos_X_dt,
                             float* d_sphere_pos_Y_dt,
                             float* d_sphere_pos_Z_dt,
                             unsigned int nSpheres,
                             sphere_data_struct* sphere_info,
                             ParamsPtr gran_params) {
    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    // Only do this for valid spheres
    if (mySphereID >= nSpheres) {
        return;
    }
    sphere_data_struct mydata = sphere_info[mySphereID];

    // The value is the sphere id, to be sorted with owner as the key
    d_sphere_pos_X[mySphereID] = mydata.pos_X;
    d_sphere_pos_Y[mySphereID] = mydata.pos_Y;
    d_sphere_pos_Z[mySphereID] = mydata.pos_Z;
    d_sphere_pos_X_dt[mySphereID] = mydata.pos_X_dt;
    d_sphere_pos_Y_dt[mySphereID] = mydata.pos_Y_dt;
    d_sphere_pos_Z_dt[mySphereID] = mydata.pos_Z_dt;
}

// Sorts data by owner SD, makes nicer memory accesses
// Uses a boatload of memory
__host__ void ChSystemGranularMonodisperse_SMC::defragment_data() {
    VERBOSE_PRINTF("Starting defrag run!\n");
    unsigned int nBlocks = (nDEs + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Set of pointers for each buffer
    unsigned int* d_owners;
    sphere_data_struct* d_sphere_data;
    // second buffer for nice sort
    unsigned int* d_owners_2;
    sphere_data_struct* d_sphere_data_2;
    // Allocate some nice memory
    gpuErrchk(cudaMalloc(&d_owners, nDEs * sizeof(unsigned int)));
    gpuErrchk(cudaMalloc(&d_sphere_data, nDEs * sizeof(sphere_data_struct)));
    gpuErrchk(cudaMalloc(&d_owners_2, nDEs * sizeof(unsigned int)));
    gpuErrchk(cudaMalloc(&d_sphere_data_2, nDEs * sizeof(sphere_data_struct)));
    owner_prepack<CUDA_THREADS_PER_BLOCK><<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
        pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data(), nDEs, d_owners,
        d_sphere_data, gran_params);
    gpuErrchk(cudaDeviceSynchronize());

    // Create a set of DoubleBuffers to wrap pairs of device pointers
    cub::DoubleBuffer<unsigned int> d_keys(d_owners, d_owners_2);
    cub::DoubleBuffer<sphere_data_struct> d_values(d_sphere_data, d_sphere_data_2);

    // Determine temporary device storage requirements
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    // pass null, cub tells us what it needs
    cub::DeviceRadixSort::SortPairs(NULL, temp_storage_bytes, d_keys, d_values, nDEs);
    gpuErrchk(cudaDeviceSynchronize());

    // Allocate temporary storage
    gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
    gpuErrchk(cudaDeviceSynchronize());

    cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, d_keys, d_values, nDEs);
    gpuErrchk(cudaDeviceSynchronize());

    owner_unpack<CUDA_THREADS_PER_BLOCK>
        <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt.data(),
                                              pos_Y_dt.data(), pos_Z_dt.data(), nDEs, d_values.Current(), gran_params);
    gpuErrchk(cudaDeviceSynchronize());
    cudaFree(d_owners);
    cudaFree(d_owners_2);
    cudaFree(d_sphere_data);
    cudaFree(d_sphere_data_2);
    cudaFree(d_temp_storage);
    VERBOSE_PRINTF("defrag finished!\n");
}

__global__ void generate_absv(const unsigned int nDEs,
                              const float* velX,
                              const float* velY,
                              const float* velZ,
                              float* d_absv) {
    unsigned int my_sphere = blockIdx.x * blockDim.x + threadIdx.x;
    if (my_sphere < nDEs) {
        float v[3] = {velX[my_sphere], velY[my_sphere], velZ[my_sphere]};
        d_absv[my_sphere] = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    }
}

__host__ float ChSystemGranular::get_max_vel() {
    float* d_absv;
    float* d_max_vel;
    float h_max_vel;
    gpuErrchk(cudaMalloc(&d_absv, nDEs * sizeof(float)));
    gpuErrchk(cudaMalloc(&d_max_vel, sizeof(float)));

    generate_absv<<<(nDEs + 255) / 256, 256>>>(nDEs, pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data(), d_absv);

    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, d_absv, d_max_vel, nDEs);
    gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
    cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, d_absv, d_max_vel, nDEs);
    gpuErrchk(cudaMemcpy(&h_max_vel, d_max_vel, sizeof(float), cudaMemcpyDeviceToHost));

    gpuErrchk(cudaFree(d_absv));
    gpuErrchk(cudaFree(d_max_vel));

    return h_max_vel;
}

__host__ void ChSystemGranularMonodisperse_SMC::initialize() {
    switch_to_SimUnits();
    generate_DEs();

    // Set aside memory for holding data structures worked with. Get some initializations going
    setup_simulation();
    copy_const_data_to_device();
    copyBD_Frame_to_device();
    gpuErrchk(cudaDeviceSynchronize());

    determine_new_stepSize_SU();

    // Seed arrays that are populated by the kernel call
    resetBroadphaseInformation();

    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    printf("doing priming!\n");
    printf("max possible composite offset is %zu\n", (size_t)nSDs * MAX_COUNT_OF_DEs_PER_SD);

    auto sphere_data = packSphereDataPointers();

    primingOperationsRectangularBox<CUDA_THREADS_PER_BLOCK>
        <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(sphere_data, nDEs, gran_params);
    gpuErrchk(cudaDeviceSynchronize());
    printf("priming finished!\n");

    printf("z grav term with timestep %f is %f\n", stepSize_SU, stepSize_SU * stepSize_SU * gravity_Z_SU);
    printf("running at approximate timestep %f\n", stepSize_SU * gran_params->TIME_UNIT);
}

__host__ double ChSystemGranularMonodisperse_SMC::advance_simulation(float duration) {
    auto sphere_data = packSphereDataPointers();

    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Settling simulation loop.
    float duration_SU = std::ceil(duration / gran_params->TIME_UNIT);
    unsigned int nsteps = duration_SU / stepSize_SU;

    VERBOSE_PRINTF("advancing by %f at timestep %f, %u timesteps at approx user timestep %f\n", duration_SU,
                   stepSize_SU, nsteps, duration / nsteps);
    float time_elapsed_SU = 0;  // time elapsed in this advance call
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (; time_elapsed_SU < stepSize_SU * nsteps; time_elapsed_SU += stepSize_SU) {
        determine_new_stepSize_SU();  // doesn't always change the timestep

        gran_params->alpha_h_bar = stepSize_SU;
        // Update the position and velocity of the BD, if relevant
        if (!BD_is_fixed) {
            updateBDPosition(stepSize_SU);  // TODO current time
        }
        resetSphereForces();

        VERBOSE_PRINTF("Starting computeSphereForces!\n");

        // Compute sphere-sphere forces
        computeSphereForces<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            sphere_data, gran_params, BC_type_list.data(), BC_params_list.data(), BC_params_list.size());

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        VERBOSE_PRINTF("Resetting broadphase info!\n");

        resetBroadphaseInformation();

        VERBOSE_PRINTF("Starting updatePositions!\n");
        updatePositions<CUDA_THREADS_PER_BLOCK>
            <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nDEs, gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        elapsedSimTime += stepSize_SU * gran_params->TIME_UNIT;  // Advance current time
    }

    return time_elapsed_SU * gran_params->TIME_UNIT;  // return elapsed UU time
}
}  // namespace granular
}  // namespace chrono
