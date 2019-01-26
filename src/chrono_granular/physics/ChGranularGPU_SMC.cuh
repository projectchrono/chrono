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
// Holds internal functions and kernels for running a sphere-sphere timestep
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly, Nic Olsen
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
#include "chrono/core/ChVector.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
#include "chrono_granular/utils/ChGranularUtilities.cuh"

using chrono::granular::sphereDataStruct;
using chrono::granular::contactDataStruct;

// NOTE the above 'using's must happen before this file is included
#include "chrono_granular/physics/ChGranularHelpers.cuh"
#include "chrono_granular/physics/ChGranularBoundaryConditions.cuh"

/// Takes in a sphere's position and inserts into the given int array[8] which subdomains, if any, are touched
/// The array is indexed with the ones bit equal to +/- x, twos bit equal to +/- y, and the fours bit equal to +/- z
/// A bit set to 0 means the lower index, whereas 1 means the higher index (lower + 1)
/// The kernel computes global x, y, and z indices for the bottom-left subdomain and then uses those to figure out
/// which subdomains described in the corresponding 8-SD cube are touched by the sphere. The kernel then converts
/// these indices to indices into the global SD list via the (currently local) conv[3] data structure Should be
/// mostly bug-free, especially away from boundaries
inline __device__ void figureOutTouchedSD(signed int sphCenter_X,
                                          signed int sphCenter_Y,
                                          signed int sphCenter_Z,
                                          unsigned int SDs[MAX_SDs_TOUCHED_BY_SPHERE],
                                          GranParamsPtr gran_params) {
    // grab radius as signed so we can use it intelligently
    const signed int sphereRadius_SU = gran_params->sphereRadius_SU;
    // I added these to fix a bug, we can inline them if/when needed but they ARE necessary
    // We need to offset so that the bottom-left corner is at the origin

    // TODO this should never be over 2 billion anyways
    signed int nx[2], ny[2], nz[2];

    {
        // TODO this should probably involve more casting
        int64_t sphere_pos_modified_X = -gran_params->BD_frame_X + sphCenter_X;
        int64_t sphere_pos_modified_Y = -gran_params->BD_frame_Y + sphCenter_Y;
        int64_t sphere_pos_modified_Z = -gran_params->BD_frame_Z + sphCenter_Z;

        // get the bottom-left-most SD that the particle touches
        // nx = (xCenter - radius) / wx
        nx[0] = (signed int)((sphere_pos_modified_X - sphereRadius_SU) / gran_params->SD_size_X_SU);
        // Same for Y and Z
        ny[0] = (signed int)((sphere_pos_modified_Y - sphereRadius_SU) / gran_params->SD_size_Y_SU);
        nz[0] = (signed int)((sphere_pos_modified_Z - sphereRadius_SU) / gran_params->SD_size_Z_SU);

        // get the top-right-most SD that the particle touches
        nx[1] = (signed int)((sphere_pos_modified_X + sphereRadius_SU) / gran_params->SD_size_X_SU);
        ny[1] = (signed int)((sphere_pos_modified_Y + sphereRadius_SU) / gran_params->SD_size_Y_SU);
        nz[1] = (signed int)((sphere_pos_modified_Z + sphereRadius_SU) / gran_params->SD_size_Z_SU);
    }
    // figure out what
    // number of iterations in each direction
    int num_x = (nx[0] == nx[1]) ? 1 : 2;
    int num_y = (ny[0] == ny[1]) ? 1 : 2;
    int num_z = (nz[0] == nz[1]) ? 1 : 2;

    // TODO unroll me
    for (int i = 0; i < num_x; i++) {
        for (int j = 0; j < num_y; j++) {
            for (int k = 0; k < num_z; k++) {
                // composite index to write to
                int id = i * 4 + j * 2 + k;
                // if I ran out of the box, give up and set this to NULL
                if ((nx[i] < 0 || nx[i] >= gran_params->nSDs_X) || (ny[j] < 0 || ny[j] >= gran_params->nSDs_Y) ||
                    (nz[k] < 0 || nz[k] >= gran_params->nSDs_Z)) {
                    SDs[id] = NULL_GRANULAR_ID;
                    continue;  // skip this SD
                }

                // ok so now this SD id is ok, carry on
                SDs[id] = nx[i] * gran_params->nSDs_Y * gran_params->nSDs_Z + ny[j] * gran_params->nSDs_Z + nz[k];
            }
        }
    }
}
/**
 * This kernel call prepares information that will be used in a subsequent kernel that performs the actual time
 * stepping.
 *
 * Template arguments:
 *   - CUB_THREADS: the number of threads used in this kernel, comes into play when invoking CUB block collectives
 *
 * Assumptions:
 *   - Granular material is made up of monodisperse spheres.
 *   - The function below assumes the spheres are in a box
 *   - The box has dimensions L x D x H.
 *   - The reference frame associated with the box:
 *       - The x-axis is along the length L of the box
 *       - The y-axis is along the width D of the box
 *       - The z-axis is along the height H of the box
 *   - A sphere cannot touch more than eight SDs
 *
 * Basic idea: use domain decomposition on the rectangular box and figure out how many SDs each sphere touches.
 * The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is
 * at the center of the box. The orientation of the box is defined relative to a world inertial reference frame.
 *
 * Nomenclature:
 *   - SD: subdomain.
 *   - BD: the big-domain, which is the union of all SDs
 *   - NULL_GRANULAR_ID: the equivalent of a non-sphere SD ID, or a non-sphere ID
 *
 * Notes:
 *   - The SD with ID=0 is the catch-all SD. This is the SD in which a sphere ends up if its not inside the rectangular
 * box. Usually, there is no sphere in this SD (THIS IS NOT IMPLEMENTED AS SUCH FOR NOW)
 *
 */
template <unsigned int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a
                                     //!< multiple of 32
__global__ void sphereBroadphase_dryrun(sphereDataStruct sphere_data,
                                        unsigned int nSpheres,  //!< Number of spheres in the box
                                        GranParamsPtr gran_params) {
    signed int xSphCenter;
    signed int ySphCenter;
    signed int zSphCenter;

    /// Set aside shared memory
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, MAX_SDs_TOUCHED_BY_SPHERE> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // This uses a lot of registers but is needed
    unsigned int SDsTouched[MAX_SDs_TOUCHED_BY_SPHERE] = {NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID};
    if (mySphereID < nSpheres) {
        // Coalesced mem access
        xSphCenter = sphere_data.pos_X[mySphereID];
        ySphCenter = sphere_data.pos_Y[mySphereID];
        zSphCenter = sphere_data.pos_Z[mySphereID];
        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched, gran_params);
    }

    __syncthreads();

    // Sort by the ID of the SD touched
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_SPHERE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] = head_flags[i];
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS spheres. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i;
            unsigned int winningStreak = 0;  // should always be under 256 by simulation constraints
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < MAX_SDs_TOUCHED_BY_SPHERE * CUB_THREADS &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            // Store start of new entries
            unsigned char sphere_offset = atomicAdd(sphere_data.SD_NumSpheresTouching + touchedSD, winningStreak);
        }
    }
}

template <unsigned int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a
                                     //!< multiple of 32
__global__ void sphereBroadphase(sphereDataStruct sphere_data,
                                 unsigned int nSpheres,  //!< Number of spheres in the box
                                 GranParamsPtr gran_params,
                                 unsigned int numentries = 0) {
    signed int xSphCenter;
    signed int ySphCenter;
    signed int zSphCenter;

    /// Set aside shared memory
    // SD component of offset into composite array
    volatile __shared__ unsigned int sphere_composite_offsets[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, MAX_SDs_TOUCHED_BY_SPHERE, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int sphIDs[MAX_SDs_TOUCHED_BY_SPHERE] = {mySphereID, mySphereID, mySphereID, mySphereID,
                                                      mySphereID, mySphereID, mySphereID, mySphereID};

    // This uses a lot of registers but is needed
    unsigned int SDsTouched[MAX_SDs_TOUCHED_BY_SPHERE] = {NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID};
    if (mySphereID < nSpheres) {
        // Coalesced mem access
        xSphCenter = sphere_data.pos_X[mySphereID];
        ySphCenter = sphere_data.pos_Y[mySphereID];
        zSphCenter = sphere_data.pos_Z[mySphereID];
        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched, gran_params);
    }

    __syncthreads();

    // Sort by the ID of the SD touched
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, sphIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_SPHERE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_SphInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        sphere_composite_offsets[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID;
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS spheres. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i;
            unsigned int winningStreak = 0;  // should always be under 256 by simulation constraints
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < MAX_SDs_TOUCHED_BY_SPHERE * CUB_THREADS &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            // Store start of new entries
            // TODO this assumes that the offset is less than uint
            unsigned int sphere_offset = atomicAdd(sphere_data.SD_SphereCompositeOffsets + touchedSD, winningStreak);

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int sphereInStreak = 0; sphereInStreak < winningStreak; sphereInStreak++) {
                // add this sphere to that sd, incrementing offset for next guy
                sphere_composite_offsets[idInShared + sphereInStreak] = sphere_offset++;
            }
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; register with sphere_data.spheres_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        unsigned int offset = sphere_composite_offsets[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i];
        // Add offsets together
        // bad SD means not a valid offset
        if (offset != NULL_GRANULAR_ID) {
            // printf("thread %d block %d is writing sphere %u to offset %u\n", threadIdx.x, blockIdx.x, sphIDs[i],
            //        offset);
            if (offset >= numentries) {
                ABORTABORTABORT("TOO MANY SPHERE ENTRIES: expected max: %u, current: %u\n", numentries, offset);
            }
            // if (offset >= max_composite_index) {
            //     ABORTABORTABORT(
            //         "overrun during priming on thread %u block %u, offset is %zu, max is %zu,  sphere is %u\n",
            //         threadIdx.x, blockIdx.x, offset, max_composite_index, sphIDs[i]);
            // } else {
            // printf("%s\n", );
            sphere_data.spheres_in_SD_composite[offset] = sphIDs[i];
            // }
        }
    }
}

/**
This device function computes the forces induces by the walls on the box on a sphere
Input:
  - sphXpos: X location, measured in the box reference system, of the sphere
  - sphYpos: Y location, measured in the box reference system, of the sphere
  - sphZpos: Z location, measured in the box reference system, of the sphere
  - sphXvel: X velocity, measured in the box reference system, of the sphere
  - sphYvel: Y velocity, measured in the box reference system, of the sphere
  - sphZvel: Z velocity, measured in the box reference system, of the sphere

Output:
  - X_Vel_corr: the X component of the force, as represented in the box reference system
  - Y_Vel_corr: the Y component of the force, as represented in the box reference system
  - Z_Vel_corr: the Z component of the force, as represented in the box reference system
*/
inline __device__ void boxWallsEffects(const int sphXpos,      //!< Global X position of DE
                                       const int sphYpos,      //!< Global Y position of DE
                                       const int sphZpos,      //!< Global Z position of DE
                                       const float sphXvel,    //!< Global X velocity of DE
                                       const float sphYvel,    //!< Global Y velocity of DE
                                       const float sphZvel,    //!< Global Z velocity of DE
                                       const float sphOmegaX,  //!< Global X velocity of DE
                                       const float sphOmegaY,  //!< Global Y velocity of DE
                                       const float sphOmegaZ,  //!< Global Z velocity of DE
                                       float3& force_from_wall,
                                       float3& ang_acc_from_wall,
                                       GranParamsPtr gran_params) {
    // classic radius grab, but signed so we can negate it easier
    const signed int sphereRadius_SU = gran_params->sphereRadius_SU;

    // Shift frame so that origin is at lower left corner
    int sphXpos_modified = -gran_params->BD_frame_X + sphXpos;
    int sphYpos_modified = -gran_params->BD_frame_Y + sphYpos;
    int sphZpos_modified = -gran_params->BD_frame_Z + sphZpos;

    constexpr float m_eff = gran_params->sphere_mass_SU / 2.f;
    // cache force
    float3 wall_force = {0, 0, 0};
    // cache force divided by radius
    float3 wall_torque_by_r = {0, 0, 0};

    // velocity difference of COMs
    float3 delta_V_com = make_float3(sphXvel - gran_params->BD_frame_X_dot, sphYvel - gran_params->BD_frame_Y_dot,
                                     sphZvel - gran_params->BD_frame_Z_dot);
    // r times Omega, these are components of r cross omega
    float3 r_Omega = {sphereRadius_SU * sphOmegaX, sphereRadius_SU * sphOmegaY, sphereRadius_SU * sphOmegaZ};

    float3 normal_damping_force = delta_V_com * gran_params->Gamma_n_s2w_SU * m_eff;

    // TODO how does multistep work here?

    // scaling factors for tangential spring/damping forces
    float composite_t_fac = gran_params->K_t_s2w_SU * gran_params->stepSize_SU +
                            gran_params->Gamma_t_s2w_SU * m_eff;  // include time integration for now

    signed int penetration;

    // Todo could dump into float[3] and make this much easier to read

    // Do X direction
    // penetration of sphere into bottom X wall
    penetration = sphXpos_modified - sphereRadius_SU;
    // if leftmost part is below wall, we have contact
    if ((penetration < 0) && abs(penetration) < sphereRadius_SU) {
        float3 curr_contrib = {0, 0, 0};
        // positive (upwards) restorative force
        curr_contrib.x += gran_params->K_n_s2w_SU * abs(penetration);
        // damping always has this minus sign
        curr_contrib.x += -1 * normal_damping_force.x;
        // adhesion term
        curr_contrib.x += -1 * gran_params->adhesionAcc_s2w;

        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP ||
            gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
            // add tangential forces
            // w cross r = -r w_z e_y + r w_y e_z
            curr_contrib.y += -1 * (composite_t_fac * (delta_V_com.y - r_Omega.z));
            curr_contrib.z += -1 * (composite_t_fac * (delta_V_com.z + r_Omega.y));
            wall_torque_by_r = wall_torque_by_r + make_float3(0, curr_contrib.z, -curr_contrib.y);
        }
        wall_force = wall_force + curr_contrib;
    }

    // Do top X wall
    penetration = gran_params->max_x_pos_unsigned - (sphXpos_modified + sphereRadius_SU);
    // if leftmost part is below wall, we have contact
    if ((penetration < 0) && abs(penetration) < sphereRadius_SU) {
        float3 curr_contrib = {0, 0, 0};

        // negative (downwards) restorative force
        curr_contrib.x += -1 * gran_params->K_n_s2w_SU * abs(penetration);
        // damping always has this minus sign
        curr_contrib.x += -1 * normal_damping_force.x;
        // adhesion term
        curr_contrib.x += gran_params->adhesionAcc_s2w;

        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP ||
            gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
            // add tangential forces
            curr_contrib.y += -1 * (composite_t_fac * (delta_V_com.y + r_Omega.z));
            curr_contrib.z += -1 * (composite_t_fac * (delta_V_com.z - r_Omega.y));
            wall_torque_by_r = wall_torque_by_r + make_float3(0, -curr_contrib.z, curr_contrib.y);
        }
        wall_force = wall_force + curr_contrib;
    }

    // Do Y direction
    // penetration of sphere into bottom Y wall
    penetration = sphYpos_modified - sphereRadius_SU;
    // if leftmost part is below wall, we have contact
    if ((penetration < 0) && abs(penetration) < sphereRadius_SU) {
        float3 curr_contrib = {0, 0, 0};

        // positive (upwards) restorative force
        curr_contrib.y += gran_params->K_n_s2w_SU * abs(penetration);
        // damping always has this minus sign
        curr_contrib.y += -1 * normal_damping_force.y;
        // adhesion term
        curr_contrib.y += -1 * gran_params->adhesionAcc_s2w;

        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP ||
            gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
            // add tangential forces
            curr_contrib.z += -1 * (composite_t_fac * (delta_V_com.z - r_Omega.x));
            curr_contrib.x += -1 * (composite_t_fac * (delta_V_com.x + r_Omega.z));
            wall_torque_by_r = wall_torque_by_r + make_float3(-curr_contrib.z, 0, curr_contrib.x);
        }
        wall_force = wall_force + curr_contrib;
    }

    // Do top Y wall
    penetration = gran_params->max_y_pos_unsigned - (sphYpos_modified + sphereRadius_SU);
    // if leftmost part is below wall, we have contact
    if ((penetration < 0) && abs(penetration) < sphereRadius_SU) {
        float3 curr_contrib = {0, 0, 0};

        // negative (downwards) restorative force
        curr_contrib.y += -1 * gran_params->K_n_s2w_SU * abs(penetration);
        // damping always has this minus sign
        curr_contrib.y += -1 * normal_damping_force.y;
        // adhesion term
        curr_contrib.y += gran_params->adhesionAcc_s2w;

        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP ||
            gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
            // add tangential forces
            curr_contrib.z += -1 * (composite_t_fac * (delta_V_com.z + r_Omega.x));
            curr_contrib.x += -1 * (composite_t_fac * (delta_V_com.x - r_Omega.z));
            wall_torque_by_r = wall_torque_by_r + make_float3(curr_contrib.z, 0, -curr_contrib.x);
        }
        wall_force = wall_force + curr_contrib;
    }

    // Do Z direction
    // penetration of sphere into bottom Z wall
    penetration = sphZpos_modified - sphereRadius_SU;
    // if leftmost part is below wall, we have contact
    if ((penetration < 0) && abs(penetration) < sphereRadius_SU) {
        float3 curr_contrib = {0, 0, 0};

        // positive (upwards) restorative force
        curr_contrib.z += gran_params->K_n_s2w_SU * abs(penetration);
        // damping always has this minus sign
        curr_contrib.z += -1 * normal_damping_force.z;
        // adhesion term
        curr_contrib.z += -1 * gran_params->adhesionAcc_s2w;

        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP ||
            gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
            // add tangential forces
            curr_contrib.x += -1 * (composite_t_fac * (delta_V_com.x - r_Omega.y));
            curr_contrib.y += -1 * (composite_t_fac * (delta_V_com.y + r_Omega.x));
            wall_torque_by_r = wall_torque_by_r + make_float3(curr_contrib.y, -curr_contrib.x, 0);
        }
        wall_force = wall_force + curr_contrib;
    }

    // Do top Z wall
    penetration = gran_params->max_z_pos_unsigned - (sphZpos_modified + sphereRadius_SU);
    // if leftmost part is below wall, we have contact
    if ((penetration < 0) && abs(penetration) < sphereRadius_SU) {
        float3 curr_contrib = {0, 0, 0};

        // negative (downwards) restorative force
        curr_contrib.z += -1 * gran_params->K_n_s2w_SU * abs(penetration);
        // damping always has this minus sign
        curr_contrib.z += -1 * normal_damping_force.z;
        // adhesion term
        curr_contrib.z += gran_params->adhesionAcc_s2w;

        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP ||
            gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
            // add tangential forces
            curr_contrib.x += -1 * (composite_t_fac * (delta_V_com.x + r_Omega.y));
            curr_contrib.y += -1 * (composite_t_fac * (delta_V_com.y - r_Omega.x));
            wall_torque_by_r = wall_torque_by_r + make_float3(-curr_contrib.y, curr_contrib.x, 0);
        }
        wall_force = wall_force + curr_contrib;
    }

    force_from_wall = force_from_wall + wall_force;
    if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
        ang_acc_from_wall = ang_acc_from_wall + (wall_torque_by_r / gran_params->sphereInertia_by_r);
    }
}

// apply gravity to a sphere
inline __device__ void applyGravity(float3& sphere_force, GranParamsPtr gran_params) {
    sphere_force.x += gran_params->gravAcc_X_SU * gran_params->sphere_mass_SU;
    sphere_force.y += gran_params->gravAcc_Y_SU * gran_params->sphere_mass_SU;
    sphere_force.z += gran_params->gravAcc_Z_SU * gran_params->sphere_mass_SU;
}

/// Compute forces on a sphere from walls, BCs, and gravity
inline __device__ void applyExternalForces_frictionless(unsigned int currSphereID,
                                                        const int3& sphPos,    //!< Global X position of DE
                                                        const float3& sphVel,  //!< Global X velocity of DE
                                                        const float3& sphOmega,
                                                        float3& sphere_force,
                                                        float3& sphere_ang_acc,
                                                        GranParamsPtr gran_params,
                                                        sphereDataStruct sphere_data,
                                                        BC_type* bc_type_list,
                                                        BC_params_t<int, int3>* bc_params_list,
                                                        unsigned int nBCs) {
    // apply wall and BC effects
    // Perhaps this sphere is hitting the wall[s]
    boxWallsEffects(sphPos.x, sphPos.y, sphPos.z, sphVel.x, sphVel.y, sphVel.z, sphOmega.x, sphOmega.y, sphOmega.z,
                    sphere_force, sphere_ang_acc, gran_params);
    // add forces from each BC
    for (unsigned int BC_id = 0; BC_id < nBCs; BC_id++) {
        // skip inactive BCs
        if (!bc_params_list[BC_id].active) {
            continue;
        }
        switch (bc_type_list[BC_id]) {
            // case BC_type::AA_BOX: {
            //     ABORTABORTABORT("ERROR: AA_BOX is currently unsupported!\n");
            //     break;
            // }
            case BC_type::SPHERE: {
                addBCForces_Sphere(currSphereID, BC_id, sphPos, sphVel, sphOmega, sphere_force, sphere_ang_acc,
                                   gran_params, sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces);
                break;
            }
            case BC_type::CONE: {
                addBCForces_ZCone(currSphereID, BC_id, sphPos, sphVel, sphOmega, sphere_force, sphere_ang_acc,
                                  gran_params, sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces);
                break;
            }
            case BC_type::PLANE: {
                float dist = 0;  // tmp used to satisfy function parameters
                addBCForces_Plane_frictionless(currSphereID, BC_id, sphPos, sphVel, sphere_force, gran_params,
                                               sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces,
                                               dist);
                break;
            }
            case BC_type::CYLINDER: {
                addBCForces_Zcyl(currSphereID, BC_id, sphPos, sphVel, sphOmega, sphere_force, sphere_ang_acc,
                                 gran_params, sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces);
                break;
            }
        }
    }
    applyGravity(sphere_force, gran_params);
}

/// Compute forces on a sphere from walls, BCs, and gravity
inline __device__ void applyExternalForces(unsigned int currSphereID,
                                           const int3& sphPos,    //!< Global X position of DE
                                           const float3& sphVel,  //!< Global X velocity of DE
                                           const float3& sphOmega,
                                           float3& sphere_force,
                                           float3& sphere_ang_acc,
                                           GranParamsPtr gran_params,
                                           sphereDataStruct sphere_data,
                                           BC_type* bc_type_list,
                                           BC_params_t<int, int3>* bc_params_list,
                                           unsigned int nBCs) {
    // apply wall and BC effects
    // Perhaps this sphere is hitting the wall[s]
    boxWallsEffects(sphPos.x, sphPos.y, sphPos.z, sphVel.x, sphVel.y, sphVel.z, sphOmega.x, sphOmega.y, sphOmega.z,
                    sphere_force, sphere_ang_acc, gran_params);
    // add forces from each BC
    for (unsigned int BC_id = 0; BC_id < nBCs; BC_id++) {
        // skip inactive BCs
        if (!bc_params_list[BC_id].active) {
            continue;
        }
        switch (bc_type_list[BC_id]) {
            // case BC_type::AA_BOX: {
            //     ABORTABORTABORT("ERROR: AA_BOX is currently unsupported!\n");
            //     break;
            // }
            case BC_type::SPHERE: {
                addBCForces_Sphere(currSphereID, BC_id, sphPos, sphVel, sphOmega, sphere_force, sphere_ang_acc,
                                   gran_params, sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces);
                break;
            }
            case BC_type::CONE: {
                addBCForces_ZCone(currSphereID, BC_id, sphPos, sphVel, sphOmega, sphere_force, sphere_ang_acc,
                                  gran_params, sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces);
                break;
            }
            case BC_type::PLANE: {
                addBCForces_Plane(currSphereID, BC_id, sphPos, sphVel, sphOmega, sphere_force, sphere_ang_acc,
                                  gran_params, sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces);
                break;
            }
            case BC_type::CYLINDER: {
                addBCForces_Zcyl(currSphereID, BC_id, sphPos, sphVel, sphOmega, sphere_force, sphere_ang_acc,
                                 gran_params, sphere_data, bc_params_list[BC_id], bc_params_list[BC_id].track_forces);
                break;
            }
        }
    }
    applyGravity(sphere_force, gran_params);
}

static __global__ void determineContactPairs(sphereDataStruct sphere_data, GranParamsPtr gran_params) {
    // Cache positions and velocities in shared memory, this doesn't hurt occupancy at the moment
    __shared__ int3 sphere_pos[MAX_COUNT_OF_SPHERES_PER_SD];
    __shared__ unsigned int sphIDs[MAX_COUNT_OF_SPHERES_PER_SD];

    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = sphere_data.SD_NumSpheresTouching[thisSD];

    unsigned char bodyB_list[MAX_SPHERES_TOUCHED_BY_SPHERE];
    unsigned int ncontacts = 0;

    if (spheresTouchingThisSD == 0) {
        return;  // no spheres here, move along
    }

    // If we overran, we have a major issue, time to crash before we make illegal memory accesses
    if (threadIdx.x == 0 && spheresTouchingThisSD > MAX_COUNT_OF_SPHERES_PER_SD) {
        // Crash now
        ABORTABORTABORT("TOO MANY SPHERES! SD %u has %u spheres\n", thisSD, spheresTouchingThisSD);
    }

    // Bring in data from global into shmem. Only a subset of threads get to do this.
    // Note that we're not using shared memory very heavily, so our bandwidth is pretty low
    if (threadIdx.x < spheresTouchingThisSD) {
        // We need int64_ts to index into composite array
        size_t offset_in_composite_Array = sphere_data.SD_SphereCompositeOffsets[thisSD] + threadIdx.x;
        unsigned int mySphereID = sphere_data.spheres_in_SD_composite[offset_in_composite_Array];
        sphere_pos[threadIdx.x] =
            make_int3(sphere_data.pos_X[mySphereID], sphere_data.pos_Y[mySphereID], sphere_data.pos_Z[mySphereID]);
        sphIDs[threadIdx.x] = mySphereID;
    }

    __syncthreads();  // Needed to make sure data gets in shmem before using it elsewhere

    // Assumes each thread is a body, not the greatest assumption but we can fix that later
    // Note that if we have more threads than bodies, some effort gets wasted.
    unsigned int bodyA = threadIdx.x;

    // Each body looks at each other body and determines whether that body is touching it
    if (bodyA < spheresTouchingThisSD) {
        for (unsigned char bodyB = 0; bodyB < spheresTouchingThisSD; bodyB++) {
            if (bodyA == bodyB)
                continue;

            bool active_contact = checkSpheresContacting_int(sphere_pos[bodyA], sphere_pos[bodyB], thisSD, gran_params);

            // We have a collision here, log it for later
            // not very divergent, super quick
            if (active_contact) {
                if (ncontacts >= MAX_SPHERES_TOUCHED_BY_SPHERE) {
                    ABORTABORTABORT("Sphere %u is touching 12 spheres already and we just found another!!!\n",
                                    sphIDs[bodyA]);
                }
                bodyB_list[ncontacts] = bodyB;  // Save the collision pair
                ncontacts++;                    // Increment the contact counter
            }
        }
        // for each contact we just found, mark it in the global map
        for (unsigned char contact_id = 0; contact_id < ncontacts; contact_id++) {
            // find and mark a spot in the contact map
            findContactPairInfo(sphere_data.sphere_contact_map, gran_params, sphIDs[bodyA],
                                sphIDs[bodyB_list[contact_id]]);
        }
    }
}

/// Compute normal forces for a contacting pair
// returns the normal force and sets the reciplength, tangent velocity, and delta_r
inline __device__ float3 computeSphereNormalForces(float& reciplength,
                                                   float3& vrel_t,
                                                   float3& delta_r,
                                                   const int3 sphereA_pos,
                                                   const int3 sphereB_pos,
                                                   const float3 sphereA_vel,
                                                   const float3 sphereB_vel,
                                                   GranParamsPtr gran_params) {
    float sphdiameter = 2. * gran_params->sphereRadius_SU;

    // compute penetrations in double
    {
        double invSphDiameter = 1. / (2. * gran_params->sphereRadius_SU);
        double3 delta_r_double = int3_to_double3(sphereA_pos - sphereB_pos) * invSphDiameter;
        // compute in double then convert to float
        reciplength = rsqrt(Dot(delta_r_double, delta_r_double));
    }

    // Compute penetration term, this becomes the delta as we want it
    float penetration = reciplength - 1.;

    float force_model_multiplier = get_force_multiplier(penetration, gran_params);

    // compute these in float now
    delta_r = int3_to_float3(sphereA_pos - sphereB_pos) / sphdiameter;

    // Velocity difference, it's better to do a coalesced access here than a fragmented access inside
    float3 v_rel = sphereA_vel - sphereB_vel;

    // Compute force updates for damping term
    // Project relative velocity to the normal
    // n = delta_r * reciplength
    // proj = Dot(delta_dot, n)
    float projection = Dot(v_rel, delta_r) * reciplength;

    // delta_dot = proj * n
    float3 vrel_n = projection * delta_r * reciplength;
    vrel_t = v_rel - vrel_n;

    constexpr float m_eff = gran_params->sphere_mass_SU / 2.f;
    // multiplier caused by Hooke vs Hertz force model

    // Add damping term
    // add spring term
    float3 force_accum = gran_params->K_n_s2s_SU * delta_r * sphdiameter * penetration * force_model_multiplier;
    force_accum = force_accum - gran_params->Gamma_n_s2s_SU * vrel_n * m_eff * force_model_multiplier;
    return force_accum;
}

/// each thread is a sphere, computing the forces its contact partners exert on it
static __global__ void computeSphereContactForces(sphereDataStruct sphere_data,
                                                  GranParamsPtr gran_params,
                                                  BC_type* bc_type_list,
                                                  BC_params_t<int, int3>* bc_params_list,
                                                  unsigned int nBCs,
                                                  unsigned int nSpheres) {
    // grab the sphere radius
    const unsigned int sphereRadius_SU = gran_params->sphereRadius_SU;

    // this sphere's coordinates
    int3 my_sphere_pos;

    // TODO figure out how we can do this better with no friction
    float3 my_omega;

    float3 my_sphere_vel;

    // my sphere ID, we're using a 1D thread->sphere map
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // my offset in the contact map
    size_t body_A_offset = MAX_SPHERES_TOUCHED_BY_SPHERE * mySphereID;

    // don't overrun the array
    if (mySphereID < nSpheres) {
        // printf("sphere %u is active\n", mySphereID);
        // Bring in data from global
        my_sphere_pos =
            make_int3(sphere_data.pos_X[mySphereID], sphere_data.pos_Y[mySphereID], sphere_data.pos_Z[mySphereID]);
        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            my_omega = make_float3(sphere_data.sphere_Omega_X[mySphereID], sphere_data.sphere_Omega_Y[mySphereID],
                                   sphere_data.sphere_Omega_Z[mySphereID]);
        }

        my_sphere_vel = make_float3(sphere_data.pos_X_dt[mySphereID], sphere_data.pos_Y_dt[mySphereID],
                                    sphere_data.pos_Z_dt[mySphereID]);

        // Now compute the force each contact partner exerts
        // Force applied to this sphere
        float3 bodyA_force = {0.f, 0.f, 0.f};
        float3 bodyA_AngAcc = {0.f, 0.f, 0.f};

        // for each sphere contacting me, compute the forces
        for (unsigned char contact_id = 0; contact_id < MAX_SPHERES_TOUCHED_BY_SPHERE; contact_id++) {
            // who am I colliding with?
            bool active_contact = sphere_data.sphere_contact_map[body_A_offset + contact_id].active;

            if (active_contact) {
                unsigned int theirSphereID = sphere_data.sphere_contact_map[body_A_offset + contact_id].body_B;

                if (theirSphereID >= nSpheres) {
                    ABORTABORTABORT("Invalid other sphere id found for sphere %u at slot %u, other is %u\n", mySphereID,
                                    contact_id, theirSphereID);
                }

                float3 vrel_t;      // tangent relative velocity
                float reciplength;  // used to compute contact normal
                float3 delta_r;     // used for contact normal
                float3 force_accum = computeSphereNormalForces(
                    reciplength, vrel_t, delta_r, my_sphere_pos,
                    make_int3(sphere_data.pos_X[theirSphereID], sphere_data.pos_Y[theirSphereID],
                              sphere_data.pos_Z[theirSphereID]),
                    my_sphere_vel,
                    make_float3(sphere_data.pos_X_dt[theirSphereID], sphere_data.pos_Y_dt[theirSphereID],
                                sphere_data.pos_Z_dt[theirSphereID]),
                    gran_params);

                // TODO fix this
                constexpr float force_model_multiplier = 1;
                constexpr float m_eff = gran_params->sphere_mass_SU / 2.f;

                // add frictional terms, if needed
                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    float3 their_omega = make_float3(sphere_data.sphere_Omega_X[theirSphereID],
                                                     sphere_data.sphere_Omega_Y[theirSphereID],
                                                     sphere_data.sphere_Omega_Z[theirSphereID]);
                    // (omega_b cross r_b - omega_a cross r_a), where r_b  = -r_a = delta_r * radius
                    // add tangential components if they exist, these are automatically tangential from the cross
                    // product
                    vrel_t = vrel_t + Cross((my_omega + their_omega), -1.f * delta_r * sphereRadius_SU);

                    // accumulator for tangent force
                    bool clamped = false;
                    float3 delta_t = {0, 0, 0};

                    // TODO improve this
                    // calculate tangential forces -- NOTE that these only come into play with friction enabled
                    if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP) {
                        // assume the contact is just this timestep
                        delta_t = vrel_t * gran_params->stepSize_SU;
                        // just in case this gets too big
                        clamped = clampTangentDisplacement(gran_params, force_accum, delta_t);
                    } else if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
                        // get the tangential displacement so far
                        delta_t = sphere_data.contact_history_map[body_A_offset + contact_id];
                        // add on what we have
                        delta_t = delta_t + vrel_t * gran_params->stepSize_SU;

                        // project onto contact normal
                        float projection = Dot(delta_t, delta_r) * reciplength;
                        // remove normal projection
                        delta_t = delta_t - projection * delta_r * reciplength;
                        // clamp tangent displacement
                        clamped = clampTangentDisplacement(gran_params, force_accum, delta_t);

                        // write back the updated displacement
                        sphere_data.contact_history_map[body_A_offset + contact_id] = delta_t;
                    }

                    float3 tangent_force = -gran_params->K_t_s2s_SU * delta_t * force_model_multiplier;

                    // if no-slip, add the tangential damping
                    if (!clamped) {
                        tangent_force = tangent_force - gran_params->Gamma_t_s2s_SU * m_eff * vrel_t;
                    }
                    // tau = r cross f = radius * n cross F
                    // 2 * radius * n = -1 * delta_r * sphdiameter
                    // assume abs(r) ~ radius, so n = delta_r
                    // compute accelerations caused by torques on body
                    bodyA_AngAcc = bodyA_AngAcc + Cross(-1 * delta_r, tangent_force) / gran_params->sphereInertia_by_r;
                    // add to total forces
                    force_accum = force_accum + tangent_force;
                }

                // Add cohesion term
                force_accum =
                    force_accum - gran_params->sphere_mass_SU * gran_params->cohesionAcc_s2s * delta_r * reciplength;

                // finally, we add this per-contact accumulator to the total force
                bodyA_force = bodyA_force + force_accum;
            }
        }

        // add in gravity and wall forces
        applyExternalForces(mySphereID, my_sphere_pos, my_sphere_vel, my_omega, bodyA_force, bodyA_AngAcc, gran_params,
                            sphere_data, bc_type_list, bc_params_list, nBCs);

        // Write the force back to global memory so that we can apply them AFTER this kernel finishes
        atomicAdd(sphere_data.sphere_acc_X + mySphereID, bodyA_force.x / gran_params->sphere_mass_SU);
        atomicAdd(sphere_data.sphere_acc_Y + mySphereID, bodyA_force.y / gran_params->sphere_mass_SU);
        atomicAdd(sphere_data.sphere_acc_Z + mySphereID, bodyA_force.z / gran_params->sphere_mass_SU);

        if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP ||
            gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
            atomicAdd(sphere_data.sphere_ang_acc_X + mySphereID, bodyA_AngAcc.x);
            atomicAdd(sphere_data.sphere_ang_acc_Y + mySphereID, bodyA_AngAcc.y);
            atomicAdd(sphere_data.sphere_ang_acc_Z + mySphereID, bodyA_AngAcc.z);
        }
    }
}

static __global__ void computeSphereForces_frictionless(sphereDataStruct sphere_data,
                                                        GranParamsPtr gran_params,
                                                        BC_type* bc_type_list,
                                                        BC_params_t<int, int3>* bc_params_list,
                                                        unsigned int nBCs) {
    // Cache positions and velocities in shared memory, this doesn't hurt occupancy at the moment
    __shared__ int3 sphere_pos[MAX_COUNT_OF_SPHERES_PER_SD];
    __shared__ float3 sphere_vel[MAX_COUNT_OF_SPHERES_PER_SD];

    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = sphere_data.SD_NumSpheresTouching[thisSD];
    unsigned int mySphereID;
    unsigned char bodyB_list[MAX_SPHERES_TOUCHED_BY_SPHERE];
    unsigned int ncontacts = 0;

    if (spheresTouchingThisSD == 0) {
        return;  // no spheres here, move along
    }

    // If we overran, we have a major issue, time to crash before we make illegal memory accesses
    if (threadIdx.x == 0 && spheresTouchingThisSD > MAX_COUNT_OF_SPHERES_PER_SD) {
        // Crash now
        ABORTABORTABORT("TOO MANY SPHERES! SD %u has %u spheres\n", thisSD, spheresTouchingThisSD);
    }

    // Bring in data from global into shmem. Only a subset of threads get to do this.
    // Note that we're not using shared memory very heavily, so our bandwidth is pretty low
    if (threadIdx.x < spheresTouchingThisSD) {
        // We need int64_ts to index into composite array
        size_t offset_in_composite_Array = sphere_data.SD_SphereCompositeOffsets[thisSD] + threadIdx.x;
        mySphereID = sphere_data.spheres_in_SD_composite[offset_in_composite_Array];
        sphere_pos[threadIdx.x] =
            make_int3(sphere_data.pos_X[mySphereID], sphere_data.pos_Y[mySphereID], sphere_data.pos_Z[mySphereID]);
        sphere_vel[threadIdx.x] = make_float3(sphere_data.pos_X_dt[mySphereID], sphere_data.pos_Y_dt[mySphereID],
                                              sphere_data.pos_Z_dt[mySphereID]);
    }

    __syncthreads();  // Needed to make sure data gets in shmem before using it elsewhere

    // Assumes each thread is a body, not the greatest assumption but we can fix that later
    // Note that if we have more threads than bodies, some effort gets wasted.
    unsigned int bodyA = threadIdx.x;

    // Each body looks at each other body and computes the force that the other body exerts on it
    if (bodyA < spheresTouchingThisSD) {
        // Force generated by this contact
        float3 bodyA_force = {0.f, 0.f, 0.f};

        for (unsigned char bodyB = 0; bodyB < spheresTouchingThisSD; bodyB++) {
            if (bodyA == bodyB)
                continue;

            bool active_contact = checkSpheresContacting_int(sphere_pos[bodyA], sphere_pos[bodyB], thisSD, gran_params);

            // We have a collision here, log it for later
            // not very divergent, super quick
            if (active_contact) {
                if (ncontacts >= MAX_SPHERES_TOUCHED_BY_SPHERE) {
                    ABORTABORTABORT("Sphere %u is touching 12 spheres already and we just found another!!!\n",
                                    mySphereID);
                }
                bodyB_list[ncontacts] = bodyB;  // Save the collision pair
                ncontacts++;                    // Increment the contact counter
            }
        }

        // NOTE that below here I used double precision because I didn't know how much precision I needed.
        // Reducing the amount of doubles will certainly speed this up Run through and do actual force
        // computations, for these we know each one is a legit collision
        for (unsigned int idx = 0; idx < ncontacts; idx++) {
            // who am I colliding with?
            unsigned char bodyB = bodyB_list[idx];

            float3 vrel_t;      // unused but needed for function signature
            float reciplength;  // used to compute contact normal
            float3 delta_r;     // used for contact normal
            float3 force_accum =
                computeSphereNormalForces(reciplength, vrel_t, delta_r, sphere_pos[bodyA], sphere_pos[bodyB],
                                          sphere_vel[bodyA], sphere_vel[bodyB], gran_params);

            // Add cohesion term
            force_accum =
                force_accum - gran_params->sphere_mass_SU * gran_params->cohesionAcc_s2s * delta_r * reciplength;
            bodyA_force = bodyA_force + force_accum;
        }

        // IMPORTANT: Make sure that the sphere belongs to *this* SD, otherwise we'll end up with double
        // counting this force. If this SD owns the body, add its wall, BC, and grav forces

        unsigned int ownerSD = SDTripletID(
            pointSDTriplet(sphere_pos[bodyA].x, sphere_pos[bodyA].y, sphere_pos[bodyA].z, gran_params), gran_params);
        if (ownerSD == thisSD) {
            // NOTE these are unused but the device function wants them
            float3 bodyA_AngAcc;
            constexpr float3 sphere_omega = {0, 0, 0};

            applyExternalForces_frictionless(mySphereID, sphere_pos[bodyA], sphere_vel[bodyA], sphere_omega,
                                             bodyA_force, bodyA_AngAcc, gran_params, sphere_data, bc_type_list,
                                             bc_params_list, nBCs);
        }

        // Write the force back to global memory so that we can apply them AFTER this kernel finishes
        atomicAdd(sphere_data.sphere_acc_X + mySphereID, bodyA_force.x / gran_params->sphere_mass_SU);
        atomicAdd(sphere_data.sphere_acc_Y + mySphereID, bodyA_force.y / gran_params->sphere_mass_SU);
        atomicAdd(sphere_data.sphere_acc_Z + mySphereID, bodyA_force.z / gran_params->sphere_mass_SU);
    }
}

// Compute update for a quantity using Forward Euler
inline __device__ float integrateForwardEuler(float stepsize_SU, float val_dt) {
    return stepsize_SU * val_dt;
}

// Compute update for a velocity using Chung
inline __device__ float integrateChung_vel(float stepsize_SU, float acc, float acc_old) {
    constexpr float gamma_hat = -1.f / 2.f;
    constexpr float gamma = 3.f / 2.f;
    return stepsize_SU * (acc * gamma + acc_old * gamma_hat);
}

// Compute update for a position using Chung
inline __device__ float integrateChung_pos(float stepsize_SU, float vel_old, float acc, float acc_old) {
    constexpr float beta = 28.f / 27.f;
    constexpr float beta_hat = .5 - beta;
    return stepsize_SU * (vel_old + stepsize_SU * (acc * beta + acc_old * beta_hat));
}

// Compute update for a velocity using Velocity Verlet
inline __device__ float integrateVelVerlet_vel(float stepsize_SU, float acc, float acc_old) {
    return 0.5f * stepsize_SU * (acc + acc_old);
}

// Compute update for a position using Velocity Verlet
inline __device__ float integrateVelVerlet_pos(float stepsize_SU, float vel_old, float acc_old) {
    return stepsize_SU * (vel_old + 0.5f * stepsize_SU * acc_old);
}

/**
 * Numerically integrates force to velocity and velocity to position, then computes the broadphase on the new
 * positions
 */
static __global__ void updatePositions(const float stepsize_SU,  //!< The numerical integration time step
                                       sphereDataStruct sphere_data,
                                       unsigned int nSpheres,
                                       GranParamsPtr gran_params) {
    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid
    // structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // if we're in multistep mode, clean up contact histories
    if (mySphereID < nSpheres && gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
        cleanupContactMap(sphere_data, mySphereID, gran_params);
    }
    __syncthreads();  // just in case

    // Write back velocity updates
    if (mySphereID < nSpheres) {
        // Check to see if we messed up badly somewhere
        if (sphere_data.sphere_acc_X[mySphereID] == NAN || sphere_data.sphere_acc_Y[mySphereID] == NAN ||
            sphere_data.sphere_acc_Z[mySphereID] == NAN) {
            ABORTABORTABORT("NAN force computed -- sphere is %u, x force is %f\n", mySphereID,
                            sphere_data.sphere_acc_X[mySphereID]);
        }

        float v_update_X = 0;
        float v_update_Y = 0;
        float v_update_Z = 0;

        float omega_update_X = 0;
        float omega_update_Y = 0;
        float omega_update_Z = 0;

        float old_vel_X = sphere_data.pos_X_dt[mySphereID];
        float old_vel_Y = sphere_data.pos_Y_dt[mySphereID];
        float old_vel_Z = sphere_data.pos_Z_dt[mySphereID];

        // no divergence, same for every thread in block
        switch (gran_params->time_integrator) {
            case chrono::granular::GRAN_TIME_INTEGRATOR::FORWARD_EULER: {
                v_update_X = integrateForwardEuler(stepsize_SU, sphere_data.sphere_acc_X[mySphereID]);
                v_update_Y = integrateForwardEuler(stepsize_SU, sphere_data.sphere_acc_Y[mySphereID]);
                v_update_Z = integrateForwardEuler(stepsize_SU, sphere_data.sphere_acc_Z[mySphereID]);

                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    // tau = I alpha => alpha = tau / I, we already computed these alphas
                    omega_update_X = integrateForwardEuler(stepsize_SU, sphere_data.sphere_ang_acc_X[mySphereID]);
                    omega_update_Y = integrateForwardEuler(stepsize_SU, sphere_data.sphere_ang_acc_Y[mySphereID]);
                    omega_update_Z = integrateForwardEuler(stepsize_SU, sphere_data.sphere_ang_acc_Z[mySphereID]);
                }
                break;
            }
            case chrono::granular::GRAN_TIME_INTEGRATOR::CHUNG: {
                v_update_X = integrateChung_vel(stepsize_SU, sphere_data.sphere_acc_X[mySphereID],
                                                sphere_data.sphere_acc_X_old[mySphereID]);
                v_update_Y = integrateChung_vel(stepsize_SU, sphere_data.sphere_acc_Y[mySphereID],
                                                sphere_data.sphere_acc_Y_old[mySphereID]);
                v_update_Z = integrateChung_vel(stepsize_SU, sphere_data.sphere_acc_Z[mySphereID],
                                                sphere_data.sphere_acc_Z_old[mySphereID]);

                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    omega_update_X = integrateChung_vel(stepsize_SU, sphere_data.sphere_ang_acc_X[mySphereID],
                                                        sphere_data.sphere_ang_acc_X_old[mySphereID]);
                    omega_update_Y = integrateChung_vel(stepsize_SU, sphere_data.sphere_ang_acc_Y[mySphereID],
                                                        sphere_data.sphere_ang_acc_Y_old[mySphereID]);
                    omega_update_Z = integrateChung_vel(stepsize_SU, sphere_data.sphere_ang_acc_Z[mySphereID],
                                                        sphere_data.sphere_ang_acc_Z_old[mySphereID]);
                }
                break;
            }
            case chrono::granular::GRAN_TIME_INTEGRATOR::VELOCITY_VERLET: {
                v_update_X = integrateVelVerlet_vel(stepsize_SU, sphere_data.sphere_force_X[mySphereID],
                                                    sphere_data.sphere_force_X_old[mySphereID], gran_params);
                v_update_Y = integrateVelVerlet_vel(stepsize_SU, sphere_data.sphere_force_Y[mySphereID],
                                                    sphere_data.sphere_force_Y_old[mySphereID], gran_params);
                v_update_Z = integrateVelVerlet_vel(stepsize_SU, sphere_data.sphere_force_Z[mySphereID],
                                                    sphere_data.sphere_force_Z_old[mySphereID], gran_params);

                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    omega_update_X = integrateVelVerlet_vel(stepsize_SU, sphere_data.sphere_ang_acc_X[mySphereID],
                                                            sphere_data.sphere_ang_acc_X_old[mySphereID], gran_params);
                    omega_update_Y = integrateVelVerlet_vel(stepsize_SU, sphere_data.sphere_ang_acc_Y[mySphereID],
                                                            sphere_data.sphere_ang_acc_Y_old[mySphereID], gran_params);
                    omega_update_Z = integrateVelVerlet_vel(stepsize_SU, sphere_data.sphere_ang_acc_Z[mySphereID],
                                                            sphere_data.sphere_ang_acc_Z_old[mySphereID], gran_params);
                    // TODO test
                }
                break;
            }
        }

        // write back the updates
        sphere_data.pos_X_dt[mySphereID] += v_update_X;
        sphere_data.pos_Y_dt[mySphereID] += v_update_Y;
        sphere_data.pos_Z_dt[mySphereID] += v_update_Z;

        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            sphere_data.sphere_Omega_X[mySphereID] += omega_update_X;
            sphere_data.sphere_Omega_Y[mySphereID] += omega_update_Y;
            sphere_data.sphere_Omega_Z[mySphereID] += omega_update_Z;
        }

        float position_update_x = 0;
        float position_update_y = 0;
        float position_update_z = 0;
        // no divergence, same for every thread in block
        switch (gran_params->time_integrator) {
            case chrono::granular::GRAN_TIME_INTEGRATOR::FORWARD_EULER: {
                position_update_x = integrateForwardEuler(stepsize_SU, sphere_data.pos_X_dt[mySphereID]);
                position_update_y = integrateForwardEuler(stepsize_SU, sphere_data.pos_Y_dt[mySphereID]);
                position_update_z = integrateForwardEuler(stepsize_SU, sphere_data.pos_Z_dt[mySphereID]);
                break;
            }
            case chrono::granular::GRAN_TIME_INTEGRATOR::CHUNG: {


                position_update_x = integrateChung_pos(stepsize_SU, old_vel_X, sphere_data.sphere_acc_X[mySphereID],
                                                       sphere_data.sphere_acc_X_old[mySphereID]);
                position_update_y = integrateChung_pos(stepsize_SU, old_vel_Y, sphere_data.sphere_acc_Y[mySphereID],
                                                       sphere_data.sphere_acc_Y_old[mySphereID]);
                position_update_z = integrateChung_pos(stepsize_SU, old_vel_Z, sphere_data.sphere_acc_Z[mySphereID],
                                                       sphere_data.sphere_acc_Z_old[mySphereID]);
                break;
            }
            case chrono::granular::GRAN_TIME_INTEGRATOR::VELOCITY_VERLET: {

                position_update_x = integrateForwardEuler(stepsize_SU, old_vel_X + v_update_X);
                position_update_y = integrateForwardEuler(stepsize_SU, old_vel_Y + v_update_Y);
                position_update_z = integrateForwardEuler(stepsize_SU, old_vel_Z + v_update_Z);
                break;
            }
        }
        // Perform numerical integration. Hitting cache, also coalesced.
        sphere_data.pos_X[mySphereID] += position_update_x;
        sphere_data.pos_Y[mySphereID] += position_update_y;
        sphere_data.pos_Z[mySphereID] += position_update_z;
    }
}
