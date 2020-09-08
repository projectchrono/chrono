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
// Holds of helper functions for GPU granular code that need to be scoped higher
//
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"

#include "chrono_thirdparty/cub/cub.cuh"

using chrono::granular::GRAN_TIME_INTEGRATOR;
using chrono::granular::GRAN_FRICTION_MODE;
using chrono::granular::GRAN_ROLLING_MODE;

// Print a user-given error message and crash
#define ABORTABORTABORT(...) \
    {                        \
        printf(__VA_ARGS__); \
        __threadfence();     \
        cub::ThreadTrap();   \
    }

#define GRAN_DEBUG_PRINTF(...) printf(__VA_ARGS__)

// Decide which SD owns this point in space
// Pass it the Center of Mass location for a DE to get its owner, also used to get contact point
inline __device__ int3 pointSDTriplet(int64_t sphCenter_X,
                                      int64_t sphCenter_Y,
                                      int64_t sphCenter_Z,
                                      GranParamsPtr gran_params) {
    // Note that this offset allows us to have moving walls and the like very easily

    int64_t sphCenter_X_modified = -gran_params->BD_frame_X + sphCenter_X;
    int64_t sphCenter_Y_modified = -gran_params->BD_frame_Y + sphCenter_Y;
    int64_t sphCenter_Z_modified = -gran_params->BD_frame_Z + sphCenter_Z;
    // printf("PST: global is %lld, %lld, %lld, modified is %lld, %lld, %lld\n", sphCenter_X, sphCenter_Y, sphCenter_Z,
    //        sphCenter_X_modified, sphCenter_Y_modified, sphCenter_Z_modified);
    int3 n;
    // Get the SD of the sphere's center in the xdir
    n.x = (sphCenter_X_modified / (int64_t)gran_params->SD_size_X_SU);
    // Same for D and H
    n.y = (sphCenter_Y_modified / (int64_t)gran_params->SD_size_Y_SU);
    n.z = (sphCenter_Z_modified / (int64_t)gran_params->SD_size_Z_SU);
    return n;
}

// Decide which SD owns this point in space
// Short form overload for regular ints
inline __device__ int3 pointSDTriplet(int sphCenter_X, int sphCenter_Y, int sphCenter_Z, GranParamsPtr gran_params) {
    // call the 64-bit overload
    return pointSDTriplet((int64_t)sphCenter_X, (int64_t)sphCenter_Y, (int64_t)sphCenter_Z, gran_params);
}

// Decide which SD owns this point in space
// overload for doubles (used in triangle code)
inline __device__ int3 pointSDTriplet(double sphCenter_X,
                                      double sphCenter_Y,
                                      double sphCenter_Z,
                                      GranParamsPtr gran_params) {
    // call the 64-bit overload
    return pointSDTriplet((int64_t)sphCenter_X, (int64_t)sphCenter_Y, (int64_t)sphCenter_Z, gran_params);
}

// Conver SD ID to SD triplet
inline __host__ __device__ int3 SDIDTriplet(unsigned int SD_ID, GranParamsPtr gran_params) {
    int3 SD_trip = {0, 0, 0};

    // printf("ID is %u\n", SD_ID);
    // find X component
    SD_trip.x = SD_ID / (gran_params->nSDs_Y * gran_params->nSDs_Z);

    // subtract off the x contribution
    SD_ID -= SD_trip.x * gran_params->nSDs_Y * gran_params->nSDs_Z;
    // printf("x is %d, ID is %u\n", SD_trip.x, SD_ID);
    // find y component
    SD_trip.y = SD_ID / gran_params->nSDs_Z;
    // subtract off the y contribution
    SD_ID -= SD_trip.y * gran_params->nSDs_Z;
    // printf("y is %d, ID is %u\n", SD_trip.y, SD_ID);

    // find z component
    SD_trip.z = SD_ID;

    return SD_trip;
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const int i, const int j, const int k, GranParamsPtr gran_params) {
    // if we're outside the BD in any direction, this is an invalid SD
    if (i < 0 || i >= gran_params->nSDs_X) {
        return NULL_GRANULAR_ID;
    }
    if (j < 0 || j >= gran_params->nSDs_Y) {
        return NULL_GRANULAR_ID;
    }
    if (k < 0 || k >= gran_params->nSDs_Z) {
        return NULL_GRANULAR_ID;
    }
    return i * gran_params->nSDs_Y * gran_params->nSDs_Z + j * gran_params->nSDs_Z + k;
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const int3& trip, GranParamsPtr gran_params) {
    return SDTripletID(trip.x, trip.y, trip.z, gran_params);
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const int trip[3], GranParamsPtr gran_params) {
    return SDTripletID(trip[0], trip[1], trip[2], gran_params);
}

/// get an index for the current contact pair
inline __device__ size_t findContactPairInfo(GranSphereDataPtr sphere_data,
                                             GranParamsPtr gran_params,
                                             unsigned int body_A,
                                             unsigned int body_B) {
    // TODO this should be size_t everywhere
    size_t body_A_offset = (size_t)MAX_SPHERES_TOUCHED_BY_SPHERE * body_A;
    // first skim through and see if this contact pair is in the map
    for (unsigned int contact_id = 0; contact_id < MAX_SPHERES_TOUCHED_BY_SPHERE; contact_id++) {
        size_t contact_index = body_A_offset + contact_id;
        if (sphere_data->contact_partners_map[contact_index] == body_B) {
            // make sure this contact is marked active
            sphere_data->contact_active_map[contact_index] = true;
            return contact_index;
        }
    }

    // if we get this far, the contact pair isn't in the map now and we need to find an empty spot
    for (unsigned int contact_id = 0; contact_id < MAX_SPHERES_TOUCHED_BY_SPHERE; contact_id++) {
        size_t contact_index = body_A_offset + contact_id;
        // check whether the slot is free right now
        if (sphere_data->contact_partners_map[contact_index] == NULL_GRANULAR_ID) {
            // claim this slot for ourselves, atomically
            // if the CAS returns NULL_GRANULAR_ID, it means that the spot was free and we claimed it
            unsigned int body_B_returned =
                atomicCAS(sphere_data->contact_partners_map + contact_index, NULL_GRANULAR_ID, body_B);
            // did we get the spot? if so, claim it
            if (NULL_GRANULAR_ID == body_B_returned) {
                // make sure this contact is marked active
                sphere_data->contact_active_map[contact_index] = true;
                return contact_index;
            }
        }
    }

    // if we got this far, we couldn't find a free contact pair. That is a violation of the 12-contacts theorem, so
    // we should probably give up now
    ABORTABORTABORT("No available contact pair slots for body %u and body %u\n", body_A, body_B);
    return NULL_GRANULAR_ID;  // shouldn't get here anyways
}

/// cleanup the contact data for a given body
inline __device__ void cleanupContactMap(GranSphereDataPtr sphere_data,
                                         unsigned int body_A,
                                         GranParamsPtr gran_params) {
    // index of the sphere into the big array
    size_t body_A_offset = (size_t)MAX_SPHERES_TOUCHED_BY_SPHERE * body_A;

    // get offsets into the global pointers
    float3* contact_history = sphere_data->contact_history_map + body_A_offset;
    unsigned int* contact_partners = sphere_data->contact_partners_map + body_A_offset;
    not_stupid_bool* contact_active = sphere_data->contact_active_map + body_A_offset;

    // sweep through each available contact slot and reset it if that slot wasn't active last timestep
    for (unsigned int contact_id = 0; contact_id < MAX_SPHERES_TOUCHED_BY_SPHERE; contact_id++) {
        // printf("contact map for sphere %u entry %u is other %u, active %u \t history is %f, %f, %f\n", body_A,
        //        contact_id, contact_partners[contact_id], contact_active[contact_id], contact_history[contact_id].x,
        //        contact_history[contact_id].y, contact_history[contact_id].z);
        // if the contact is not active, reset it
        if (contact_active[contact_id] == false) {
            contact_partners[contact_id] = NULL_GRANULAR_ID;
            if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
                constexpr float3 null_history = {0, 0, 0};
                contact_history[contact_id] = null_history;
            }
        } else {
            // otherwise reset the active bit for the next step
            contact_active[contact_id] = false;
        }
    }
}

inline __device__ bool checkLocalPointInSD(const int3& point, GranParamsPtr gran_params) {
    // TODO verify that this is correct
    // TODO optimize me
    bool ret = (point.x >= 0) && (point.y >= 0) && (point.z >= 0);
    ret = ret && (point.x <= gran_params->SD_size_X_SU) && (point.y <= gran_params->SD_size_Y_SU) &&
          (point.z <= gran_params->SD_size_Z_SU);
    return ret;
}
/// in integer, check whether a pair of spheres is in contact
inline __device__ bool checkSpheresContacting_int(const int3& sphereA_pos,
                                                  const int3& sphereB_pos,
                                                  unsigned int thisSD,
                                                  GranParamsPtr gran_params) {
    // Compute penetration to check for collision, we can use ints provided the diameter is small enough
    int64_t penetration_int = 0;

    // This avoids computing a square to figure our if collision or not
    int64_t deltaX = (sphereA_pos.x - sphereB_pos.x);
    int64_t deltaY = (sphereA_pos.y - sphereB_pos.y);
    int64_t deltaZ = (sphereA_pos.z - sphereB_pos.z);

    penetration_int = deltaX * deltaX;
    penetration_int += deltaY * deltaY;
    penetration_int += deltaZ * deltaZ;

    // Here we need to check if the contact point is in this SD.

    // Take spatial average of positions to get position of contact point
    // NOTE that we *do* want integer division since the SD-checking code uses ints anyways. Computing
    // this as an int is *much* faster than float, much less double, on Conlain's machine
    int3 contact_pos = (sphereA_pos + sphereB_pos) / 2;

    // NOTE this point is now local to the current SD

    bool contact_in_SD = checkLocalPointInSD(contact_pos, gran_params);

    const int64_t contact_threshold = (4l * gran_params->sphereRadius_SU) * gran_params->sphereRadius_SU;

    return contact_in_SD && penetration_int < contact_threshold;
}

// NOTE: expects force_accum to be normal force only
inline __device__ float3 computeRollingAngAcc(GranSphereDataPtr sphere_data,
                                              GranParamsPtr gran_params,
                                              float rolling_coeff,
                                              float spinning_coeff,
                                              const float3& normal_force,
                                              const float3& my_omega,
                                              const float3& their_omega,
                                              // TODO check to make sure r_contact is what is passed everywhere
                                              // vec from my center to center of contact
                                              const float3& r_contact) {
    float3 delta_Ang_Acc = {0., 0., 0.};

    if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS &&
        gran_params->rolling_mode != GRAN_ROLLING_MODE::NO_RESISTANCE) {
        float3 omega_rel = their_omega - my_omega;

        switch (gran_params->rolling_mode) {
            case GRAN_ROLLING_MODE::SCHWARTZ: {
                // As in chrono parallel
                // Rolling component
                // v_rot = l_p (w_p x n) - l_n (w_n x n)
                const float3 v_rot = Cross(omega_rel, r_contact);
                if (Length(v_rot) < 1e-7f) {  // TODO choose epsilon in SU L/T units
                    return make_float3(0.f, 0.f, 0.f);
                }

                const float dr = Length(r_contact);
                const float normal_force_mag = Length(normal_force);
                float3 torque = (rolling_coeff * normal_force_mag / Length(v_rot)) * Cross(r_contact, v_rot);

                // d_aa = torque / inertia
                delta_Ang_Acc = 1.f / (gran_params->sphereInertia_by_r * gran_params->sphereRadius_SU) * torque;
                break;
            }
            default: {
                ABORTABORTABORT("Rolling mode not implemented\n");
            }
        }
    }

    return delta_Ang_Acc;
}

/// Compute single-step friction displacement
/// set delta_t for the displacement
inline __device__ void computeSingleStepDisplacement(GranParamsPtr gran_params,
                                                     const float3& rel_vel,
                                                     float3& delta_t) {
    delta_t = rel_vel * gran_params->stepSize_SU;
    float ut = Length(delta_t);
}

/// Compute multi-step friction displacement
/// set delta_t for the displacement
inline __device__ void computeMultiStepDisplacement(GranParamsPtr gran_params,
                                                    GranSphereDataPtr sphere_data,
                                                    const size_t& contact_id,
                                                    const float3& vrel_t,
                                                    const float3& contact_normal,
                                                    float3& delta_t) {
    // get the tangential displacement so far
    delta_t = sphere_data->contact_history_map[contact_id];
    // add on what we have for this step
    delta_t = delta_t + vrel_t * gran_params->stepSize_SU;

    // project onto contact normal
    float disp_proj = Dot(delta_t, contact_normal);
    // remove normal projection
    delta_t = delta_t - disp_proj * contact_normal;

    // write back the updated displacement
    sphere_data->contact_history_map[contact_id] = delta_t;
}

inline __device__ void updateMultiStepDisplacement(GranSphereDataPtr sphere_data,
                                                   const size_t& contact_index,
                                                   const float3& vrel_t,
                                                   const float3& contact_normal,
                                                   const float k_t,
                                                   const float gamma_t,
                                                   const float m_eff,
                                                   const float force_model_multiplier,
                                                   const float3& tangent_force) {
    // Reverse engineer the delta_t from the clamped force and update the map
    sphere_data->contact_history_map[contact_index] =
        ((tangent_force / force_model_multiplier) + gamma_t * m_eff * vrel_t) / -k_t;
}

/// compute friction forces for a contact
/// returns tangent force including hertz factor, clamped and all
inline __device__ float3 computeFrictionForces(GranParamsPtr gran_params,
                                               GranSphereDataPtr sphere_data,
                                               size_t contact_index,
                                               float static_friction_coeff,
                                               float k_t,
                                               float gamma_t,
                                               float force_model_multiplier,
                                               float m_eff,
                                               const float3& normal_force,
                                               const float3& vrel_t,
                                               const float3& contact_normal) {
    float3 delta_t = {0.f, 0.f, 0.f};

    if (gran_params->friction_mode == GRAN_FRICTION_MODE::SINGLE_STEP) {
        computeSingleStepDisplacement(gran_params, vrel_t, delta_t);
    } else if (gran_params->friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
        // TODO optimize this if we already have the contact ID
        // Also updates tangential creep
        computeMultiStepDisplacement(gran_params, sphere_data, contact_index, vrel_t, contact_normal, delta_t);
    }

    float3 tangent_force = force_model_multiplier * (-k_t * delta_t - gamma_t * m_eff * vrel_t);
    const float ft = Length(tangent_force);  // could be small

    // TODO what value is (1) a negligible SU force (2) a safe number to divide by numerically?
    constexpr float GRAN_MACHINE_EPSILON = 1e-6f;
    if (ft < GRAN_MACHINE_EPSILON) {
        return make_float3(0.f, 0.f, 0.f);
    }

    // If the force is beyond the coulomb criterion, clamp it
    const float ft_max = Length(normal_force) * static_friction_coeff;
    if (ft > ft_max) {
        // Scale tangent_force to coulomb condition and use stiffness portion of that to clamp displacement
        tangent_force = tangent_force * ft_max / ft;  // TODO stability
        if (gran_params->friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
            updateMultiStepDisplacement(sphere_data, contact_index, vrel_t, contact_normal, k_t, gamma_t, m_eff,
                                        force_model_multiplier, tangent_force);
        }
    }

    return tangent_force;
}

// overload for if the body ids are given rather than contact id
inline __device__ float3 computeFrictionForces(GranParamsPtr gran_params,
                                               GranSphereDataPtr sphere_data,
                                               unsigned int body_A_index,
                                               unsigned int body_B_index,
                                               float static_friction_coeff,
                                               float k_t,
                                               float gamma_t,
                                               float force_model_multiplier,
                                               float m_eff,
                                               const float3& normal_force,
                                               const float3& rel_vel,
                                               const float3& contact_normal) {
    size_t contact_id = 0;

    // if multistep, compute contact id, otherwise we don't care anyways
    if (gran_params->friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
        contact_id = findContactPairInfo(sphere_data, gran_params, body_A_index, body_B_index);
    }
    return computeFrictionForces(gran_params, sphere_data, contact_id, static_friction_coeff, k_t, gamma_t,
                                 force_model_multiplier, m_eff, normal_force, rel_vel, contact_normal);
}
