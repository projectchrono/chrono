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

/// NOTE that this requires the normal force to be in hookean form (no hertz factor yet)
/// enforce the Coulomb condition that Ft <= mu Fn
/// by enforcing ut <= mu Fn / kt
inline __device__ bool clampTangentDisplacement(GranParamsPtr gran_params,
                                                float kt,
                                                float static_friction_coeff,
                                                const float3& normal_force,
                                                const float force_model_multiplier,
                                                float3& tangent_disp) {
    // divide out hertz force factor
    float ut_max = static_friction_coeff * Length(normal_force) / (kt * force_model_multiplier);
    // TODO also consider wall mu and kt clamping
    float ut = Length(tangent_disp);
    constexpr float GRAN_MACHINE_EPSILON = 1.e-8f;
    // If u_t is very small, force it to be exactly zero and return no clamping
    if (ut < GRAN_MACHINE_EPSILON) {
        tangent_disp = make_float3(0.f, 0.f, 0.f);
        return false;
    }
    if (ut > ut_max) {
        // TODO is this stable???:Q

        tangent_disp = tangent_disp * ut_max / ut;
        return true;
    }
    return false;
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
                                              const float3& normal_force,
                                              const float3& my_omega,
                                              const float3& their_omega,
                                              // points from their center to my center (in line with normal force)
                                              const float3& delta_r) {
    float3 delta_Ang_Acc = {0., 0., 0.};

    if (gran_params->friction_mode != GRAN_FRICTION_MODE::FRICTIONLESS &&
        gran_params->rolling_mode != GRAN_ROLLING_MODE::NO_RESISTANCE) {
        float3 omega_rel = my_omega - their_omega;
        float omega_rel_mag = Length(omega_rel);
        // normalize relative rotation
        if (omega_rel_mag != 0.f) {  // TODO some small bound?
            omega_rel = omega_rel / omega_rel_mag;
        }

        const float normal_force_mag = Length(normal_force);
        switch (gran_params->rolling_mode) {
            case GRAN_ROLLING_MODE::CONSTANT_TORQUE: {
                // M = -w / ||w|| * mu_r * r_eff * ||f_n||
                // Assumes r_eff = r/2
                // Reff / inerta = 1 / (2 * inertia_by_r)
                /// TODO ADD BC CHECK
                delta_Ang_Acc =
                    -1 * omega_rel * rolling_coeff * normal_force_mag / (2 * gran_params->sphereInertia_by_r);
                break;
            }
            case GRAN_ROLLING_MODE::VISCOUS: {
                // relative contact point velocity due to rotation ????
                const float v_mag = Length(Cross(-1 * delta_r, my_omega + their_omega));

                float3 torque = -rolling_coeff * v_mag * normal_force_mag * omega_rel;
                delta_Ang_Acc = 1.f / (gran_params->sphereInertia_by_r * gran_params->sphereRadius_SU) * torque;
                break;
            }
            default: { ABORTABORTABORT("Rolling mode not implemented\n"); }
        }
    }
    return delta_Ang_Acc;
}

/// Compute single-step friction displacement and clamp it
/// set delta_t for the displacement
inline __device__ bool computeSingleStepDisplacement(GranParamsPtr gran_params,
                                                     GranSphereDataPtr sphere_data,
                                                     float static_friction_coeff,
                                                     const float3& normal_force,
                                                     float force_model_multiplier,
                                                     const float3& rel_vel,
                                                     const float3& contact_normal,
                                                     const float k_t,
                                                     float3& delta_t) {
    delta_t = rel_vel * gran_params->stepSize_SU;
    bool clamped = clampTangentDisplacement(gran_params, k_t, static_friction_coeff, normal_force,
                                            force_model_multiplier, delta_t);
    return clamped;
}

/// Compute multi-step friction displacement and clamp it
/// set delta_t for the displacement
inline __device__ bool computeMultiStepDisplacement(GranParamsPtr gran_params,
                                                    GranSphereDataPtr sphere_data,
                                                    float static_friction_coeff,
                                                    const size_t& contact_id,
                                                    const float3& normal_force,
                                                    float force_model_multiplier,
                                                    const float3& rel_vel,
                                                    const float3& contact_normal,
                                                    const float k_t,
                                                    float3& delta_t) {
    // get the tangential displacement so far
    delta_t = sphere_data->contact_history_map[contact_id];
    // add on what we have for this step
    delta_t = delta_t + rel_vel * gran_params->stepSize_SU;

    // project onto contact normal
    float disp_proj = Dot(delta_t, contact_normal);
    // remove normal projection
    delta_t = delta_t - disp_proj * contact_normal;
    // clamp tangent displacement
    bool clamped = clampTangentDisplacement(gran_params, k_t, static_friction_coeff, normal_force,
                                            force_model_multiplier, delta_t);

    // write back the updated displacement
    sphere_data->contact_history_map[contact_id] = delta_t;
    return clamped;
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
                                               const float3& rel_vel,
                                               const float3& contact_normal) {
    float3 delta_t = {0., 0., 0.};
    bool clamped = true;

    if (gran_params->friction_mode == GRAN_FRICTION_MODE::SINGLE_STEP) {
        clamped = computeSingleStepDisplacement(gran_params, sphere_data, static_friction_coeff, normal_force,
                                                force_model_multiplier, rel_vel, contact_normal, k_t, delta_t);
    } else if (gran_params->friction_mode == GRAN_FRICTION_MODE::MULTI_STEP) {
        // TODO optimize this if we already have the contact ID
        clamped =
            computeMultiStepDisplacement(gran_params, sphere_data, static_friction_coeff, contact_index, normal_force,
                                         force_model_multiplier, rel_vel, contact_normal, k_t, delta_t);
    }

    float3 tangent_force = -k_t * delta_t * force_model_multiplier;
    // if no-slip, add the tangential damping
    if (!clamped) {
        tangent_force = tangent_force - gamma_t * m_eff * rel_vel * force_model_multiplier;
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
