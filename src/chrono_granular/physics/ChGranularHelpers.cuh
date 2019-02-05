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
// Holds of helper functions for GPU granular code that need to be scoped higher
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly, Nic Olsen
// =============================================================================

#include "chrono_granular/physics/ChGranular.h"

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
inline __device__ unsigned int findContactPairInfo(contactDataStruct* sphere_contact_map,
                                                   GranParamsPtr gran_params,
                                                   unsigned int body_A,
                                                   unsigned int body_B) {
    // TODO this should be size_t everywhere
    unsigned int body_A_offset = MAX_SPHERES_TOUCHED_BY_SPHERE * body_A;
    // first skim through and see if this contact pair is in the map
    for (unsigned int contact_id = 0; contact_id < MAX_SPHERES_TOUCHED_BY_SPHERE; contact_id++) {
        unsigned int contact_index = body_A_offset + contact_id;
        if (sphere_contact_map[contact_index].body_B == body_B) {
            // make sure this contact is marked active
            sphere_contact_map[contact_index].active = true;
            return contact_index;
        }
    }

    // if we get this far, the contact pair isn't in the map now and we need to find an empty spot
    for (unsigned int contact_id = 0; contact_id < MAX_SPHERES_TOUCHED_BY_SPHERE; contact_id++) {
        unsigned int contact_index = body_A_offset + contact_id;
        // check whether the slot is free right now
        if (sphere_contact_map[contact_index].body_B == NULL_GRANULAR_ID) {
            // claim this slot for ourselves, atomically
            // if the CAS returns NULL_GRANULAR_ID, it means that the spot was free and we claimed it
            unsigned int body_B_returned =
                atomicCAS(&(sphere_contact_map[contact_index].body_B), NULL_GRANULAR_ID, body_B);
            // did we get the spot? if so, claim it
            if (NULL_GRANULAR_ID == body_B_returned) {
                // make sure this contact is marked active
                sphere_contact_map[contact_index].active = true;
                return contact_index;
            }
        }
    }

    // if we got this far, we couldn't find a free contact pair. That is a violation of the 12-contacts theorem, so
    // we should probably give up now
    ABORTABORTABORT("No available contact pair slots for body %u and body %u\n", body_A, body_B);
    return NULL_GRANULAR_ID;  // shouldn't get here anyways
}

// cleanup the contact data for a given body
inline __device__ void cleanupContactMap(GranSphereDataPtr sphere_data,
                                         unsigned int body_A,
                                         GranParamsPtr gran_params) {
    // printf("cleaning up body %u contacts\n", body_A);
    size_t body_A_offset = MAX_SPHERES_TOUCHED_BY_SPHERE * body_A;
    // first skim through and see if this contact pair is in the map
    for (unsigned int contact_id = 0; contact_id < MAX_SPHERES_TOUCHED_BY_SPHERE; contact_id++) {
        // if the contact is not active, reset it
        if (sphere_data->sphere_contact_map[body_A_offset + contact_id].active == false) {
            // printf("contact %u for body %u is inactive now, removing\n", );
            sphere_data->sphere_contact_map[body_A_offset + contact_id].body_B = NULL_GRANULAR_ID;
            if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP) {
                sphere_data->contact_history_map[body_A_offset + contact_id] = {0., 0., 0.};
            }
        } else {
            // otherwise reset the active bit for next time
            sphere_data->sphere_contact_map[body_A_offset + contact_id].active = false;
        }
    }
}

/// enforce the Coulomb condition that Ft <= mu Fn
/// by enforcing ut <= mu Fn / kt
inline __device__ bool clampTangentDisplacement(GranParamsPtr gran_params,
                                                const float3& normal_force,
                                                float3& tangent_disp) {
    float ut_max = gran_params->static_friction_coeff * Length(normal_force) / gran_params->K_t_s2s_SU;
    // TODO also consider wall mu and kt clamping
    float ut = Length(tangent_disp);
    if (ut > ut_max) {
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

/// Get the force multiplier for a contact given the penetration
/// delta_n is penetration normalized by diameter
// NOTE that this is here because the BC code also needs it
inline __device__ float get_force_multiplier(float delta_n, GranParamsPtr gran_params) {
    switch (gran_params->force_model) {
        case chrono::granular::GRAN_FORCE_MODEL::HOOKE: {
            return 1.f;
        }
        case chrono::granular::GRAN_FORCE_MODEL::HERTZ: {
            return sqrt(delta_n);
        }
    }
    // if we get here, something is wrong
    ABORTABORTABORT("Invalid contact model\n");
    return 0;  // this should never happen
}