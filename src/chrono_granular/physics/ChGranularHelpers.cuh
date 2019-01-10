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

// Decide which SD owns this point in space
// Pass it the Center of Mass location for a DE to get its owner, also used to get contact point
inline __device__ int3 pointSDTriplet(int sphCenter_X, int sphCenter_Y, int sphCenter_Z, GranParamsPtr gran_params) {
    // Note that this offset allows us to have moving walls and the like very easily

    int64_t sphCenter_X_modified = -gran_params->BD_frame_X + sphCenter_X;
    int64_t sphCenter_Y_modified = -gran_params->BD_frame_Y + sphCenter_Y;
    int64_t sphCenter_Z_modified = -gran_params->BD_frame_Z + sphCenter_Z;
    int3 n;
    // Get the SD of the sphere's center in the xdir
    n.x = (sphCenter_X_modified) / gran_params->SD_size_X_SU;
    // Same for D and H
    n.y = (sphCenter_Y_modified) / gran_params->SD_size_Y_SU;
    n.z = (sphCenter_Z_modified) / gran_params->SD_size_Z_SU;
    return n;
}

// inline __device__ pointSDID(int sphCenter_X, int sphCenter_Y, int sphCenter_Z, GranParamsPtr gran_params) {}

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

    // We need to make sure we don't count a collision twice -- if a contact pair is in multiple SDs,
    // count it only in the one that holds the contact point
    unsigned int contactSD =
        SDTripletID(pointSDTriplet(contact_pos.x, contact_pos.y, contact_pos.z, gran_params), gran_params);

    const int64_t contact_threshold = (4l * gran_params->sphereRadius_SU) * gran_params->sphereRadius_SU;

    return contactSD == thisSD && penetration_int < contact_threshold;
}