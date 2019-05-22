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

#include "chrono_granular/physics/ChGranularGPU_SMC.cuh"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_granular/ChGranularDefines.h"

// these define things that mess with cub
#include "chrono_granular/physics/ChGranularCollision.cuh"
#include "chrono_granular/physics/ChGranularBoxTriangle.cuh"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
// get nicer handles to pointer names, enforce const-ness on the mesh params
typedef const chrono::granular::ChGranParams_trimesh* MeshParamsPtr;
typedef chrono::granular::ChTriangleSoup<float3>* TriangleSoupPtr;

/// Point is in the LRF, rot_mat rotates LRF to GRF, pos translates LRF to GRF
/// LRF: local reference frame
/// GRF: global reference frame
template <class IN_T, class IN_T3, class OUT_T3 = IN_T3>
__device__ OUT_T3 apply_frame_transform(const IN_T3& point, const IN_T* pos, const IN_T* rot_mat) {
    OUT_T3 result;

    // Apply rotation matrix to point
    result.x = rot_mat[0] * point.x + rot_mat[1] * point.y + rot_mat[2] * point.z;
    result.y = rot_mat[3] * point.x + rot_mat[4] * point.y + rot_mat[5] * point.z;
    result.z = rot_mat[6] * point.x + rot_mat[7] * point.y + rot_mat[8] * point.z;

    // Apply translation
    result.x += pos[0];
    result.y += pos[1];
    result.z += pos[2];

    return result;
}

template <class T3>
__device__ void convert_pos_UU2SU(T3& pos, GranParamsPtr gran_params) {
    pos.x /= gran_params->LENGTH_UNIT;
    pos.y /= gran_params->LENGTH_UNIT;
    pos.z /= gran_params->LENGTH_UNIT;
}

/// Takes in a triangle ID and figures out an SD AABB for broadphase use
__inline__ __device__ void triangle_figureOutSDBox(const float3& vA,
                                                   const float3& vB,
                                                   const float3& vC,
                                                   const bool inflated,
                                                   int* L,
                                                   int* U,
                                                   GranParamsPtr gran_params) {
    int3 min_pt;
    min_pt.x = MIN(vA.x, MIN(vB.x, vC.x));
    min_pt.y = MIN(vA.y, MIN(vB.y, vC.y));
    min_pt.z = MIN(vA.z, MIN(vB.z, vC.z));

    int3 max_pt;
    max_pt.x = MAX(vA.x, MAX(vB.x, vC.x));
    max_pt.y = MAX(vA.y, MAX(vB.y, vC.y));
    max_pt.z = MAX(vA.z, MAX(vB.z, vC.z));

    if (inflated) {
        int3 offset =
            make_int3(gran_params->sphereRadius_SU, gran_params->sphereRadius_SU, gran_params->sphereRadius_SU);
        min_pt = min_pt - offset;
        max_pt = max_pt + offset;
    }

    int3 tmp = pointSDTriplet(min_pt.x, min_pt.y, min_pt.z, gran_params);
    L[0] = tmp.x;
    L[1] = tmp.y;
    L[2] = tmp.z;

    tmp = pointSDTriplet(max_pt.x, max_pt.y, max_pt.z, gran_params);
    U[0] = tmp.x;
    U[1] = tmp.y;
    U[2] = tmp.z;
}

/// Takes in a triangle's position in UU and finds out how many SDs it touches
/// Triangle broadphase is done in float by applying the frame transform
/// and then converting the GRF position to SU
inline __device__ unsigned int triangle_countTouchedSDs(unsigned int triangleID,
                                                        const TriangleSoupPtr triangleSoup,
                                                        GranParamsPtr gran_params,
                                                        MeshParamsPtr tri_params) {
    float3 vA, vB, vC;

    // Transform LRF to GRF
    unsigned int fam = triangleSoup->triangleFamily_ID[triangleID];
    vA = apply_frame_transform<float, float3>(triangleSoup->node1[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vB = apply_frame_transform<float, float3>(triangleSoup->node2[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vC = apply_frame_transform<float, float3>(triangleSoup->node3[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);

    // Convert UU to SU
    convert_pos_UU2SU<float3>(vA, gran_params);
    convert_pos_UU2SU<float3>(vB, gran_params);
    convert_pos_UU2SU<float3>(vC, gran_params);

    // bottom-left and top-right corners
    int L[3];
    int U[3];
    const bool inflated = triangleSoup->inflated[fam];
    triangle_figureOutSDBox(vA, vB, vC, inflated, L, U, gran_params);
    // Case 1: All vetices are in the same SD
    if (L[0] == U[0] && L[1] == U[1] && L[2] == U[2]) {
        unsigned int currSD = SDTripletID(L, gran_params);
        if (currSD != NULL_GRANULAR_ID) {
            return 1;
        } else {
            // TODO optimize me?
            return 0;
        }
    }

    unsigned int n_axes_diff = 0;  // Count axes that have different SD bounds
    unsigned int axes_diff;        // axis of variation (if only one)

    for (int i = 0; i < 3; i++) {
        if (L[i] != U[i]) {
            axes_diff = i;  // If there are more than one, this won't be used anyway
            n_axes_diff++;
        }
    }
    unsigned int numSDsTouched = 0;

    // Case 2: Triangle lies in a Nx1x1, 1xNx1, or 1x1xN block of SDs
    if (n_axes_diff == 1) {
        // add one since it's in each of these SDs
        int SD_i[3] = {L[0], L[1], L[2]};  // start at 'bottom' and move up
        for (int i = L[axes_diff]; i <= U[axes_diff]; i++) {
            SD_i[axes_diff] = i;  // current SD index along this direction
            unsigned int currSD = SDTripletID(SD_i, gran_params);
            if (currSD != NULL_GRANULAR_ID) {
                numSDsTouched++;
            }
        }
        return numSDsTouched;
    }

    // Case 3: Triangle spans more than one dimension of spheresTouchingThisSD
    // TODO check that this is safe to do
    float SDcenter[3];
    float SDhalfSizes[3];
    for (int i = L[0]; i <= U[0]; i++) {
        for (int j = L[1]; j <= U[1]; j++) {
            for (int k = L[2]; k <= U[2]; k++) {
                SDhalfSizes[0] = gran_params->SD_size_X_SU / 2;
                SDhalfSizes[1] = gran_params->SD_size_Y_SU / 2;
                SDhalfSizes[2] = gran_params->SD_size_Z_SU / 2;

                SDcenter[0] = gran_params->BD_frame_X + (i * 2 + 1) * SDhalfSizes[0];
                SDcenter[1] = gran_params->BD_frame_Y + (j * 2 + 1) * SDhalfSizes[1];
                SDcenter[2] = gran_params->BD_frame_Z + (k * 2 + 1) * SDhalfSizes[2];

                if (inflated || check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    unsigned int currSD = SDTripletID(i, j, k, gran_params);
                    if (currSD != NULL_GRANULAR_ID) {
                        numSDsTouched++;
                    }
                }
            }
        }
    }
    return numSDsTouched;
}

/// Takes in a triangle's position in UU and finds out what SDs it touches
/// Triangle broadphase is done in float by applying the frame transform
/// and then converting the GRF position to SU
inline __device__ void triangle_figureOutTouchedSDs(unsigned int triangleID,
                                                    const TriangleSoupPtr triangleSoup,
                                                    unsigned int* touchedSDs,
                                                    GranParamsPtr gran_params,
                                                    MeshParamsPtr tri_params) {
    float3 vA, vB, vC;

    // Transform LRF to GRF
    unsigned int fam = triangleSoup->triangleFamily_ID[triangleID];
    vA = apply_frame_transform<float, float3>(triangleSoup->node1[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vB = apply_frame_transform<float, float3>(triangleSoup->node2[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vC = apply_frame_transform<float, float3>(triangleSoup->node3[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);

    // Convert UU to SU
    convert_pos_UU2SU<float3>(vA, gran_params);
    convert_pos_UU2SU<float3>(vB, gran_params);
    convert_pos_UU2SU<float3>(vC, gran_params);

    // bottom-left and top-right corners
    int L[3];
    int U[3];
    const bool inflated = triangleSoup->inflated[fam];
    triangle_figureOutSDBox(vA, vB, vC, inflated, L, U, gran_params);

    // TODO modularize more code
    // Case 1: All vetices are in the same SD
    if (L[0] == U[0] && L[1] == U[1] && L[2] == U[2]) {
        // if we get here and don't have a valid SD, this should be an error
        unsigned int currSD = SDTripletID(L, gran_params);
        if (currSD != NULL_GRANULAR_ID) {
            touchedSDs[0] = currSD;
        }
        return;
    }

    unsigned int SD_count = 0;
    unsigned int n_axes_diff = 0;  // Count axes that have different SD bounds
    unsigned int axes_diff;        // axis of variation (if only one)

    for (int i = 0; i < 3; i++) {
        if (L[i] != U[i]) {
            axes_diff = i;  // If there are more than one, this won't be used anyway
            n_axes_diff++;
        }
    }

    // Case 2: Triangle lies in a Nx1x1, 1xNx1, or 1x1xN block of SDs
    if (n_axes_diff == 1) {
        int SD_i[3] = {L[0], L[1], L[2]};  // start at 'bottom' and move up
        for (int i = L[axes_diff]; i <= U[axes_diff]; i++) {
            SD_i[axes_diff] = i;  // current SD index along this direction

            unsigned int currSD = SDTripletID(SD_i, gran_params);
            if (currSD != NULL_GRANULAR_ID) {
                touchedSDs[SD_count++] = currSD;
            }
        }

        return;
    }

    // Case 3: Triangle spans more than one dimension of spheresTouchingThisSD
    float SDcenter[3];
    float SDhalfSizes[3];
    for (int i = L[0]; i <= U[0]; i++) {
        for (int j = L[1]; j <= U[1]; j++) {
            for (int k = L[2]; k <= U[2]; k++) {
                SDhalfSizes[0] = gran_params->SD_size_X_SU / 2;
                SDhalfSizes[1] = gran_params->SD_size_Y_SU / 2;
                SDhalfSizes[2] = gran_params->SD_size_Z_SU / 2;

                SDcenter[0] = gran_params->BD_frame_X + (i * 2 + 1) * SDhalfSizes[0];
                SDcenter[1] = gran_params->BD_frame_Y + (j * 2 + 1) * SDhalfSizes[1];
                SDcenter[2] = gran_params->BD_frame_Z + (k * 2 + 1) * SDhalfSizes[2];

                // If mesh is inflated, we don't have a higher-resultion check yet
                if (inflated || check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    unsigned int currSD = SDTripletID(i, j, k, gran_params);
                    if (currSD != NULL_GRANULAR_ID) {
                        touchedSDs[SD_count++] = currSD;
                    }
                }
            }
        }
    }
}

__global__ void triangleSoup_CountSDsTouched(
    const TriangleSoupPtr d_triangleSoup,
    unsigned int* Triangle_NumSDsTouching,  //!< number of SDs touching this Triangle
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        Triangle_NumSDsTouching[myTriangleID] =
            triangle_countTouchedSDs(myTriangleID, d_triangleSoup, gran_params, mesh_params);
    }
}

__global__ void triangleSoup_StoreSDsTouched(
    const TriangleSoupPtr d_triangleSoup,
    unsigned int* Triangle_NumSDsTouching,     //!< number of SDs touching this Triangle
    unsigned int* TriangleSDCompositeOffsets,  //!< number of SDs touching this Triangle
    unsigned int* Triangle_SDsComposite,       //!< number of SDs touching this Triangle
    unsigned int* Triangle_TriIDsComposite,    //!< number of SDs touching this Triangle
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup,
                                     Triangle_SDsComposite + TriangleSDCompositeOffsets[myTriangleID], gran_params,
                                     mesh_params);
        // TODO could be faster
        for (unsigned int i = 0; i < Triangle_NumSDsTouching[myTriangleID]; i++) {
            // write back this triangle ID to be sorted
            Triangle_TriIDsComposite[TriangleSDCompositeOffsets[myTriangleID] + i] = myTriangleID;
        }
    }
}

template <unsigned int N_CUDATHREADS>
__global__ void interactionTerrain_TriangleSoup(
    TriangleSoupPtr d_triangleSoup,  //!< Contains information pertaining to triangle soup (in device mem.)
    GranSphereDataPtr sphere_data,
    unsigned int* triangles_in_SD_composite,    //!< Big array that works in conjunction with SD_numTrianglesTouching.
    unsigned int* SD_numTrianglesTouching,      //!< number of triangles touching this SD
    unsigned int* SD_TriangleCompositeOffsets,  //!< offset of triangles in the composite array for each SD
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params,
    unsigned int triangleFamilyHistmapOffset) {
    __shared__ unsigned int triangleIDs[MAX_TRIANGLE_COUNT_PER_SD];  //!< global ID of the triangles touching this SD

    __shared__ int3 sphere_pos_local[MAX_COUNT_OF_SPHERES_PER_SD];  //!< local coordinate of the sphere

    __shared__ float3 sphere_vel[MAX_COUNT_OF_SPHERES_PER_SD];

    // TODO figure out how we can do this better with no friction
    __shared__ float3 omega[MAX_COUNT_OF_SPHERES_PER_SD];

    __shared__ double3 node1[MAX_TRIANGLE_COUNT_PER_SD];  //!< Coordinates of the 1st node of the triangle
    __shared__ double3 node2[MAX_TRIANGLE_COUNT_PER_SD];  //!< Coordinates of the 2nd node of the triangle
    __shared__ double3 node3[MAX_TRIANGLE_COUNT_PER_SD];  //!< Coordinates of the 3rd node of the triangle

    // define an alias first
    unsigned int thisSD = blockIdx.x;

    if (SD_numTrianglesTouching[thisSD] == 0) {
        return;  // no triangle touches this block's SD
    }
    unsigned int spheresTouchingThisSD = sphere_data->SD_NumSpheresTouching[thisSD];
    if (spheresTouchingThisSD == 0) {
        return;  // no sphere touches this block's SD
    }

    // Getting here means that there are both triangles and DEs in this SD.
    unsigned int numSDTriangles = SD_numTrianglesTouching[thisSD];
    unsigned int sphereIDLocal = threadIdx.x;
    unsigned int sphereIDGlobal = NULL_GRANULAR_ID;
    // Bring in data from global into shmem. Only a subset of threads get to do this.
    // Note that we're not using shared memory very heavily, so our bandwidth is pretty low
    if (sphereIDLocal < spheresTouchingThisSD) {
        size_t SD_composite_offset = sphere_data->SD_SphereCompositeOffsets[thisSD];

        // TODO standardize this
        size_t offset_in_composite_Array = SD_composite_offset + sphereIDLocal;
        sphereIDGlobal = sphere_data->spheres_in_SD_composite[offset_in_composite_Array];

        sphere_pos_local[sphereIDLocal] =
            make_int3(sphere_data->sphere_local_pos_X[sphereIDGlobal], sphere_data->sphere_local_pos_Y[sphereIDGlobal],
                      sphere_data->sphere_local_pos_Z[sphereIDGlobal]);

        unsigned int sphere_owner_SD = sphere_data->sphere_owner_SDs[sphereIDGlobal];
        // if this SD doesn't own that sphere, add an offset to account
        if (sphere_owner_SD != thisSD) {
            sphere_pos_local[sphereIDLocal] =
                sphere_pos_local[sphereIDLocal] + getOffsetFromSDs(thisSD, sphere_owner_SD, gran_params);
        }

        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            omega[sphereIDLocal] =
                make_float3(sphere_data->sphere_Omega_X[sphereIDGlobal], sphere_data->sphere_Omega_Y[sphereIDGlobal],
                            sphere_data->sphere_Omega_Z[sphereIDGlobal]);
        }
        sphere_vel[sphereIDLocal] =
            make_float3(sphere_data->pos_X_dt[sphereIDGlobal], sphere_data->pos_Y_dt[sphereIDGlobal],
                        sphere_data->pos_Z_dt[sphereIDGlobal]);
    }
    // Populate the shared memory with mesh triangle data
    unsigned int tripsToCoverTriangles = (numSDTriangles + blockDim.x - 1) / blockDim.x;
    unsigned int local_ID = threadIdx.x;
    for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
        if (local_ID < numSDTriangles) {
            size_t SD_composite_offset = SD_TriangleCompositeOffsets[thisSD];
            if (SD_composite_offset == NULL_GRANULAR_ID) {
                ABORTABORTABORT("Invalid composite offset %llu for SD %u, touching %u triangles\n", NULL_GRANULAR_ID,
                                thisSD, numSDTriangles);
            }
            size_t offset_in_composite_Array = SD_composite_offset + local_ID;

            unsigned int globalID = triangles_in_SD_composite[offset_in_composite_Array];
            triangleIDs[local_ID] = globalID;

            // Read node positions from global memory into shared memory
            // NOTE implicit cast from float to double here
            unsigned int fam = d_triangleSoup->triangleFamily_ID[globalID];
            node1[local_ID] = apply_frame_transform<double, float3, double3>(
                d_triangleSoup->node1[globalID], mesh_params->fam_frame_narrow[fam].pos,
                mesh_params->fam_frame_narrow[fam].rot_mat);

            node2[local_ID] = apply_frame_transform<double, float3, double3>(
                d_triangleSoup->node2[globalID], mesh_params->fam_frame_narrow[fam].pos,
                mesh_params->fam_frame_narrow[fam].rot_mat);

            node3[local_ID] = apply_frame_transform<double, float3, double3>(
                d_triangleSoup->node3[globalID], mesh_params->fam_frame_narrow[fam].pos,
                mesh_params->fam_frame_narrow[fam].rot_mat);

            convert_pos_UU2SU<double3>(node1[local_ID], gran_params);
            convert_pos_UU2SU<double3>(node2[local_ID], gran_params);
            convert_pos_UU2SU<double3>(node3[local_ID], gran_params);
        }
        local_ID += blockDim.x;
    }

    __syncthreads();  // this call ensures data is in its place in shared memory

    float3 sphere_force = {0.f, 0.f, 0.f};
    float3 sphere_AngAcc = {0.f, 0.f, 0.f};
    if (sphereIDLocal < spheresTouchingThisSD) {
        // loop over each triangle in the SD and compute the force this sphere (thread) exerts on it
        for (unsigned int triangleLocalID = 0; triangleLocalID < numSDTriangles; triangleLocalID++) {
            /// we have a valid sphere and a valid triganle; check if in contact
            float3 normal;  // Unit normal from pt2 to pt1 (triangle contact point to sphere contact point)
            float depth;    // Negative in overlap
            float3 pt1_float;

            // Transform LRF to GRF
            const unsigned int fam = d_triangleSoup->triangleFamily_ID[triangleIDs[triangleLocalID]];
            bool valid_contact = false;

            // vector from center of mesh body to contact point, assume this can be held in a float
            float3 fromCenter;

            {
                double3 pt1;  // Contact point on triangle
                // NOTE sphere_pos_local is relative to THIS SD, not its owner SD
                double3 sphCntr =
                    int64_t3_to_double3(convertPosLocalToGlobal(thisSD, sphere_pos_local[sphereIDLocal], gran_params));
                valid_contact = face_sphere_cd(node1[triangleLocalID], node2[triangleLocalID], node3[triangleLocalID],
                                               sphCntr, gran_params->sphereRadius_SU, d_triangleSoup->inflated[fam],
                                               d_triangleSoup->inflation_radii[fam], normal, depth, pt1);

                valid_contact = valid_contact &&
                                SDTripletID(pointSDTriplet(pt1.x, pt1.y, pt1.z, gran_params), gran_params) == thisSD;
                pt1_float = make_float3(pt1.x, pt1.y, pt1.z);

                double3 meshCenter_double =
                    make_double3(mesh_params->fam_frame_narrow[fam].pos[0], mesh_params->fam_frame_narrow[fam].pos[1],
                                 mesh_params->fam_frame_narrow[fam].pos[2]);
                convert_pos_UU2SU<double3>(meshCenter_double, gran_params);

                double3 fromCenter_double = pt1 - meshCenter_double;
                fromCenter = make_float3(fromCenter_double.x, fromCenter_double.y, fromCenter_double.z);
            }

            // If there is a collision, add an impulse to the sphere
            if (valid_contact) {
                // TODO contact models
                // Use the CD information to compute the force on the grElement
                float3 delta = -depth * normal;

                // effective radius is just sphere radius -- assume meshes are locally flat (a safe assumption?)
                float hertz_force_factor = sqrt(abs(depth) / gran_params->sphereRadius_SU);

                float3 force_accum = hertz_force_factor * mesh_params->K_n_s2m_SU * delta;

                // Compute force updates for adhesion term, opposite the spring term
                // NOTE ratio is wrt the weight of a sphere of mass 1
                // NOTE the cancelation of two negatives
                force_accum = force_accum + gran_params->sphere_mass_SU * gran_params->adhesionAcc_s2w * delta / depth;

                // Velocity difference, it's better to do a coalesced access here than a fragmented access
                // inside
                float3 v_rel = sphere_vel[sphereIDLocal] - d_triangleSoup->vel[fam];

                // TODO assumes pos is the center of mass of the mesh
                // TODO can this be float?
                float3 meshCenter =
                    make_float3(mesh_params->fam_frame_broad[fam].pos[0], mesh_params->fam_frame_broad[fam].pos[1],
                                mesh_params->fam_frame_broad[fam].pos[2]);
                convert_pos_UU2SU<float3>(meshCenter, gran_params);

                // NOTE depth is negative and normal points from triangle to sphere center
                float3 r = pt1_float + normal * (depth / 2) - meshCenter;

                // Add angular velocity contribution from mesh
                v_rel = v_rel + Cross(d_triangleSoup->omega[fam], r);

                // add tangential components if they exist
                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    // Vector from the center of sphere to center of contact volume
                    float3 r_A = -(gran_params->sphereRadius_SU + depth / 2.f) * normal;
                    v_rel = v_rel + Cross(omega[sphereIDLocal], r_A);
                }

                // Forace accumulator on sphere for this sphere-triangle collision
                // Compute force updates for normal spring term

                // Compute force updates for damping term
                // NOTE assumes sphere mass of 1
                float fam_mass_SU = d_triangleSoup->familyMass_SU[fam];
                constexpr float sphere_mass_SU = gran_params->sphere_mass_SU;
                float m_eff = sphere_mass_SU * fam_mass_SU / (sphere_mass_SU + fam_mass_SU);
                float3 vrel_n = Dot(v_rel, normal) * normal;
                v_rel = v_rel - vrel_n;  // v_rel is now tangential relative velocity

                // Add normal damping term
                force_accum = force_accum - hertz_force_factor * mesh_params->Gamma_n_s2m_SU * m_eff * vrel_n;

                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    float3 roll_ang_acc =
                        computeRollingAngAcc(sphere_data, gran_params, mesh_params->rolling_coeff_s2m_SU, force_accum,
                                             omega[sphereIDLocal], d_triangleSoup->omega[fam], delta);

                    // sphere_AngAcc = sphere_AngAcc + roll_ang_acc;

                    unsigned int BC_histmap_label = triangleFamilyHistmapOffset + fam;

                    // compute tangent force
                    float3 tangent_force = computeFrictionForces(
                        gran_params, sphere_data, sphereIDGlobal, BC_histmap_label,
                        mesh_params->static_friction_coeff_s2m, mesh_params->K_t_s2m_SU, mesh_params->Gamma_t_s2m_SU,
                        hertz_force_factor, m_eff, force_accum, v_rel, normal);

                    force_accum = force_accum + tangent_force;
                    sphere_AngAcc = sphere_AngAcc + Cross(-1 * delta, tangent_force) / gran_params->sphereInertia_by_r;
                }

                // Use the CD information to compute the force and torque on the family of this triangle
                sphere_force = sphere_force + force_accum;

                // Force on the mesh is opposite the force on the sphere
                float3 force_total = -1.f * force_accum;

                float3 torque = Cross(fromCenter, force_total);
                // TODO we could be much smarter about reducing this atomic write
                unsigned int fam = d_triangleSoup->triangleFamily_ID[triangleIDs[triangleLocalID]];
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 0, force_total.x);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 1, force_total.y);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 2, force_total.z);

                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 3, torque.x);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 4, torque.y);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 5, torque.z);
            }
        }  // end of per-triangle loop
        // write back sphere forces
        atomicAdd(sphere_data->sphere_acc_X + sphereIDGlobal, sphere_force.x / gran_params->sphere_mass_SU);
        atomicAdd(sphere_data->sphere_acc_Y + sphereIDGlobal, sphere_force.y / gran_params->sphere_mass_SU);
        atomicAdd(sphere_data->sphere_acc_Z + sphereIDGlobal, sphere_force.z / gran_params->sphere_mass_SU);

        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            // write back torques for later
            atomicAdd(sphere_data->sphere_ang_acc_X + sphereIDGlobal, sphere_AngAcc.x);
            atomicAdd(sphere_data->sphere_ang_acc_Y + sphereIDGlobal, sphere_AngAcc.y);
            atomicAdd(sphere_data->sphere_ang_acc_Z + sphereIDGlobal, sphere_AngAcc.z);
        }
    }  // end sphere id check
}  // end kernel