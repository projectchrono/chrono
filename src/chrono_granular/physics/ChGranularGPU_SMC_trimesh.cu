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

#include "chrono_granular/physics/ChGranularGPU_SMC_trimesh.cuh"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_granular/physics/ChGranularGPU_SMC.cuh"

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

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    unsigned int numTriangles = meshSoup->nTrianglesInSoup;
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
                                               sphCntr, gran_params->sphereRadius_SU, normal, depth, pt1);

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
                force_accum = force_accum + gran_params->sphere_mass_SU * mesh_params->adhesionAcc_s2m * delta / depth;

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
                v_rel = v_rel - Cross(d_triangleSoup->omega[fam], r);

                // add tangential components if they exist
                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    // Vector from the center of sphere to center of contact volume
                    float3 r_A = -(gran_params->sphereRadius_SU + depth / 2.f) * normal;
                    v_rel = v_rel + Cross(omega[sphereIDLocal], r_A);
                }

                // Force accumulator on sphere for this sphere-triangle collision
                // Compute force updates for normal spring term

                // Compute force updates for damping term
                // NOTE assumes sphere mass of 1
                float fam_mass_SU = d_triangleSoup->familyMass_SU[fam];
                const float sphere_mass_SU = gran_params->sphere_mass_SU;
                float m_eff = sphere_mass_SU * fam_mass_SU / (sphere_mass_SU + fam_mass_SU);
                float3 vrel_n = Dot(v_rel, normal) * normal;
                v_rel = v_rel - vrel_n;  // v_rel is now tangential relative velocity
                
                // Add normal damping term
                force_accum = force_accum - hertz_force_factor * mesh_params->Gamma_n_s2m_SU * m_eff * vrel_n;
        
                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    // radius pointing from the contact point to the center of particle
                    float3 Rc = (gran_params->sphereRadius_SU + depth / 2.f) * normal;
                    float3 roll_ang_acc = computeRollingAngAcc(
                        sphere_data, gran_params, mesh_params->rolling_coeff_s2m_SU, mesh_params->spinning_coeff_s2m_SU,
                        force_accum, omega[sphereIDLocal], d_triangleSoup->omega[fam], Rc);

                    sphere_AngAcc = sphere_AngAcc + roll_ang_acc;

                    unsigned int BC_histmap_label = triangleFamilyHistmapOffset + fam;

                    // compute tangent force
                    float3 tangent_force = computeFrictionForces(
                        gran_params, sphere_data, sphereIDGlobal, BC_histmap_label,
                        mesh_params->static_friction_coeff_s2m, mesh_params->K_t_s2m_SU, mesh_params->Gamma_t_s2m_SU,
                        hertz_force_factor, m_eff, force_accum, v_rel, normal);

                    force_accum = force_accum + tangent_force;
                    sphere_AngAcc = sphere_AngAcc + Cross(-1.f * normal, tangent_force) / gran_params->sphereInertia_by_r;
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

__host__ double ChSystemGranularSMC_trimesh::advance_simulation(float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Settling simulation loop.
    float duration_SU = (float)(duration / TIME_SU2UU);
    unsigned int nsteps = (unsigned int)std::round(duration_SU / stepSize_SU);

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
                sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                (unsigned int)BC_params_list_SU.size());
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        } else if (gran_params->friction_mode == SINGLE_STEP || gran_params->friction_mode == MULTI_STEP) {
            // figure out who is contacting
            determineContactPairs<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(sphere_data, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());

            computeSphereContactForces<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                (unsigned int)BC_params_list_SU.size(), nSpheres);
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
            unsigned int triangleFamilyHistmapOffset =
                gran_params->nSpheres + 1 + (unsigned int)BC_params_list_SU.size() + 1;
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
        elapsedSimTime += (float)(stepSize_SU * TIME_SU2UU);  // Advance current time
    }

    return time_elapsed_SU * TIME_SU2UU;  // return elapsed UU time
}
}  // namespace granular
}  // namespace chrono
