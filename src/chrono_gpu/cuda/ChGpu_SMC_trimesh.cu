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
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut, Ruochun Zhang
// =============================================================================

#include "chrono_gpu/cuda/ChGpu_SMC_trimesh.cuh"
#include "chrono_gpu/cuda/ChGpu_SMC.cuh"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"
#include "chrono_gpu/utils/ChGpuUtilities.h"
#include <math_constants.h>

namespace chrono {
namespace gpu {

__host__ void ChSystemGpuMesh_impl::runTriangleBroadphase() {
    METRICS_PRINTF("Resetting broadphase info!\n");

    unsigned int numTriangles = meshSoup->nTrianglesInSoup;
    unsigned int nblocks = (numTriangles + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    determineCountOfSDsTouchedByEachTriangle<<<nblocks, CUDA_THREADS_PER_BLOCK>>>(
        meshSoup, Triangle_NumSDsTouching.data(), gran_params, tri_params);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    // do prefix scan
    size_t temp_storage_bytes = 0;
    unsigned int* out_ptr = Triangle_SDsCompositeOffsets.data();
    unsigned int* in_ptr = Triangle_NumSDsTouching.data();

    // copy data into the tmp array
    gpuErrchk(cudaMemcpy(out_ptr, in_ptr, numTriangles * sizeof(unsigned int), cudaMemcpyDeviceToDevice));
    cub::DeviceScan::ExclusiveSum(NULL, temp_storage_bytes, in_ptr, out_ptr, numTriangles);
    gpuErrchk(cudaDeviceSynchronize());

    // get pointer to device memory; this memory block will be used internally by CUB, for scratch area
    void* d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    // Run exclusive prefix sum
    cub::DeviceScan::ExclusiveSum(d_scratch_space, temp_storage_bytes, in_ptr, out_ptr, numTriangles);
    gpuErrchk(cudaDeviceSynchronize());
    unsigned int numOfTriangleTouchingSD_instances;  // total number of instances in which a triangle touches an SD
    numOfTriangleTouchingSD_instances = out_ptr[numTriangles - 1] + in_ptr[numTriangles - 1];

    // resize, if need be, several dummy vectors that handle in managed memory
    SDsTouchedByEachTriangle_composite_out.resize(numOfTriangleTouchingSD_instances, NULL_CHGPU_ID);
    SDsTouchedByEachTriangle_composite.resize(numOfTriangleTouchingSD_instances, NULL_CHGPU_ID);
    TriangleIDS_ByMultiplicity_out.resize(numOfTriangleTouchingSD_instances, NULL_CHGPU_ID);
    TriangleIDS_ByMultiplicity.resize(numOfTriangleTouchingSD_instances, NULL_CHGPU_ID);

    // sort key-value where the key is SD id, value is triangle ID in composite array
    storeSDsTouchedByEachTriangle<<<nblocks, CUDA_THREADS_PER_BLOCK>>>(
        meshSoup, Triangle_NumSDsTouching.data(), Triangle_SDsCompositeOffsets.data(),
        SDsTouchedByEachTriangle_composite.data(), TriangleIDS_ByMultiplicity.data(), gran_params, tri_params);
    gpuErrchk(cudaDeviceSynchronize());

    unsigned int* d_keys_in = SDsTouchedByEachTriangle_composite.data();
    unsigned int* d_keys_out = SDsTouchedByEachTriangle_composite_out.data();
    unsigned int* d_values_in = TriangleIDS_ByMultiplicity.data();
    unsigned int* d_values_out = TriangleIDS_ByMultiplicity_out.data();

    // Run CUB sorting operation, key-value type.
    // Key: the ID of the SD.
    // Value: the ID of the triangle that touches the "Key" SD.
    // The outcome of the sort operation will look like this:
    // SDs:       23 23 23 89 89  89  89  107 107 107 etc.
    // Triangle:   5  9 17 43 67 108 221    6  12 298 etc.
    // First, determine temporary device storage requirements; pass null, CUB tells us what it needs
    cub::DeviceRadixSort::SortPairs(NULL, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in, d_values_out,
                                    numOfTriangleTouchingSD_instances);
    gpuErrchk(cudaDeviceSynchronize());

    // get pointer to device memory; this memory block will be used internally by CUB
    d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    cub::DeviceRadixSort::SortPairs(d_scratch_space, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in,
                                    d_values_out, numOfTriangleTouchingSD_instances);
    gpuErrchk(cudaDeviceSynchronize());

    // We started with SDs touching a triangle; we just flipped this through the key-value sort. That is, we now
    // know the collection of triangles that touch each SD; SD by SD.
    SD_trianglesInEachSD_composite.resize(TriangleIDS_ByMultiplicity_out.size());
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaMemcpy(SD_trianglesInEachSD_composite.data(), TriangleIDS_ByMultiplicity_out.data(),
                         numOfTriangleTouchingSD_instances * sizeof(unsigned int), cudaMemcpyDeviceToDevice));

    // The CUB encode operation below will tell us what SDs are actually touched by triangles, and how many triangles
    // touch each SD.
    //
    // "d_in" is SDsTouchedByEachTriangle_composite_out; contains the IDs of the SDs that have triangles in them; if an
    // SD is touched by "t" triangles, it'll show up "t" times in this array
    unsigned int* d_in = d_keys_out;
    // d_unique_out stores a list of *unique* SDs with the following property: each SD in this list has at least one
    // triangle touching it. In terms of memory, this is pretty wasteful since it's unilkely that all SDs are touched by
    // at least one triangle; perhaps revisit later.
    unsigned int* d_unique_out =
        (unsigned int*)stateOfSolver_resources.pDeviceMemoryScratchSpace(nSDs * sizeof(unsigned int));
    // squatting on SD_TrianglesCompositeOffsets device vector; its size is nSDs. Works in tandem with d_unique_out.
    // If d_unique_out[4]=72, d_counts_out[4] says how many triangles touch SD 72.
    unsigned int* d_counts_out = SD_TrianglesCompositeOffsets.data();
    // squatting on TriangleIDS_ByMultiplicity, which is not needed anymore. We're using only *one* entry in this array.
    // Output value represents the number of SDs that have at last one triangle touching the SD
    unsigned int* d_num_runs_out = Triangle_SDsCompositeOffsets.data();
    // dry run, figure out the number of bytes that will be used in the actual run
    cub::DeviceRunLengthEncode::Encode(NULL, temp_storage_bytes, d_in, d_unique_out, d_counts_out, d_num_runs_out,
                                       numOfTriangleTouchingSD_instances);
    gpuErrchk(cudaDeviceSynchronize());

    d_scratch_space = TriangleIDS_ByMultiplicity.data();
    // Run the actual encoding operation
    cub::DeviceRunLengthEncode::Encode(d_scratch_space, temp_storage_bytes, d_in, d_unique_out, d_counts_out,
                                       d_num_runs_out, numOfTriangleTouchingSD_instances);
    gpuErrchk(cudaDeviceSynchronize());

    // SD_numTrianglesTouching contains only zeros
    // compute offsets in SD_trianglesInEachSD_composite and also counts for how many triangles touch each SD.
    // Start by zeroing out, it's important since not all entries will be touched in
    gpuErrchk(cudaMemset(SD_numTrianglesTouching.data(), 0, nSDs * sizeof(unsigned int)));
    nblocks = ((*d_num_runs_out) + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    if (nblocks > 0) {
        finalizeSD_numTrianglesTouching<<<nblocks, CUDA_THREADS_PER_BLOCK>>>(d_unique_out, d_counts_out, d_num_runs_out,
                                                                             SD_numTrianglesTouching.data());
        gpuErrchk(cudaDeviceSynchronize());
    }

    // Now assert that no SD has over max amount of triangles
    // If there is one, exit graciously
    in_ptr = SD_numTrianglesTouching.data();
    // Just borrow the first element of SD_TrianglesCompositeOffsets to store the max value
    unsigned int* maxTriCount = SD_TrianglesCompositeOffsets.data();
    cub::DeviceReduce::Max(NULL, temp_storage_bytes, in_ptr, maxTriCount, nSDs);
    gpuErrchk(cudaDeviceSynchronize());
    d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    cub::DeviceReduce::Max(d_scratch_space, temp_storage_bytes, in_ptr, maxTriCount, nSDs);
    gpuErrchk(cudaDeviceSynchronize());
    if (*maxTriCount > MAX_TRIANGLE_COUNT_PER_SD)
        CHGPU_ERROR("ERROR! %u triangles are found in one of the SDs! The max allowance is %u.\n", *maxTriCount,
                    MAX_TRIANGLE_COUNT_PER_SD);

    // Lastly, we need to do a CUB prefix scan to get the offsets in the big composite array
    in_ptr = SD_numTrianglesTouching.data();
    out_ptr = SD_TrianglesCompositeOffsets.data();
    cub::DeviceScan::ExclusiveSum(NULL, temp_storage_bytes, in_ptr, out_ptr, nSDs);
    gpuErrchk(cudaDeviceSynchronize());
    d_scratch_space = (void*)stateOfSolver_resources.pDeviceMemoryScratchSpace(temp_storage_bytes);
    // Run CUB exclusive prefix sum
    cub::DeviceScan::ExclusiveSum(d_scratch_space, temp_storage_bytes, in_ptr, out_ptr, nSDs);
    gpuErrchk(cudaDeviceSynchronize());
}

__global__ void interactionGranMat_TriangleSoup_matBased(ChSystemGpuMesh_impl::TriangleSoupPtr d_triangleSoup,
                                                         ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                                         const unsigned int* SD_trianglesInEachSD_composite,
                                                         const unsigned int* SD_numTrianglesTouching,
                                                         const unsigned int* SD_TrianglesCompositeOffsets,
                                                         ChSystemGpu_impl::GranParamsPtr gran_params,
                                                         ChSystemGpuMesh_impl::MeshParamsPtr mesh_params,
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
    unsigned int sphereIDGlobal = NULL_CHGPU_ID;

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

        if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
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
            size_t SD_composite_offset = SD_TrianglesCompositeOffsets[thisSD];
            if (SD_composite_offset == NULL_CHGPU_ID) {
                ABORTABORTABORT("Invalid composite offset %lu for SD %u, touching %u triangles\n", NULL_CHGPU_ID,
                                thisSD, numSDTriangles);
            }
            size_t offset_in_composite_Array = SD_composite_offset + local_ID;

            unsigned int globalID = SD_trianglesInEachSD_composite[offset_in_composite_Array];
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
                // normal points from triangle to sphere
                float3 delta = -depth * normal;

                // effective radius is just sphere radius -- assume meshes are locally flat (a safe assumption?)
                // float hertz_force_factor = sqrt(abs(depth) / gran_params->sphereRadius_SU);

                // helper variables
                float sqrt_Rd = sqrt(abs(depth) * gran_params->sphereRadius_SU);
                float Sn = 2. * mesh_params->E_eff_s2m_SU * sqrt_Rd;

                float loge = (mesh_params->COR_s2m_SU < EPSILON) ? log(EPSILON) : log(mesh_params->COR_s2m_SU);
                float beta = loge / sqrt(loge * loge + CUDART_PI_F * CUDART_PI_F);

                // effective mass = mass_mesh * mass_sphere / (m_mesh + mass_sphere)
                float fam_mass_SU = d_triangleSoup->familyMass_SU[fam];
                const float sphere_mass_SU = gran_params->sphere_mass_SU;
                float m_eff = sphere_mass_SU * fam_mass_SU / (sphere_mass_SU + fam_mass_SU);

                // stiffness and damping coefficient
                float kn = (2.0 / 3.0) * Sn;
                float gn = 2 * sqrt(5.0 / 6.0) * beta * sqrt(Sn * m_eff);
                // relative velocity = v_sphere - v_mesh
                float3 v_rel = sphere_vel[sphereIDLocal] - d_triangleSoup->vel[fam];

                // assumes pos is the center of mass of the mesh
                float3 meshCenter =
                    make_float3(mesh_params->fam_frame_broad[fam].pos[0], mesh_params->fam_frame_broad[fam].pos[1],
                                mesh_params->fam_frame_broad[fam].pos[2]);
                convert_pos_UU2SU<float3>(meshCenter, gran_params);

                // NOTE depth is negative and normal points from triangle to sphere center
                float3 r = pt1_float + normal * (depth / 2) - meshCenter;

                // Add angular velocity contribution from mesh
                v_rel = v_rel - Cross(d_triangleSoup->omega[fam], r);

                // add tangential components if they exist
                if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
                    // Vector from the center of sphere to center of contact volume
                    float3 r_A = -(gran_params->sphereRadius_SU + depth / 2.f) * normal;
                    v_rel = v_rel + Cross(omega[sphereIDLocal], r_A);
                }

                // normal component of relative velocity
                float projection = Dot(v_rel, normal);

                // tangential component of relative velocity
                float3 vrel_t = v_rel - projection * normal;

                // normal force magnitude
                float forceN_mag = -kn * depth + gn * projection;

                float3 force_accum = forceN_mag * normal;

                // Compute force updates for adhesion term, opposite the spring term
                // NOTE ratio is wrt the weight of a sphere of mass 1
                // NOTE the cancelation of two negatives
                force_accum = force_accum + gran_params->sphere_mass_SU * mesh_params->adhesionAcc_s2m * delta / depth;

                // tangential component
                if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
                    // radius pointing from the contact point to the center of particle
                    float3 Rc = (gran_params->sphereRadius_SU + depth / 2.f) * normal;
                    float3 roll_ang_acc = computeRollingAngAcc(
                        sphere_data, gran_params, mesh_params->rolling_coeff_s2m_SU, mesh_params->spinning_coeff_s2m_SU,
                        force_accum, omega[sphereIDLocal], d_triangleSoup->omega[fam], Rc);

                    sphere_AngAcc = sphere_AngAcc + roll_ang_acc;

                    unsigned int BC_histmap_label = triangleFamilyHistmapOffset + fam;

                    // compute tangent force
                    float3 tangent_force = computeFrictionForces_matBased(
                        gran_params, sphere_data, sphereIDGlobal, BC_histmap_label,
                        mesh_params->static_friction_coeff_s2m, mesh_params->E_eff_s2m_SU, mesh_params->G_eff_s2m_SU,
                        sqrt_Rd, beta, force_accum, vrel_t, normal, m_eff);

                    ////float force_unit = gran_params->MASS_UNIT * gran_params->LENGTH_UNIT /
                    ////                   (gran_params->TIME_UNIT * gran_params->TIME_UNIT);

                    ////float velocity_unit = gran_params->LENGTH_UNIT / gran_params->TIME_UNIT;

                    force_accum = force_accum + tangent_force;
                    sphere_AngAcc =
                        sphere_AngAcc + Cross(-1.f * normal, tangent_force) / gran_params->sphereInertia_by_r;
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

        if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
            // write back torques for later
            atomicAdd(sphere_data->sphere_ang_acc_X + sphereIDGlobal, sphere_AngAcc.x);
            atomicAdd(sphere_data->sphere_ang_acc_Y + sphereIDGlobal, sphere_AngAcc.y);
            atomicAdd(sphere_data->sphere_ang_acc_Z + sphereIDGlobal, sphere_AngAcc.z);
        }
    }  // end sphere id check
}  // end kernel

/// <summary>
/// Kernel accounts for the interaction between the granular material and the triangles making up the triangle soup
/// </summary>
/// <param name="d_triangleSoup">- information about triangle soup (in device mem.)</param>
/// <param name="sphere_data">- data structure containing pointers to granular-material related info</param>
/// <param name="SD_trianglesInEachSD_composite">- array saying which triangles touch an SD; has information for each
/// SD</param> <param name="SD_numTrianglesTouching">- number of triangles touching each SD</param> <param
/// name="SD_TrianglesCompositeOffsets">- offsets in the composite array for each SD; where each SD starts storing its
/// triangles</param> <param name="gran_params">- parameters associated with the granular material</param> <param
/// name="mesh_params">- parameters associated with the triangle soup</param> <param
/// name="triangleFamilyHistmapOffset">- offset in the array of friction history (?)</param> <returns></returns>
__global__ void interactionGranMat_TriangleSoup(ChSystemGpuMesh_impl::TriangleSoupPtr d_triangleSoup,
                                                ChSystemGpu_impl::GranSphereDataPtr sphere_data,
                                                const unsigned int* SD_trianglesInEachSD_composite,
                                                const unsigned int* SD_numTrianglesTouching,
                                                const unsigned int* SD_TrianglesCompositeOffsets,
                                                ChSystemGpu_impl::GranParamsPtr gran_params,
                                                ChSystemGpuMesh_impl::MeshParamsPtr mesh_params,
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
    unsigned int sphereIDGlobal = NULL_CHGPU_ID;

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

        if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
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
            size_t SD_composite_offset = SD_TrianglesCompositeOffsets[thisSD];
            if (SD_composite_offset == NULL_CHGPU_ID) {
                ABORTABORTABORT("Invalid composite offset %lu for SD %u, touching %u triangles\n", NULL_CHGPU_ID,
                                thisSD, numSDTriangles);
            }
            size_t offset_in_composite_Array = SD_composite_offset + local_ID;

            unsigned int globalID = SD_trianglesInEachSD_composite[offset_in_composite_Array];
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
                if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
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

                if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
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

                    ////float force_unit = gran_params->MASS_UNIT * gran_params->LENGTH_UNIT /
                    ////                   (gran_params->TIME_UNIT * gran_params->TIME_UNIT);

                    ////float velocity_unit = gran_params->LENGTH_UNIT / gran_params->TIME_UNIT;

                    force_accum = force_accum + tangent_force;
                    sphere_AngAcc =
                        sphere_AngAcc + Cross(-1.f * normal, tangent_force) / gran_params->sphereInertia_by_r;
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

        if (gran_params->friction_mode != chrono::gpu::CHGPU_FRICTION_MODE::FRICTIONLESS) {
            // write back torques for later
            atomicAdd(sphere_data->sphere_ang_acc_X + sphereIDGlobal, sphere_AngAcc.x);
            atomicAdd(sphere_data->sphere_ang_acc_Y + sphereIDGlobal, sphere_AngAcc.y);
            atomicAdd(sphere_data->sphere_ang_acc_Z + sphereIDGlobal, sphere_AngAcc.z);
        }
    }  // end sphere id check
}  // end kernel

__host__ double ChSystemGpuMesh_impl::AdvanceSimulation(float duration) {
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

    float time_elapsed_SU = 0.f;  // time elapsed in this call (SU)
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (; time_elapsed_SU < stepSize_SU * nsteps; time_elapsed_SU += stepSize_SU) {
        updateBCPositions();
        runSphereBroadphase();

        resetSphereAccelerations();
        resetBCForces();
        if (meshSoup->nTrianglesInSoup != 0 && mesh_collision_enabled) {
            gpuErrchk(
                cudaMemset(meshSoup->generalizedForcesPerFamily, 0, 6 * meshSoup->numTriangleFamilies * sizeof(float)));
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup->nTrianglesInSoup != 0 && mesh_collision_enabled) {
            runTriangleBroadphase();
        }

        METRICS_PRINTF("Starting computeSphereForces!\n");

        if (gran_params->friction_mode == CHGPU_FRICTION_MODE::FRICTIONLESS) {
            // Compute sphere-sphere forces
            if (gran_params->use_mat_based == true) {
                METRICS_PRINTF("use material based model\n");
                computeSphereForces_frictionless_matBased<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                    sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                    (unsigned int)BC_params_list_SU.size());

            } else {
                METRICS_PRINTF("use user defined model\n");
                computeSphereForces_frictionless<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                    sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                    (unsigned int)BC_params_list_SU.size());
            }
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }
        // frictional contact
        else if (gran_params->friction_mode == CHGPU_FRICTION_MODE::SINGLE_STEP ||
                 gran_params->friction_mode == CHGPU_FRICTION_MODE::MULTI_STEP) {
            // figure out who is contacting
            determineContactPairs<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(sphere_data, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            METRICS_PRINTF("Frictional case.\n");
            if (gran_params->use_mat_based == true) {
                METRICS_PRINTF("compute sphere-sphere and sphere-bc mat based\n");
                computeSphereContactForces_matBased<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                    sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                    (unsigned int)BC_params_list_SU.size(), nSpheres);
            } else {
                METRICS_PRINTF("compute sphere-sphere and sphere-bc user defined\n");
                computeSphereContactForces<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                    sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(),
                    (unsigned int)BC_params_list_SU.size(), nSpheres);
            }
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup->numTriangleFamilies != 0 && mesh_collision_enabled) {
            // TODO please do not use a template here
            // triangle labels come after BC labels numerically
            unsigned int triangleFamilyHistmapOffset =
                gran_params->nSpheres + 1 + (unsigned int)BC_params_list_SU.size() + 1;
            // compute sphere-triangle forces
            if (tri_params->use_mat_based == true) {
                interactionGranMat_TriangleSoup_matBased<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                    meshSoup, sphere_data, SD_trianglesInEachSD_composite.data(), SD_numTrianglesTouching.data(),
                    SD_TrianglesCompositeOffsets.data(), gran_params, tri_params, triangleFamilyHistmapOffset);
            } else {
                //   //              printf("compute sphere-mesh user defined\n");

                interactionGranMat_TriangleSoup<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                    meshSoup, sphere_data, SD_trianglesInEachSD_composite.data(), SD_numTrianglesTouching.data(),
                    SD_TrianglesCompositeOffsets.data(), gran_params, tri_params, triangleFamilyHistmapOffset);
            }
        }

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        METRICS_PRINTF("Starting integrateSpheres!\n");
        integrateSpheres<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (gran_params->friction_mode != CHGPU_FRICTION_MODE::FRICTIONLESS) {
            const unsigned int nThreadsUpdateHist = 2 * CUDA_THREADS_PER_BLOCK;
            unsigned int fricMapSize = nSpheres * MAX_SPHERES_TOUCHED_BY_SPHERE;
            unsigned int nBlocksFricHistoryPostProcess = (fricMapSize + nThreadsUpdateHist - 1) / nThreadsUpdateHist;
            updateFrictionData<<<nBlocksFricHistoryPostProcess, nThreadsUpdateHist>>>(fricMapSize, sphere_data,
                                                                                      gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            updateAngVels<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }

        elapsedSimTime += (float)(stepSize_SU * TIME_SU2UU);  // Advance current time
    }

    return time_elapsed_SU * TIME_SU2UU;  // return elapsed UU time
}
}  // namespace gpu
}  // namespace chrono
