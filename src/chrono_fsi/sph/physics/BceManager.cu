// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu, Luning Bakke
// =============================================================================
//
// Base class for processing boundary condition enforcing (bce) markers forces
// in FSI system.
// TODO: There need to be a better way to compute bce marker forces for different solvers.
// For explicit solver, it is essentially derivVelRhoD times the marker mass,
// For I2SPH, it is derivVelRhoD only
// =============================================================================

#include <type_traits>

#include "chrono_fsi/sph/physics/BceManager.cuh"
#include "chrono_fsi/sph/physics/ChSphGeneral.cuh"

namespace chrono {
namespace fsi {
namespace sph {

// -----------------------------------------------------------------------------

__device__ double atomicAdd_double(double* address, double val) {
    unsigned long long int* address_as_ull = (unsigned long long int*)address;
    unsigned long long int old = *address_as_ull, assumed;

    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(val + __longlong_as_double(assumed)));
    } while (assumed != old);

    return __longlong_as_double(old);
}

// -----------------------------------------------------------------------------

__global__ void Populate_RigidSPH_MeshPos_LRF_D(Real3* rigid_BCEcoords_D,
                                                Real4* posRadD,
                                                uint* rigid_BCEsolids_D,
                                                Real3* posRigidD,
                                                Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numRigidMarkers)
        return;

    int rigidIndex = rigid_BCEsolids_D[index];
    uint rigidMarkerIndex = index + countersD.startRigidMarkers;
    Real4 q4 = qD[rigidIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);
    Real3 dist3 = mR3(posRadD[rigidMarkerIndex]) - posRigidD[rigidIndex];
    Real3 dist3LF = InverseRotate_By_RotationMatrix_DeviceHost(a1, a2, a3, dist3);

    // Save the coordinates in the local reference of a rigid body
    rigid_BCEcoords_D[index] = dist3LF;
}

__global__ void CalcRigidForces_D(Real3* __restrict__ rigid_FSI_ForcesD,
                                  Real3* __restrict__ rigid_FSI_TorquesD,
                                  const uint* __restrict__ rigidBodyBlockValidThreads,
                                  const uint* __restrict__ rigidBodyAccumulatedPaddedThreads,
                                  const Real4* __restrict__ derivVelRhoD,
                                  const Real4* __restrict__ posRadD,
                                  const uint* __restrict__ rigid_BCEsolids_D,
                                  const Real3* __restrict__ posRigidD,
                                  const uint* __restrict__ mapOriginalToSorted,
                                  const uint numRigidMarkers,
                                  const Real markerMass,
                                  const uint startRigidMarkers,
                                  const SPHMethod sph_method) {
    extern __shared__ char sharedMem[];
    const uint blockSize = blockDim.x;

    // Shared memory allocations
    Real3* sharedForces = (Real3*)sharedMem;                  // Size: blockSize
    Real3* sharedTorques = (Real3*)&sharedForces[blockSize];  // Size: blockSize

    uint threadIdx_x = threadIdx.x;
    uint globalIndex = blockIdx.x * blockDim.x + threadIdx_x;

    // Valid threads in current block
    uint validThreads = rigidBodyBlockValidThreads[blockIdx.x];
    // Valid threads in previous block
    uint paddedThreads = rigidBodyAccumulatedPaddedThreads[blockIdx.x];

    uint globalIndex_with_padding = globalIndex - paddedThreads;

    if (globalIndex_with_padding >= numRigidMarkers)
        return;

    uint rigidMarkerIndex = globalIndex_with_padding + startRigidMarkers;

    uint sortedIndex = mapOriginalToSorted[rigidMarkerIndex];

    // Get RigidIndex for the current marker
    uint RigidIndex = rigid_BCEsolids_D[globalIndex_with_padding];

    Real3 Force = make_Real3(0.0f, 0.0f, 0.0f);
    Real3 Torque = make_Real3(0.0f, 0.0f, 0.0f);
    if (threadIdx_x < validThreads) {
        if (sph_method == SPHMethod::WCSPH) {
            Force = mR3(derivVelRhoD[sortedIndex]) * markerMass;
        } else {
            Force = mR3(derivVelRhoD[sortedIndex]);
        }

        Real3 dist3 = mR3(posRadD[sortedIndex]) - posRigidD[RigidIndex];
        Torque = cross(dist3, Force);
    }

    sharedForces[threadIdx_x] = Force;
    sharedTorques[threadIdx_x] = Torque;

    __syncthreads();

    // Standard block wise reduction - each block only contains elements from a single rigid body
    for (uint stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (threadIdx_x < stride && threadIdx_x + stride < validThreads) {
            sharedForces[threadIdx_x].x += sharedForces[threadIdx_x + stride].x;
            sharedForces[threadIdx_x].y += sharedForces[threadIdx_x + stride].y;
            sharedForces[threadIdx_x].z += sharedForces[threadIdx_x + stride].z;

            sharedTorques[threadIdx_x].x += sharedTorques[threadIdx_x + stride].x;
            sharedTorques[threadIdx_x].y += sharedTorques[threadIdx_x + stride].y;
            sharedTorques[threadIdx_x].z += sharedTorques[threadIdx_x + stride].z;
        }
        __syncthreads();
    }

    if (threadIdx_x == 0) {
        // Atomic addition to global arrays
        atomicAdd(&rigid_FSI_ForcesD[RigidIndex].x, sharedForces[0].x);
        atomicAdd(&rigid_FSI_ForcesD[RigidIndex].y, sharedForces[0].y);
        atomicAdd(&rigid_FSI_ForcesD[RigidIndex].z, sharedForces[0].z);

        atomicAdd(&rigid_FSI_TorquesD[RigidIndex].x, sharedTorques[0].x);
        atomicAdd(&rigid_FSI_TorquesD[RigidIndex].y, sharedTorques[0].y);
        atomicAdd(&rigid_FSI_TorquesD[RigidIndex].z, sharedTorques[0].z);
    }
}

__global__ void CalcFlex1DForces_D(Real3* __restrict__ flex1D_FSIforces_D,
                                   const Real4* __restrict__ derivVelRhoD,
                                   const uint2* __restrict__ flex1D_Nodes_D,
                                   const uint3* __restrict__ flex1D_BCEsolids_D,
                                   const Real3* __restrict__ flex1D_BCEcoords_D,
                                   const uint* __restrict__ mapOriginalToSorted,
                                   const uint numFlexMarkers1D,
                                   const uint startFlexMarkers1D,
                                   const Real markerMass,
                                   const SPHMethod sph_method) {
    extern __shared__ char sharedMem[];
    // Each thread handles a segment which contains 2 nodes
    const int maxNodesPerMarker = 2;
    const int threadsPerBlock = blockDim.x;

    // Shared memory for node indices and forces
    uint* sharedNodeIndices = (uint*)sharedMem;  // Size: maxNodesPerMarker * threadsPerBlock
    Real3* sharedForces = (Real3*)&sharedNodeIndices[maxNodesPerMarker * threadsPerBlock];  // Same size

    uint globalIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (globalIndex >= numFlexMarkers1D)
        return;

    uint flex_index = globalIndex + startFlexMarkers1D;
    uint sortedIndex = mapOriginalToSorted[flex_index];

    // Read data
    uint3 flex_solid = flex1D_BCEsolids_D[globalIndex];
    uint flex_seg = flex_solid.z;

    Real3 Force;
    if (paramsD.sph_method == SPHMethod::WCSPH) {
        Force = mR3(derivVelRhoD[sortedIndex]) * paramsD.markerMass;
    } else {
        Force = mR3(derivVelRhoD[sortedIndex]);
    }
    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];
    uint n0 = seg_nodes.x;
    uint n1 = seg_nodes.y;

    Real lambda0 = flex1D_BCEcoords_D[globalIndex].x;
    Real lambda1 = 1 - lambda0;

    // Compute contributions to nodes
    Real3 force_n0 = Force * lambda0;
    Real3 force_n1 = Force * lambda1;

    // Store node indices and forces in shared memory
    sharedNodeIndices[threadIdx.x * maxNodesPerMarker + 0] = n0;
    sharedNodeIndices[threadIdx.x * maxNodesPerMarker + 1] = n1;

    sharedForces[threadIdx.x * maxNodesPerMarker + 0] = force_n0;
    sharedForces[threadIdx.x * maxNodesPerMarker + 1] = force_n1;

    __syncthreads();

    // Now perform reduction per node within the block
    // We'll use a simple approach since nodes can be shared among markers
    for (int i = 0; i < maxNodesPerMarker; ++i) {
        uint nodeIndex = sharedNodeIndices[threadIdx.x * maxNodesPerMarker + i];
        Real3 force = sharedForces[threadIdx.x * maxNodesPerMarker + i];

        // Use atomic operations to accumulate forces per node
        atomicAdd(&flex1D_FSIforces_D[nodeIndex].x, force.x);
        atomicAdd(&flex1D_FSIforces_D[nodeIndex].y, force.y);
        atomicAdd(&flex1D_FSIforces_D[nodeIndex].z, force.z);
    }
}

__global__ void CalcFlex2DForces_D(Real3* __restrict__ flex2D_FSIforces_D,
                                   const Real4* __restrict__ derivVelRhoD,
                                   const uint3* __restrict__ flex2D_Nodes_D,
                                   const uint3* __restrict__ flex2D_BCEsolids_D,
                                   const Real3* __restrict__ flex2D_BCEcoords_D,
                                   const uint* __restrict__ mapOriginalToSorted,
                                   const uint numFlexMarkers2D,
                                   const uint startFlexMarkers2D,
                                   const Real markerMass,
                                   const SPHMethod sph_method) {
    extern __shared__ char sharedMem[];

    // Each marker deals with 3 nodes in the 2D case
    const int maxNodesPerMarker = 3;
    const int threadsPerBlock = blockDim.x;

    // Shared memory for node indices and forces
    uint* sharedNodeIndices = (uint*)sharedMem;  // Size: maxNodesPerMarker * threadsPerBlock
    Real3* sharedForces = (Real3*)&sharedNodeIndices[maxNodesPerMarker * threadsPerBlock];  // Same size

    uint globalIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (globalIndex >= numFlexMarkers2D)
        return;

    uint flex_index = globalIndex + startFlexMarkers2D;
    uint sortedIndex = mapOriginalToSorted[flex_index];

    // Read data
    uint3 flex_solid = flex2D_BCEsolids_D[globalIndex];
    uint flex_tri = flex_solid.z;

    Real3 Force;
    if (paramsD.sph_method == SPHMethod::WCSPH) {
        Force = mR3(derivVelRhoD[sortedIndex]) * markerMass;
    } else {
        Force = mR3(derivVelRhoD[sortedIndex]);
    }

    uint3 tri_nodes = flex2D_Nodes_D[flex_tri];
    uint n0 = tri_nodes.x;
    uint n1 = tri_nodes.y;
    uint n2 = tri_nodes.z;

    Real lambda0 = flex2D_BCEcoords_D[globalIndex].x;
    Real lambda1 = flex2D_BCEcoords_D[globalIndex].y;
    Real lambda2 = 1.0f - lambda0 - lambda1;

    // Compute contributions to nodes
    Real3 force_n0 = Force * lambda0;
    Real3 force_n1 = Force * lambda1;
    Real3 force_n2 = Force * lambda2;

    // Store node indices and forces in shared memory
    sharedNodeIndices[threadIdx.x * maxNodesPerMarker + 0] = n0;
    sharedNodeIndices[threadIdx.x * maxNodesPerMarker + 1] = n1;
    sharedNodeIndices[threadIdx.x * maxNodesPerMarker + 2] = n2;

    sharedForces[threadIdx.x * maxNodesPerMarker + 0] = force_n0;
    sharedForces[threadIdx.x * maxNodesPerMarker + 1] = force_n1;
    sharedForces[threadIdx.x * maxNodesPerMarker + 2] = force_n2;

    __syncthreads();

    // Now perform accumulation per node within the block
    // We'll use a simple approach since nodes can be shared among markers
    for (int i = 0; i < maxNodesPerMarker; ++i) {
        uint nodeIndex = sharedNodeIndices[threadIdx.x * maxNodesPerMarker + i];
        Real3 force = sharedForces[threadIdx.x * maxNodesPerMarker + i];

        // Use atomic operations to accumulate forces per node
        atomicAdd(&flex2D_FSIforces_D[nodeIndex].x, force.x);
        atomicAdd(&flex2D_FSIforces_D[nodeIndex].y, force.y);
        atomicAdd(&flex2D_FSIforces_D[nodeIndex].z, force.z);
    }
}

// -----------------------------------------------------------------------------

__global__ void CalcRigidBceAccelerationD(Real3* bceAcc,
                                          Real4* q_fsiBodies_D,
                                          Real3* accRigid_fsiBodies_D,
                                          Real3* omegaVelLRF_fsiBodies_D,
                                          Real3* omegaAccLRF_fsiBodies_D,
                                          Real3* rigid_BCEcoords_D,
                                          const uint* rigid_BCEsolids_D,
                                          const uint* mapOriginalToSorted) {
    uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (bceIndex >= countersD.numRigidMarkers)
        return;

    uint rigidMarkerIndex = bceIndex + countersD.startRigidMarkers;
    uint sortedIndex = mapOriginalToSorted[rigidMarkerIndex];

    int rigidBodyIndex = rigid_BCEsolids_D[bceIndex];

    // linear acceleration (CM)
    Real3 acc3 = accRigid_fsiBodies_D[rigidBodyIndex];

    Real4 q4 = q_fsiBodies_D[rigidBodyIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);
    Real3 wVel3 = omegaVelLRF_fsiBodies_D[rigidBodyIndex];
    Real3 rigidSPH_MeshPos_LRF = rigid_BCEcoords_D[bceIndex];
    Real3 wVelCrossS = cross(wVel3, rigidSPH_MeshPos_LRF);
    Real3 wVelCrossWVelCrossS = cross(wVel3, wVelCrossS);

    // centrigugal acceleration
    acc3 += mR3(dot(a1, wVelCrossWVelCrossS), dot(a2, wVelCrossWVelCrossS), dot(a3, wVelCrossWVelCrossS));

    Real3 wAcc3 = omegaAccLRF_fsiBodies_D[rigidBodyIndex];
    Real3 wAccCrossS = cross(wAcc3, rigidSPH_MeshPos_LRF);

    // tangential acceleration
    acc3 += mR3(dot(a1, wAccCrossS), dot(a2, wAccCrossS), dot(a3, wAccCrossS));

    bceAcc[sortedIndex] = acc3;
}

__global__ void CalcFlex1DBceAcceleration_D(
    Real3* bceAcc,              // [num BCEs on all solids]  marker accelerations (output)
    Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 1D nodes
    uint2* flex1D_Nodes_D,      // [num segments]            node indices for each 1D segment
    uint3* flex1D_BCEsolids_D,  // [num BCEs on 1D segments] association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D,  // [num BCEs on 1D segments] local coordinates of BCE markers on FEA 1-D segments
    const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers1D)
        return;

    uint sortedIndex = mapOriginalToSorted[index + countersD.startFlexMarkers1D];

    uint3 flex_solid = flex1D_BCEsolids_D[index];  // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;             // index of segment in associated mesh
    uint flex_seg = flex_solid.z;  // index of segment in global list

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 A0 = acc_fsi_fea_D[seg_nodes.x];       // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[seg_nodes.y];       // (absolute) acceleration of node 1

    Real lambda0 = flex1D_BCEcoords_D[index].x;  // segment coordinate
    Real lambda1 = 1 - lambda0;                  // segment coordinate

    bceAcc[sortedIndex] = A0 * lambda0 + A1 * lambda1;
}

__global__ void CalcFlex2DBceAcceleration_D(
    Real3* bceAcc,              // [num BCEs on all solids]  marker accelerations (output)
    Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 2D nodes
    uint3* flex2D_Nodes_D,      // [num triangles]           triangle node indices
    uint3* flex2D_BCEsolids_D,  // [num BCEs on 1D segments] association of flex BCEs with a mesh and face
    Real3* flex2D_BCEcoords_D,  // [num BCEs on 1D segments] local coordinates of BCE markers on FEA 2-D faces
    const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers2D)
        return;

    uint sortedIndex = mapOriginalToSorted[index + countersD.startFlexMarkers2D];

    uint3 flex_solid = flex2D_BCEsolids_D[index];  // associated flex mesh and face
    ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
    ////uint flex_mesh_tri = flex_solid.y;             // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;  // index of triangle in global list

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    Real3 A0 = acc_fsi_fea_D[tri_nodes.x];      // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[tri_nodes.y];      // (absolute) acceleration of node 1
    Real3 A2 = acc_fsi_fea_D[tri_nodes.z];      // (absolute) acceleration of node 2

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate

    bceAcc[sortedIndex] = A0 * lambda0 + A1 * lambda1 + A2 * lambda2;
}

// -----------------------------------------------------------------------------

__global__ void UpdateBodyMarkerState_D(Real4* posRadD,
                                        Real3* velMasD,
                                        Real3* rigid_BCEcoords_D,
                                        uint* rigid_BCEsolids_D,
                                        Real3* posRigidD,
                                        Real3* velRigidD,
                                        Real3* omegaLRF_D,
                                        Real4* qD,
                                        const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numRigidMarkers)
        return;

    uint rigidMarkerIndex = index + countersD.startRigidMarkers;
    uint sortedIndex = mapOriginalToSorted[rigidMarkerIndex];
    int rigidBodyIndex = rigid_BCEsolids_D[index];

    Real4 q4 = qD[rigidBodyIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);

    Real3 rigidSPH_MeshPos_LRF = rigid_BCEcoords_D[index];

    // position
    Real h = posRadD[sortedIndex].w;
    Real3 p_Rigid = posRigidD[rigidBodyIndex];
    Real3 pos =
        p_Rigid + mR3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF), dot(a3, rigidSPH_MeshPos_LRF));
    posRadD[sortedIndex] = mR4(pos, h);

    // velocity
    Real3 v_Rigid = velRigidD[rigidBodyIndex];
    Real3 omega3 = omegaLRF_D[rigidBodyIndex];
    Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
    velMasD[sortedIndex] = v_Rigid + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
}

__global__ void UpdateMeshMarker1DState_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
    uint2* flex1D_Nodes_D,      // segment node indices
    uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D,  // local coordinates of BCE markers on FEA 1-D segments
    const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers1D)
        return;

    uint flex_index = index + countersD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint sortedIndex = mapOriginalToSorted[flex_index];
    uint3 flex_solid = flex1D_BCEsolids_D[index];  // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    uint flex_seg = flex_solid.z;  // index of segment in global list

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 P0 = pos_fsi_fea_D[seg_nodes.x];       // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[seg_nodes.y];       // (absolute) position of node 1
    Real3 V0 = vel_fsi_fea_D[seg_nodes.x];       // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[seg_nodes.y];       // (absolute) velocity of node 1

    Real3 x_dir = normalize(P1 - P0);
    Real3 y_dir = mR3(-x_dir.y - x_dir.z, x_dir.x - x_dir.z, x_dir.x + x_dir.y);
    y_dir = y_dir / length(y_dir);
    Real3 z_dir = cross(x_dir, y_dir);

    Real lambda0 = flex1D_BCEcoords_D[index].x;  // segment coordinate
    Real lambda1 = 1 - lambda0;                  // segment coordinate
    Real y_val = flex1D_BCEcoords_D[index].y;    // off-segment y coordinate
    Real z_val = flex1D_BCEcoords_D[index].z;    // off-segment z coordinate

    Real3 P = P0 * lambda0 + P1 * lambda1 + y_val * y_dir + z_val * z_dir;  // BCE marker position
    Real3 V = V0 * lambda0 + V1 * lambda1;                                  // BCE marker velocity

    Real h = posRadD[sortedIndex].w;
    posRadD[sortedIndex] = mR4(P, h);
    velMasD[sortedIndex] = V;
}

__global__ void UpdateMeshMarker2DState_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 2-D face nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 2-D face nodes
    uint3* flex2D_Nodes_D,      // triangle node indices
    uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
    Real3* flex2D_BCEcoords_D,  // local coordinates of BCE markers on FEA 2-D faces
    const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers2D)
        return;

    uint flex_index = index + countersD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint sortedIndex = mapOriginalToSorted[flex_index];
    uint3 flex_solid = flex2D_BCEsolids_D[index];  // associated flex mesh and face
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;  // index of triangle in global list

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    Real3 P0 = pos_fsi_fea_D[tri_nodes.x];      // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[tri_nodes.y];      // (absolute) position of node 1
    Real3 P2 = pos_fsi_fea_D[tri_nodes.z];      // (absolute) position of node 2
    Real3 V0 = vel_fsi_fea_D[tri_nodes.x];      // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[tri_nodes.y];      // (absolute) velocity of node 1
    Real3 V2 = vel_fsi_fea_D[tri_nodes.z];      // (absolute) velocity of node 2

    Real3 normal = normalize(cross(P1 - P0, P2 - P1));

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate
    Real z_val = flex2D_BCEcoords_D[index].z;    // off-face coordinate

    Real3 P = P0 * lambda0 + P1 * lambda1 + P2 * lambda2 + z_val * normal;  // BCE marker position
    Real3 V = V0 * lambda0 + V1 * lambda1 + V2 * lambda2;                   // BCE marker velocity

    Real h = posRadD[sortedIndex].w;
    posRadD[sortedIndex] = mR4(P, h);
    velMasD[sortedIndex] = V;
}

__global__ void UpdateBodyMarkerStateUnsorted_D(Real4* posRadD,
                                                Real3* velMasD,
                                                Real3* rigid_BCEcoords_D,
                                                uint* rigid_BCEsolids_D,
                                                Real3* posRigidD,
                                                Real3* velRigidD,
                                                Real3* omegaLRF_D,
                                                Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numRigidMarkers)
        return;

    uint rigidMarkerIndex = index + countersD.startRigidMarkers;
    int rigidBodyIndex = rigid_BCEsolids_D[index];

    Real4 q4 = qD[rigidBodyIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);

    Real3 rigidSPH_MeshPos_LRF = rigid_BCEcoords_D[index];

    // position
    Real h = posRadD[rigidMarkerIndex].w;
    Real3 p_Rigid = posRigidD[rigidBodyIndex];
    Real3 pos =
        p_Rigid + mR3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF), dot(a3, rigidSPH_MeshPos_LRF));
    posRadD[rigidMarkerIndex] = mR4(pos, h);

    // velocity
    Real3 v_Rigid = velRigidD[rigidBodyIndex];
    Real3 omega3 = omegaLRF_D[rigidBodyIndex];
    Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
    velMasD[rigidMarkerIndex] = v_Rigid + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
}

__global__ void UpdateMeshMarker1DStateUnsorted_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
    uint2* flex1D_Nodes_D,      // segment node indices
    uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D   // local coordinates of BCE markers on FEA 1-D segments
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers1D)
        return;

    uint flex_index = index + countersD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint3 flex_solid = flex1D_BCEsolids_D[index];            // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    uint flex_seg = flex_solid.z;  // index of segment in global list

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 P0 = pos_fsi_fea_D[seg_nodes.x];       // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[seg_nodes.y];       // (absolute) position of node 1
    Real3 V0 = vel_fsi_fea_D[seg_nodes.x];       // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[seg_nodes.y];       // (absolute) velocity of node 1

    Real3 x_dir = normalize(P1 - P0);
    Real3 y_dir = mR3(-x_dir.y - x_dir.z, x_dir.x - x_dir.z, x_dir.x + x_dir.y);
    y_dir = y_dir / length(y_dir);
    Real3 z_dir = cross(x_dir, y_dir);

    Real lambda0 = flex1D_BCEcoords_D[index].x;  // segment coordinate
    Real lambda1 = 1 - lambda0;                  // segment coordinate
    Real y_val = flex1D_BCEcoords_D[index].y;    // off-segment y coordinate
    Real z_val = flex1D_BCEcoords_D[index].z;    // off-segment z coordinate

    Real3 P = P0 * lambda0 + P1 * lambda1 + y_val * y_dir + z_val * z_dir;  // BCE marker position
    Real3 V = V0 * lambda0 + V1 * lambda1;                                  // BCE marker velocity

    Real h = posRadD[flex_index].w;
    posRadD[flex_index] = mR4(P, h);
    velMasD[flex_index] = V;
}

__global__ void UpdateMeshMarker2DStateUnsorted_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 2-D face nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 2-D face nodes
    uint3* flex2D_Nodes_D,      // triangle node indices
    uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
    Real3* flex2D_BCEcoords_D   // local coordinates of BCE markers on FEA 2-D faces
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers2D)
        return;

    uint flex_index = index + countersD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint3 flex_solid = flex2D_BCEsolids_D[index];            // associated flex mesh and face
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;  // index of triangle in global list

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    Real3 P0 = pos_fsi_fea_D[tri_nodes.x];      // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[tri_nodes.y];      // (absolute) position of node 1
    Real3 P2 = pos_fsi_fea_D[tri_nodes.z];      // (absolute) position of node 2
    Real3 V0 = vel_fsi_fea_D[tri_nodes.x];      // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[tri_nodes.y];      // (absolute) velocity of node 1
    Real3 V2 = vel_fsi_fea_D[tri_nodes.z];      // (absolute) velocity of node 2

    Real3 normal = normalize(cross(P1 - P0, P2 - P1));

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate
    Real z_val = flex2D_BCEcoords_D[index].z;    // off-face coordinate

    Real3 P = P0 * lambda0 + P1 * lambda1 + P2 * lambda2 + z_val * normal;  // BCE marker position
    Real3 V = V0 * lambda0 + V1 * lambda1 + V2 * lambda2;                   // BCE marker velocity

    Real h = posRadD[flex_index].w;
    posRadD[flex_index] = mR4(P, h);
    velMasD[flex_index] = V;
}

// =============================================================================

BceManager::BceManager(FsiDataManager& data_mgr, bool verbose) : m_data_mgr(data_mgr), m_verbose(verbose) {
    m_totalForceRigid.resize(0);
    m_totalTorqueRigid.resize(0);
    m_rigidBodyBlockSize = 512;
    m_rigidBodyGridSize = 0;
}

BceManager::~BceManager() {}

// -----------------------------------------------------------------------------

void BceManager::Initialize(std::vector<int> fsiBodyBceNum) {
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));

    // Resizing the arrays used to modify the BCE velocity and pressure according to Adami
    m_totalForceRigid.resize(m_data_mgr.countersH->numFsiBodies);
    m_totalTorqueRigid.resize(m_data_mgr.countersH->numFsiBodies);

    ////int haveGhost = (m_data_mgr.countersH->numGhostMarkers > 0) ? 1 : 0;
    ////int haveHelper = (m_data_mgr.countersH->numHelperMarkers > 0) ? 1 : 0;
    int haveRigid = (m_data_mgr.countersH->numFsiBodies > 0) ? 1 : 0;
    int haveFlex1D = (m_data_mgr.countersH->numFsiElements1D > 0) ? 1 : 0;
    int haveFlex2D = (m_data_mgr.countersH->numFsiElements2D > 0) ? 1 : 0;

    // Populate local position of BCE markers - on rigid bodies
    if (haveRigid) {
        Populate_RigidSPH_MeshPos_LRF(fsiBodyBceNum);
        // TODO (Huzaifa): Try to see if this additional function is needed
        UpdateBodyMarkerStateInitial();
    }

    // Populate local position of BCE markers - on flexible bodies
    if (haveFlex1D) {
        m_data_mgr.flex1D_Nodes_D = m_data_mgr.flex1D_Nodes_H;
        m_data_mgr.flex1D_BCEsolids_D = m_data_mgr.flex1D_BCEsolids_H;
        m_data_mgr.flex1D_BCEcoords_D = m_data_mgr.flex1D_BCEcoords_H;
        // TODO (Huzaifa): Try to see if this additional function is needed
        UpdateMeshMarker1DStateInitial();
    }

    if (haveFlex2D) {
        m_data_mgr.flex2D_Nodes_D = m_data_mgr.flex2D_Nodes_H;
        m_data_mgr.flex2D_BCEsolids_D = m_data_mgr.flex2D_BCEsolids_H;
        m_data_mgr.flex2D_BCEcoords_D = m_data_mgr.flex2D_BCEcoords_H;
        // TODO (Huzaifa): Try to see if this additional function is needed
        UpdateMeshMarker2DStateInitial();
    }
}

// -----------------------------------------------------------------------------

void BceManager::Populate_RigidSPH_MeshPos_LRF(std::vector<int> fsiBodyBceNum) {
    // Create map between a BCE on a rigid body and the associated body ID
    uint start_bce = 0;
    thrust::host_vector<uint> h_rigidBodyBlockValidThreads(0);
    // 1 zero is pre added so that in a block with invalid threads, there is no need to map back the invalid threads in
    // the global array. This is only required in the very next block
    thrust::host_vector<uint> h_rigidBodyAccumulatedPaddedThreads(1, 0);
    uint accumulatedPaddedThreads = 0;
    for (int irigid = 0; irigid < fsiBodyBceNum.size(); irigid++) {
        uint end_bce = start_bce + fsiBodyBceNum[irigid];
        thrust::fill(m_data_mgr.rigid_BCEsolids_D.begin() + start_bce, m_data_mgr.rigid_BCEsolids_D.begin() + end_bce,
                     irigid);

        // Calculate block requirements with thread padding to ensure that during rigid body force accumulation
        // each block only handles one rigid body
        // For bodies with > m_rigidBodyBlockSize BCE markers, we need to split the work into multiple blocks and pad
        // the last block with invalid threads
        // For bodies with <= m_rigidBodyBlockSize BCE markers, we only need one block and pad that block with invalid
        // threads
        // Additionally, we accumulate the number of padded thread in each block to ensure we go to the right global
        // index which does not account for thread padding
        uint numBlocks = (fsiBodyBceNum[irigid] + m_rigidBodyBlockSize - 1) / m_rigidBodyBlockSize;
        for (uint blockNum = 0; blockNum < numBlocks; blockNum++) {
            uint numValidThreads = min(m_rigidBodyBlockSize, fsiBodyBceNum[irigid] - blockNum * m_rigidBodyBlockSize);
            h_rigidBodyBlockValidThreads.push_back(numValidThreads);
            uint numPaddedThreadsInThisBlock = m_rigidBodyBlockSize - numValidThreads;
            accumulatedPaddedThreads += numPaddedThreadsInThisBlock;
            h_rigidBodyAccumulatedPaddedThreads.push_back(accumulatedPaddedThreads);
        }
        m_rigidBodyGridSize += numBlocks;
        start_bce = end_bce;
    }
    m_rigidBodyBlockValidThreads = h_rigidBodyBlockValidThreads;
    m_rigidBodyAccumulatedPaddedThreads = h_rigidBodyAccumulatedPaddedThreads;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, nBlocks, nThreads);

    Populate_RigidSPH_MeshPos_LRF_D<<<nBlocks, nThreads>>>(
        mR3CAST(m_data_mgr.rigid_BCEcoords_D), mR4CAST(m_data_mgr.sphMarkers_D->posRadD),
        U1CAST(m_data_mgr.rigid_BCEsolids_D), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
        mR4CAST(m_data_mgr.fsiBodyState_D->rot));

    cudaDeviceSynchronize();
    cudaCheckError();
}

//--------------------------------------------------------------------------------------------------------------------------------
// Calculate accelerations of solid BCE markers -> load m_data_mgr.bceAcc

void BceManager::CalcRigidBceAcceleration() {
    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, numBlocks, numThreads);

    CalcRigidBceAccelerationD<<<numBlocks, numThreads>>>(
        mR3CAST(m_data_mgr.bceAcc), mR4CAST(m_data_mgr.fsiBodyState_D->rot),
        mR3CAST(m_data_mgr.fsiBodyState_D->lin_acc), mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel),
        mR3CAST(m_data_mgr.fsiBodyState_D->ang_acc), mR3CAST(m_data_mgr.rigid_BCEcoords_D),
        U1CAST(m_data_mgr.rigid_BCEsolids_D), U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted));

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::CalcFlex1DBceAcceleration() {
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers1D, 256, nBlocks, nThreads);

    CalcFlex1DBceAcceleration_D<<<nBlocks, nThreads>>>(             //
        mR3CAST(m_data_mgr.bceAcc),                                 //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->acc_fsi_fea_D),        //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                          //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                      //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D),                     //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::CalcFlex2DBceAcceleration() {
    if (m_data_mgr.countersH->numFsiElements2D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers2D, 256, nBlocks, nThreads);

    CalcFlex2DBceAcceleration_D<<<nBlocks, nThreads>>>(             //
        mR3CAST(m_data_mgr.bceAcc),                                 //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->acc_fsi_fea_D),        //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                          //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                      //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D),                     //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

//--------------------------------------------------------------------------------------------------------------------------------
void BceManager::updateBCEAcc() {
    size_t size_ref = m_data_mgr.referenceArray.size();
    int numBceMarkers = m_data_mgr.referenceArray[size_ref - 1].y - m_data_mgr.referenceArray[0].y;
    auto N_solid = m_data_mgr.countersH->numRigidMarkers + m_data_mgr.countersH->numFlexMarkers1D +
                   m_data_mgr.countersH->numFlexMarkers2D;
    auto N_all = N_solid + m_data_mgr.countersH->numBoundaryMarkers;
    if (N_all != numBceMarkers) {
        throw std::runtime_error(
            "Error! Number of rigid, flexible and boundary markers are "
            "saved incorrectly. Thrown from updateBCEAcc!\n");
    }

    // Update portion set to boundary, rigid, and flexible BCE particles
    int4 updatePortion = mI4(m_data_mgr.referenceArray[0].y, m_data_mgr.referenceArray[1].y,
                             m_data_mgr.referenceArray[2].y, m_data_mgr.referenceArray[3].y);

    // Only update boundary BCE particles if no rigid/flexible particles
    if (size_ref == 2) {
        updatePortion.z = m_data_mgr.referenceArray[1].y;
        updatePortion.w = m_data_mgr.referenceArray[1].y;
    }

    // Update boundary and rigid/flexible BCE particles
    if (size_ref == 3)
        updatePortion.w = m_data_mgr.referenceArray[2].y;

    // Calculate accelerations of solid BCE marker
    if (m_data_mgr.countersH->numRigidMarkers > 0) {
        CalcRigidBceAcceleration();
    }
    if (m_data_mgr.countersH->numFlexMarkers1D > 0) {
        CalcFlex1DBceAcceleration();
    }
    if (m_data_mgr.countersH->numFlexMarkers2D > 0) {
        CalcFlex2DBceAcceleration();
    }
}

// -----------------------------------------------------------------------------
void BceManager::Rigid_Forces_Torques() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    thrust::fill(m_data_mgr.rigid_FSI_ForcesD.begin(), m_data_mgr.rigid_FSI_ForcesD.end(), mR3(0));
    thrust::fill(m_data_mgr.rigid_FSI_TorquesD.begin(), m_data_mgr.rigid_FSI_TorquesD.end(), mR3(0));

    // Calculate shared memory size
    size_t sharedMemSize = 2 * m_rigidBodyBlockSize * sizeof(Real3);

    CalcRigidForces_D<<<m_rigidBodyGridSize, m_rigidBodyBlockSize, sharedMemSize>>>(
        mR3CAST(m_data_mgr.rigid_FSI_ForcesD), mR3CAST(m_data_mgr.rigid_FSI_TorquesD),
        U1CAST(m_rigidBodyBlockValidThreads), U1CAST(m_rigidBodyAccumulatedPaddedThreads),
        mR4CAST(m_data_mgr.derivVelRhoD), mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD),
        U1CAST(m_data_mgr.rigid_BCEsolids_D), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted), (uint)m_data_mgr.countersH->numRigidMarkers,
        m_data_mgr.paramsH->markerMass, (uint)m_data_mgr.countersH->startRigidMarkers, m_data_mgr.paramsH->sph_method);

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::Flex1D_Forces() {
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;

    // Initialize accumulator to zero
    thrust::fill(m_data_mgr.flex1D_FSIforces_D.begin(), m_data_mgr.flex1D_FSIforces_D.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers1D, 256, nBlocks, nThreads);
    // Each marker deals with 2 nodes in the 1D case
    size_t sharedMemSize = nThreads * (2 * sizeof(uint) + 2 * sizeof(Real3));

    CalcFlex1DForces_D<<<nBlocks, nThreads, sharedMemSize>>>(        //
        mR3CAST(m_data_mgr.flex1D_FSIforces_D),                      //
        mR4CAST(m_data_mgr.derivVelRhoD),                            //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                           //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                       //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D),                      //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted),  //
        (uint)m_data_mgr.countersH->numFlexMarkers1D,                //
        (uint)m_data_mgr.countersH->startFlexMarkers1D,              //
        m_data_mgr.paramsH->markerMass,                              //
        m_data_mgr.paramsH->sph_method                               //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::Flex2D_Forces() {
    if (m_data_mgr.countersH->numFsiElements2D == 0)
        return;

    // Initialize accumulator to zero
    thrust::fill(m_data_mgr.flex2D_FSIforces_D.begin(), m_data_mgr.flex2D_FSIforces_D.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers2D, 256, nBlocks, nThreads);
    // Each marker deals with 3 nodes in the 2D case
    size_t sharedMemSize = nThreads * (3 * sizeof(uint) + 3 * sizeof(Real3));

    CalcFlex2DForces_D<<<nBlocks, nThreads, sharedMemSize>>>(        //
        mR3CAST(m_data_mgr.flex2D_FSIforces_D),                      //
        mR4CAST(m_data_mgr.derivVelRhoD),                            //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                           //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                       //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D),                      //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted),  //
        (uint)m_data_mgr.countersH->numFlexMarkers2D,                //
        (uint)m_data_mgr.countersH->startFlexMarkers2D,              //
        m_data_mgr.paramsH->markerMass,                              //
        m_data_mgr.paramsH->sph_method                               //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

// -----------------------------------------------------------------------------

void BceManager::UpdateBodyMarkerState() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerState_D<<<nBlocks, nThreads>>>(
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),
        mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D),
        mR3CAST(m_data_mgr.fsiBodyState_D->pos), mR3CAST(m_data_mgr.fsiBodyState_D->lin_vel),
        mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel), mR4CAST(m_data_mgr.fsiBodyState_D->rot),
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted));

    cudaDeviceSynchronize();
    cudaCheckError();
}

// This is applied only during BCE initialization
void BceManager::UpdateBodyMarkerStateInitial() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerStateUnsorted_D<<<nBlocks, nThreads>>>(
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),
        mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D),
        mR3CAST(m_data_mgr.fsiBodyState_D->pos), mR3CAST(m_data_mgr.fsiBodyState_D->lin_vel),
        mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel), mR4CAST(m_data_mgr.fsiBodyState_D->rot));

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::UpdateMeshMarker1DState() {
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers1D, 256, nBlocks, nThreads);

    UpdateMeshMarker1DState_D<<<nBlocks, nThreads>>>(                                                              //
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),        //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos_fsi_fea_D), mR3CAST(m_data_mgr.fsiMesh1DState_D->vel_fsi_fea_D),  //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                                                                         //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                                                                     //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D),                                                                    //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)                                                 //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::UpdateMeshMarker1DStateInitial() {
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers1D, 256, nBlocks, nThreads);

    UpdateMeshMarker1DStateUnsorted_D<<<nBlocks, nThreads>>>(                                                      //
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),                      //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos_fsi_fea_D), mR3CAST(m_data_mgr.fsiMesh1DState_D->vel_fsi_fea_D),  //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                                                                         //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                                                                     //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D)                                                                     //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::UpdateMeshMarker2DState() {
    if (m_data_mgr.countersH->numFsiElements2D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers2D, 256, nBlocks, nThreads);

    UpdateMeshMarker2DState_D<<<nBlocks, nThreads>>>(                                                              //
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),        //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->pos_fsi_fea_D), mR3CAST(m_data_mgr.fsiMesh2DState_D->vel_fsi_fea_D),  //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                                                                         //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                                                                     //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D),                                                                    //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)                                                 //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void BceManager::UpdateMeshMarker2DStateInitial() {
    if (m_data_mgr.countersH->numFsiElements2D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers2D, 256, nBlocks, nThreads);

    UpdateMeshMarker2DStateUnsorted_D<<<nBlocks, nThreads>>>(                                                      //
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),                      //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->pos_fsi_fea_D), mR3CAST(m_data_mgr.fsiMesh2DState_D->vel_fsi_fea_D),  //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                                                                         //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                                                                     //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D)                                                                     //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
