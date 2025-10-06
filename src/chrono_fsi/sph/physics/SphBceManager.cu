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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu, Luning Bakke, Radu Serban
// =============================================================================
//
// Base class for processing boundary condition enforcing (bce) markers forces
// in FSI system.
// =============================================================================

//// TODO: There need to be a better way to compute bce marker forces for different solvers.
//// For explicit solver, it is essentially derivVelRhoD times the marker mass,
//// For I2SPH, it is derivVelRhoD only

#include <type_traits>
#include <fstream>

#include <thrust/iterator/constant_iterator.h>
#include <thrust/reduce.h>
#include <thrust/transform.h>
#include <thrust/copy.h>

#include "chrono_fsi/sph/physics/SphBceManager.cuh"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"

namespace chrono {
namespace fsi {
namespace sph {

SphBceManager::SphBceManager(FsiDataManager& data_mgr, NodeDirections node_directions_mode, bool verbose, bool check_errors)
    : m_data_mgr(data_mgr),
      m_node_directions_mode(node_directions_mode),
      m_verbose(verbose),
      m_check_errors(check_errors) {
    m_totalForceRigid.resize(0);
    m_totalTorqueRigid.resize(0);
    m_rigid_block_size = 512;
    m_rigid_grid_size = 0;
}

SphBceManager::~SphBceManager() {}

// -----------------------------------------------------------------------------

void SphBceManager::Initialize(std::vector<int> fsiBodyBceNum) {
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(ChFsiParamsSPH));
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
        SetForceAccumulationBlocks(fsiBodyBceNum);

        m_data_mgr.rigid_BCEcoords_D = m_data_mgr.rigid_BCEcoords_H;
        m_data_mgr.rigid_BCEsolids_D = m_data_mgr.rigid_BCEsolids_H;
        //// TODO (Huzaifa): Try to see if this additional function is needed
        UpdateBodyMarkerStateInitial();
    }

    // Populate local position of BCE markers - on flexible bodies
    if (haveFlex1D) {
        m_data_mgr.flex1D_Nodes_D = m_data_mgr.flex1D_Nodes_H;
        m_data_mgr.flex1D_BCEsolids_D = m_data_mgr.flex1D_BCEsolids_H;
        m_data_mgr.flex1D_BCEcoords_D = m_data_mgr.flex1D_BCEcoords_H;
        //// TODO (Huzaifa): Try to see if this additional function is needed
        UpdateMeshMarker1DStateInitial();
    }

    if (haveFlex2D) {
        m_data_mgr.flex2D_Nodes_D = m_data_mgr.flex2D_Nodes_H;
        m_data_mgr.flex2D_BCEsolids_D = m_data_mgr.flex2D_BCEsolids_H;
        m_data_mgr.flex2D_BCEcoords_D = m_data_mgr.flex2D_BCEcoords_H;
        //// TODO (Huzaifa): Try to see if this additional function is needed
        UpdateMeshMarker2DStateInitial();
    }
}

// -----------------------------------------------------------------------------

void SphBceManager::SetForceAccumulationBlocks(std::vector<int> fsiBodyBceNum) {
    // 1 zero is pre added so that in a block with invalid threads, there is no need to map back the invalid threads in
    // the global array. This is only required in the very next block
    thrust::host_vector<uint> rigid_valid_threads(0);
    thrust::host_vector<uint> rigid_accumulated_threads(1, 0);

    uint accumulatedPaddedThreads = 0;
    for (int irigid = 0; irigid < fsiBodyBceNum.size(); irigid++) {
        // Calculate block requirements with thread padding to ensure that during rigid body force accumulation each
        // block only handles one rigid body.
        //  - for bodies with > m_rigid_block_size BCE markers, split the work into multiple blocks and pad the last
        //    block with invalid threads.
        //  - for bodies with <= m_rigid_block_size BCE markers, need only one block and pad that block with invalid
        //    threads.
        // Additionally, accumulate the number of padded thread in each block to ensure we go to the right global index
        // which does not account for thread padding.
        uint numBlocks = (fsiBodyBceNum[irigid] + m_rigid_block_size - 1) / m_rigid_block_size;
        for (uint blockNum = 0; blockNum < numBlocks; blockNum++) {
            uint numValidThreads = min(m_rigid_block_size, fsiBodyBceNum[irigid] - blockNum * m_rigid_block_size);
            rigid_valid_threads.push_back(numValidThreads);
            uint numPaddedThreadsInThisBlock = m_rigid_block_size - numValidThreads;
            accumulatedPaddedThreads += numPaddedThreadsInThisBlock;
            rigid_accumulated_threads.push_back(accumulatedPaddedThreads);
        }
        m_rigid_grid_size += numBlocks;
    }

    // Copy vectors to device
    m_rigid_valid_threads = rigid_valid_threads;
    m_rigid_accumulated_threads = rigid_accumulated_threads;
}

// -----------------------------------------------------------------------------
// CalcRigidBceAcceleration
// CalcFlex1DBceAcceleration
// CalcFlex2DBceAcceleration
// -----------------------------------------------------------------------------

__global__ void CalcRigidBceAccelerationD(Real3* accelerations,        // BCE marker accelerations (output)
                                          const Real3* BCE_pos_local,  // BCE body-local coordinates
                                          const uint* body_IDs,        // rigid body ID for each BCE marker
                                          const Real4* body_rot,       // body orientation (relative to global frame)
                                          const Real3* body_angvel,    // body ang. vels. (relative to global frame)
                                          const Real3* body_linacc,    // body lin. acels. (relative to global frame)
                                          const Real3* body_angacc,    // body ang. acels. (relative to global frame)
                                          const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numRigidMarkers)
        return;

    uint marker_index = index + countersD.startRigidMarkers;
    uint sorted_index = mapOriginalToSorted[marker_index];

    int body_ID = body_IDs[index];
    Real3 local = BCE_pos_local[index];

    Real3 u, v, w;
    RotationMatrixFromQuaternion(u, v, w, body_rot[body_ID]);

    // linear acceleration
    Real3 acc = body_linacc[body_ID];

    // centrifugal acceleration
    Real3 omega = body_angvel[body_ID];
    Real3 omega_cross = cross(omega, local);
    Real3 omega_cross_cross = cross(omega, omega_cross);

    acc += mR3(dot(u, omega_cross_cross), dot(v, omega_cross_cross), dot(w, omega_cross_cross));

    // tangential acceleration
    Real3 alpha_cross = cross(body_angacc[body_ID], local);
    acc += mR3(dot(u, alpha_cross), dot(v, alpha_cross), dot(w, alpha_cross));

    accelerations[sorted_index] = acc;
}

__global__ void CalcFlex1DBceAcceleration_D(
    Real3* accelerations,             // [num BCEs on all solids]  BCE marker accelerations (output)
    const Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 1D nodes
    const uint2* flex1D_Nodes_D,      // [num segments]            node indices for each 1D segment
    const uint3* flex1D_BCEsolids_D,  // [num BCEs on 1D segments] association of flex BCEs with a mesh and segment
    const Real3* flex1D_BCEcoords_D,  // [num BCEs on 1D segments] local coordinates of BCE markers on FEA 1-D segments
    const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers1D)
        return;

    uint sorted_index = mapOriginalToSorted[index + countersD.startFlexMarkers1D];

    uint3 flex_solid = flex1D_BCEsolids_D[index];  // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;             // index of segment in associated mesh
    uint flex_seg = flex_solid.z;  // index of segment in global list

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 A0 = acc_fsi_fea_D[seg_nodes.x];       // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[seg_nodes.y];       // (absolute) acceleration of node 1

    Real lambda0 = flex1D_BCEcoords_D[index].x;  // segment coordinate
    Real lambda1 = 1 - lambda0;                  // segment coordinate

    accelerations[sorted_index] = A0 * lambda0 + A1 * lambda1;
}

__global__ void CalcFlex2DBceAcceleration_D(
    Real3* accelerations,             // [num BCEs on all solids]  BCE marker accelerations (output)
    const Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 2D nodes
    const uint3* flex2D_Nodes_D,      // [num triangles]           triangle node indices
    const uint3* flex2D_BCEsolids_D,  // [num BCEs on 1D segments] association of flex BCEs with a mesh and face
    const Real3* flex2D_BCEcoords_D,  // [num BCEs on 1D segments] local coordinates of BCE markers on FEA 2-D faces
    const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers2D)
        return;

    uint sorted_index = mapOriginalToSorted[index + countersD.startFlexMarkers2D];

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

    accelerations[sorted_index] = A0 * lambda0 + A1 * lambda1 + A2 * lambda2;
}

void SphBceManager::CalcRigidBceAcceleration() {
    uint numThreads, numBlocks;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, numBlocks, numThreads);

    CalcRigidBceAccelerationD<<<numBlocks, numThreads>>>(                                          //
        mR3CAST(m_data_mgr.bceAcc),                                                                //
        mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D),               //
        mR4CAST(m_data_mgr.fsiBodyState_D->rot), mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel),      //
        mR3CAST(m_data_mgr.fsiBodyState_D->lin_acc), mR3CAST(m_data_mgr.fsiBodyState_D->ang_acc),  //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)                                 //
    );
    if (m_check_errors) {
        cudaCheckError();
    }
}

void SphBceManager::CalcFlex1DBceAcceleration() {
    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers1D, 256, nBlocks, nThreads);

    CalcFlex1DBceAcceleration_D<<<nBlocks, nThreads>>>(             //
        mR3CAST(m_data_mgr.bceAcc),                                 //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->acc),                  //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                          //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                      //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D),                     //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)  //
    );
    if (m_check_errors) {
        cudaCheckError();
    }
}

void SphBceManager::CalcFlex2DBceAcceleration() {
    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers2D, 256, nBlocks, nThreads);

    CalcFlex2DBceAcceleration_D<<<nBlocks, nThreads>>>(             //
        mR3CAST(m_data_mgr.bceAcc),                                 //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->acc),                  //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                          //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                      //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D),                     //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)  //
    );
    if (m_check_errors) {
        cudaCheckError();
    }
}

// -----------------------------------------------------------------------------
// Calculate accelerations of solid BCE markers -> load m_data_mgr.bceAcc
// -----------------------------------------------------------------------------

void SphBceManager::updateBCEAcc() {
    if (m_data_mgr.countersH->numRigidMarkers > 0)
        CalcRigidBceAcceleration();

    if (m_data_mgr.countersH->numFlexMarkers1D > 0)
        CalcFlex1DBceAcceleration();

    if (m_data_mgr.countersH->numFlexMarkers2D > 0)
        CalcFlex2DBceAcceleration();
}

// -----------------------------------------------------------------------------
// Rigid_Forces_Torques
// Flex1D_Forces
// Flex2D_Forces
// -----------------------------------------------------------------------------

__global__ void CalcRigidForces_D(Real3* __restrict__ body_forces,
                                  Real3* __restrict__ body_torques,
                                  const uint* __restrict__ rigidBodyBlockValidThreads,
                                  const uint* __restrict__ rigidBodyAccumulatedPaddedThreads,
                                  const Real4* __restrict__ derivatives,
                                  const Real4* __restrict__ positions,
                                  const uint* __restrict__ body_IDs,
                                  const Real3* __restrict__ body_pos,
                                  const uint* __restrict__ mapOriginalToSorted,
                                  const uint numRigidMarkers,
                                  const Real markerMass,
                                  const uint startRigidMarkers) {
    extern __shared__ char sharedMem[];
    const uint blockSize = blockDim.x;

    // Shared memory allocations
    Real3* sharedForces = (Real3*)sharedMem;                  // Size: blockSize
    Real3* sharedTorques = (Real3*)&sharedForces[blockSize];  // Size: blockSize

    uint threadIdx_x = threadIdx.x;
    uint global_index = blockIdx.x * blockDim.x + threadIdx_x;

    // Valid threads in current block
    uint validThreads = rigidBodyBlockValidThreads[blockIdx.x];
    // Valid threads in previous block
    uint paddedThreads = rigidBodyAccumulatedPaddedThreads[blockIdx.x];

    uint global_index_padded = global_index - paddedThreads;

    if (global_index_padded >= numRigidMarkers)
        return;

    uint marker_index = global_index_padded + startRigidMarkers;
    uint sorted_index = mapOriginalToSorted[marker_index];

    // Get body ID for the current marker
    uint body_ID = body_IDs[global_index_padded];

    Real3 Force = make_Real3(0.0f, 0.0f, 0.0f);
    Real3 Torque = make_Real3(0.0f, 0.0f, 0.0f);
    if (threadIdx_x < validThreads) {
        if (paramsD.integration_scheme == IntegrationScheme::IMPLICIT_SPH)
            Force = mR3(derivatives[sorted_index]);
        else
            Force = mR3(derivatives[sorted_index]) * markerMass;

        Real3 dist3 = mR3(positions[sorted_index]) - body_pos[body_ID];
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
        atomicAdd(&body_forces[body_ID].x, sharedForces[0].x);
        atomicAdd(&body_forces[body_ID].y, sharedForces[0].y);
        atomicAdd(&body_forces[body_ID].z, sharedForces[0].z);

        atomicAdd(&body_torques[body_ID].x, sharedTorques[0].x);
        atomicAdd(&body_torques[body_ID].y, sharedTorques[0].y);
        atomicAdd(&body_torques[body_ID].z, sharedTorques[0].z);
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
                                   const Real markerMass) {
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
    if (paramsD.integration_scheme == IntegrationScheme::IMPLICIT_SPH)
        Force = mR3(derivVelRhoD[sortedIndex]);
    else
        Force = mR3(derivVelRhoD[sortedIndex]) * paramsD.markerMass;
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
                                   const Real markerMass) {
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
    if (paramsD.integration_scheme == IntegrationScheme::IMPLICIT_SPH)
        Force = mR3(derivVelRhoD[sortedIndex]);
    else
        Force = mR3(derivVelRhoD[sortedIndex]) * markerMass;

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

void SphBceManager::Rigid_Forces_Torques() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    thrust::fill(m_data_mgr.rigid_FSI_ForcesD.begin(), m_data_mgr.rigid_FSI_ForcesD.end(), mR3(0));
    thrust::fill(m_data_mgr.rigid_FSI_TorquesD.begin(), m_data_mgr.rigid_FSI_TorquesD.end(), mR3(0));

    // Calculate shared memory size
    size_t sharedMemSize = 2 * m_rigid_block_size * sizeof(Real3);

    CalcRigidForces_D<<<m_rigid_grid_size, m_rigid_block_size, sharedMemSize>>>(
        mR3CAST(m_data_mgr.rigid_FSI_ForcesD), mR3CAST(m_data_mgr.rigid_FSI_TorquesD),
        U1CAST(m_rigid_valid_threads), U1CAST(m_rigid_accumulated_threads),
        mR4CAST(m_data_mgr.derivVelRhoD), mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD),
        U1CAST(m_data_mgr.rigid_BCEsolids_D), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted), (uint)m_data_mgr.countersH->numRigidMarkers,
        m_data_mgr.paramsH->markerMass, (uint)m_data_mgr.countersH->startRigidMarkers);

    if (m_check_errors) {
        cudaCheckError();
    }
}

void SphBceManager::Flex1D_Forces() {
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
        m_data_mgr.paramsH->markerMass);

    if (m_check_errors) {
        cudaCheckError();
    }
}

void SphBceManager::Flex2D_Forces() {
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
        m_data_mgr.paramsH->markerMass);

    if (m_check_errors) {
        cudaCheckError();
    }
}

// -----------------------------------------------------------------------------
// UpdateBodyMarkerState
// UpdateBodyMarkerStateInitial
// -----------------------------------------------------------------------------

__global__ void UpdateBodyMarkerState_D(Real4* positions,            // global marker positions (sorted)
                                        Real3* velocities,           // global marker velocities (sorted)
                                        const Real3* BCE_pos_local,  // BCE body-local coordinates
                                        const uint* body_IDs,        // rigid body ID for each BCE marker
                                        const Real3* body_pos,       // body positions (relative to global frame)
                                        const Real4* body_rot,       // body orientation (relative to global frame)
                                        const Real3* body_linvel,    // body lin. vels. (relative to global frame)
                                        const Real3* body_angvel,    // body ang. vels. (relative to global frame)
                                        const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numRigidMarkers)
        return;

    uint marker_index = index + countersD.startRigidMarkers;
    uint sorted_index = mapOriginalToSorted[marker_index];

    int body_ID = body_IDs[index];
    Real3 local = BCE_pos_local[index];

    Real3 u, v, w;
    RotationMatrixFromQuaternion(u, v, w, body_rot[body_ID]);

    // BCE marker position
    Real h = positions[sorted_index].w;
    Real3 pos = body_pos[body_ID] + mR3(dot(u, local), dot(v, local), dot(w, local));
    positions[sorted_index] = mR4(pos, h);

    // BCE marker velocity
    Real3 omega_cross = cross(body_angvel[body_ID], local);
    velocities[sorted_index] =
        body_linvel[body_ID] + mR3(dot(u, omega_cross), dot(v, omega_cross), dot(w, omega_cross));
}

__global__ void UpdateBodyMarkerStateUnsorted_D(
    Real4* positions,            // global marker positions (original)
    Real3* velocities,           // global marker velocities (original)
    const Real3* BCE_pos_local,  // BCE body-local coordinates
    const uint* body_IDs,        // rigid body ID for each BCE marker
    const Real3* body_pos,       // body positions (relative to global frame)
    const Real4* body_rot,       // body orientation (relative to global frame)
    const Real3* body_linvel,    // body lin. vels. (relative to global frame)
    const Real3* body_angvel) {  // body ang. vels. (relative to global frame)
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numRigidMarkers)
        return;

    uint marker_index = index + countersD.startRigidMarkers;

    int body_ID = body_IDs[index];
    Real3 local = BCE_pos_local[index];

    Real3 u, v, w;
    RotationMatrixFromQuaternion(u, v, w, body_rot[body_ID]);

    // BCE marker position
    Real h = positions[marker_index].w;
    Real3 pos = body_pos[body_ID] + mR3(dot(u, local), dot(v, local), dot(w, local));
    positions[marker_index] = mR4(pos, h);

    // BCE marker velocity
    Real3 omega_cross = cross(body_angvel[body_ID], local);
    velocities[marker_index] =
        body_linvel[body_ID] + mR3(dot(u, omega_cross), dot(v, omega_cross), dot(w, omega_cross));
}

void SphBceManager::UpdateBodyMarkerState() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerState_D<<<nBlocks, nThreads>>>(
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),
        mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D),
        mR3CAST(m_data_mgr.fsiBodyState_D->pos), mR4CAST(m_data_mgr.fsiBodyState_D->rot),
        mR3CAST(m_data_mgr.fsiBodyState_D->lin_vel), mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel),
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted));

    if (m_check_errors) {
        cudaCheckError();
    }
}

void SphBceManager::UpdateBodyMarkerStateInitial() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerStateUnsorted_D<<<nBlocks, nThreads>>>(
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),
        mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D),
        mR3CAST(m_data_mgr.fsiBodyState_D->pos), mR4CAST(m_data_mgr.fsiBodyState_D->rot),
        mR3CAST(m_data_mgr.fsiBodyState_D->lin_vel), mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel));

    if (m_check_errors) {
        cudaCheckError();
    }
}

// -----------------------------------------------------------------------------
// Options for nodal direction calculation
// NODAL_DIR_METHOD = 1:  average over adjacent elements
// NODAL_DIR_METHOD = 2:  normalized sum over adjacent elements
// -----------------------------------------------------------------------------

#define NODAL_DIR_METHOD 1

// -----------------------------------------------------------------------------
// CalcNodeDir1D
// UpdateMeshMarker1DState
// UpdateMeshMarker1DStateInitial
// -----------------------------------------------------------------------------

struct printInt2 {
    __host__ __device__ void operator()(const int2& v) { printf("%d %d   ", v.x, v.y); }
};

struct normalizeReal3 {
    __host__ __device__ Real3 operator()(const Real3& v) { return get_normalized(v); }
};

struct averageDirs {
    __host__ __device__ Real3 operator()(const Real3& v, uint count) { return v / Real(count); }
};

__global__ void CalcNodeDir1D_D(uint* ext_nodes,             // extended node indices (2 per segment)
                                Real3* ext_dirs,             // extended node directions (2 per segment)
                                const Real3* pos,            // positions of FEA 1-D segment nodes
                                const uint2* flex1D_Nodes_D  // segment node indices
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFsiNodes1D)
        return;

    uint2 seg_nodes = flex1D_Nodes_D[index];  // indices of the 2 nodes on associated segment
    Real3 P0 = pos[seg_nodes.x];              // (absolute) position of node 0
    Real3 P1 = pos[seg_nodes.y];              // (absolute) position of node 1

    Real3 d = P1 - P0;
#if NODAL_DIR_METHOD == 2
    normalize(d);
#endif

    ext_nodes[2 * index + 0] = seg_nodes.x;
    ext_nodes[2 * index + 1] = seg_nodes.y;
    ext_dirs[2 * index + 0] = d;
    ext_dirs[2 * index + 1] = d;
}

void SphBceManager::CalcNodeDirections1D(thrust::device_vector<Real3>& dirs) {
    uint num_nodes = (uint)m_data_mgr.countersH->numFsiNodes1D;

    uint nBlocks, nThreads;
    computeGridSize(num_nodes, 256, nBlocks, nThreads);

    // Load segment directions to each adjacent node
    thrust::device_vector<uint> ext_nodes(2 * num_nodes);
    thrust::device_vector<Real3> ext_dirs(2 * num_nodes);
    CalcNodeDir1D_D<<<nBlocks, nThreads>>>(U1CAST(ext_nodes), mR3CAST(ext_dirs),
                                           mR3CAST(m_data_mgr.fsiMesh1DState_D->pos),
                                           U2CAST(m_data_mgr.flex1D_Nodes_D));

    ////thrust::for_each(m_data_mgr.flex1D_Nodes_D.begin(), m_data_mgr.flex1D_Nodes_D.end(), printInt2());
    ////std::cout << std::endl;

    //// TODO RADU - must sort by key before reductions

    // Sum directions from adjacent segments
    {
        thrust::device_vector<uint> out_nodes(2 * num_nodes);
        thrust::reduce_by_key(ext_nodes.begin(), ext_nodes.end(), ext_dirs.begin(), out_nodes.begin(), dirs.begin(),
                              thrust::equal_to<uint>());
    }

#if NODAL_DIR_METHOD == 1
    // Average directions
    {
        thrust::device_vector<uint> out_nodes(2 * num_nodes);
        thrust::device_vector<uint> counts(2 * num_nodes);
        thrust::reduce_by_key(ext_nodes.begin(), ext_nodes.end(), thrust::make_constant_iterator(1), out_nodes.begin(),
                              counts.begin());

        ////thrust::copy(counts.begin(), counts.end(), std::ostream_iterator<uint>(std::cout, " "));
        ////std::cout << std::endl;

        thrust::transform(dirs.begin(), dirs.end(), counts.begin(), dirs.begin(), averageDirs());
    }
#elif NODAL_DIR_METHOD == 2
    // Normalize nodal directions
    thrust::transform(dirs.begin(), dirs.end(), dirs.begin(), normalizeReal3());
#endif

    ////thrust::copy(dirs.begin(), dirs.end(), std::ostream_iterator<Real3>(std::cout, " | "));
    ////std::cout << std::endl;
}

__global__ void UpdateMeshMarker1DState_D(
    Real4* posRadD,                   // marker positions (output)
    Real3* velMasD,                   // marker velocities (output)
    const Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
    const Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
    bool use_dirs,                    // use nodal directions
    const Real3* dirs,                // nodal directions
    const uint2* flex1D_Nodes_D,      // segment node indices
    const uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    const Real3* flex1D_BCEcoords_D,  // local coordinates of BCE markers on FEA 1-D segments
    const uint* mapOriginalToSorted   //
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers1D)
        return;

    uint flex_index = index + countersD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint3 flex_solid = flex1D_BCEsolids_D[index];            // associated flex mesh and segment
    uint flex_seg = flex_solid.z;                            // index of segment in global list
    ////uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 P0 = pos_fsi_fea_D[seg_nodes.x];       // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[seg_nodes.y];       // (absolute) position of node 1
    Real3 V0 = vel_fsi_fea_D[seg_nodes.x];       // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[seg_nodes.y];       // (absolute) velocity of node 1

    Real t = flex1D_BCEcoords_D[index].x;      // segment coordinate
    Real y_val = flex1D_BCEcoords_D[index].y;  // off-segment y coordinate
    Real z_val = flex1D_BCEcoords_D[index].z;  // off-segment z coordinate

    Real3 P;  // position along segment center-line
    Real3 D;  // normal along segment center-line
    if (use_dirs) {
        auto t2 = t * t;
        auto t3 = t2 * t;

        auto a0 = 2 * t3 - 3 * t2 + 1;
        auto a1 = -2 * t3 + 3 * t2;
        auto b0 = t3 - 2 * t2 + t;
        auto b1 = t3 - t2;
        P = P0 * a0 + P1 * a1 + dirs[seg_nodes.x] * b0 + dirs[seg_nodes.y] * b1;

        auto a0d = 6 * t2 - 6 * t;
        auto a1d = -6 * t2 + 6 * t;
        auto b0d = 3 * t2 - 4 * t + 1;
        auto b1d = 3 * t2 - 2 * t;
        D = P0 * a0d + P1 * a1d + dirs[seg_nodes.x] * b0d + dirs[seg_nodes.y] * b1d;
    } else {
        P = P0 * (1 - t) + P1 * t;
        D = P1 - P0;
    }

    // Create local frame
    Real3 x_dir = get_normalized(D);
    Real3 y_dir;
    Real3 z_dir;
    get_orthogonal_axes(x_dir, y_dir, z_dir);

    P += y_val * y_dir + z_val * z_dir;  // BCE marker position
    Real3 V = V0 * (1 - t) + V1 * t;     // BCE marker velocity

    uint sorted_flex_index = mapOriginalToSorted[flex_index];
    Real h = posRadD[sorted_flex_index].w;
    posRadD[sorted_flex_index] = mR4(P, h);
    velMasD[sorted_flex_index] = V;
}

__global__ void UpdateMeshMarker1DStateUnsorted_D(
    Real4* posRadD,                   // marker positions (output)
    Real3* velMasD,                   // marker velocities (output)
    const Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
    const Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
    bool use_dirs,                    // use nodal directions
    const Real3* dirs,                // nodal directions
    const uint2* flex1D_Nodes_D,      // segment node indices
    const uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    const Real3* flex1D_BCEcoords_D   // local coordinates of BCE markers on FEA 1-D segments
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers1D)
        return;

    uint flex_index = index + countersD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint3 flex_solid = flex1D_BCEsolids_D[index];            // associated flex mesh and segment
    uint flex_seg = flex_solid.z;                            // index of segment in global list
    ////uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 P0 = pos_fsi_fea_D[seg_nodes.x];       // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[seg_nodes.y];       // (absolute) position of node 1
    Real3 V0 = vel_fsi_fea_D[seg_nodes.x];       // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[seg_nodes.y];       // (absolute) velocity of node 1

    Real t = flex1D_BCEcoords_D[index].x;      // segment coordinate
    Real y_val = flex1D_BCEcoords_D[index].y;  // off-segment y coordinate
    Real z_val = flex1D_BCEcoords_D[index].z;  // off-segment z coordinate

    Real3 P;  // position along segment center-line
    Real3 D;  // normal along segment center-line
    if (use_dirs) {
        auto t2 = t * t;
        auto t3 = t2 * t;

        auto a0 = 2 * t3 - 3 * t2 + 1;
        auto a1 = -2 * t3 + 3 * t2;
        auto b0 = t3 - 2 * t2 + t;
        auto b1 = t3 - t2;
        P = P0 * a0 + P1 * a1 + dirs[seg_nodes.x] * b0 + dirs[seg_nodes.y] * b1;

        auto a0d = 6 * t2 - 6 * t;
        auto a1d = -6 * t2 + 6 * t;
        auto b0d = 3 * t2 - 4 * t + 1;
        auto b1d = 3 * t2 - 2 * t;
        D = P0 * a0d + P1 * a1d + dirs[seg_nodes.x] * b0d + dirs[seg_nodes.y] * b1d;
    } else {
        P = P0 * (1 - t) + P1 * t;
        D = P1 - P0;
    }

    // Create local frame
    Real3 x_dir = get_normalized(D);
    Real3 y_dir;
    Real3 z_dir;
    get_orthogonal_axes(x_dir, y_dir, z_dir);

    P += y_val * y_dir + z_val * z_dir;  // BCE marker position
    Real3 V = V0 * (1 - t) + V1 * t;     // BCE marker velocity

    Real h = posRadD[flex_index].w;
    posRadD[flex_index] = mR4(P, h);
    velMasD[flex_index] = V;
}

void SphBceManager::UpdateMeshMarker1DState() {
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;

    // If needed, calculate current node directions as averages
    if (m_node_directions_mode == NodeDirections::AVERAGE) {
        CalcNodeDirections1D(m_data_mgr.fsiMesh1DState_D->dir);
    }

    bool use_node_directions = (m_node_directions_mode != NodeDirections::NONE);

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers1D, 256, nBlocks, nThreads);

    UpdateMeshMarker1DState_D<<<nBlocks, nThreads>>>(                                                        //
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),  //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), mR3CAST(m_data_mgr.fsiMesh1DState_D->vel),                //
        use_node_directions, mR3CAST(m_data_mgr.fsiMesh1DState_D->dir),                                      //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                                                                   //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                                                               //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D),                                                              //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)                                           //
    );

    if (m_check_errors) {
        cudaCheckError();
    }
}

void SphBceManager::UpdateMeshMarker1DStateInitial() {
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;

    // If needed, calculate current node directions as averages
    if (m_node_directions_mode == NodeDirections::AVERAGE) {
        CalcNodeDirections1D(m_data_mgr.fsiMesh1DState_D->dir);
    }

    bool use_node_directions = (m_node_directions_mode != NodeDirections::NONE);

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers1D, 256, nBlocks, nThreads);

    UpdateMeshMarker1DStateUnsorted_D<<<nBlocks, nThreads>>>(                                  //
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),  //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), mR3CAST(m_data_mgr.fsiMesh1DState_D->vel),  //
        use_node_directions, mR3CAST(m_data_mgr.fsiMesh1DState_D->dir),                        //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                                                     //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                                                 //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D)                                                 //
    );

    if (m_check_errors) {
        cudaCheckError();
    }
}

// -----------------------------------------------------------------------------
// UpdateMeshMarker2DState
// UpdateMeshMarker2DStateInitial
// -----------------------------------------------------------------------------

__global__ void CalcNodeDir2D_D(uint* ext_nodes,             // extended node indices (3 per triangle)
                                Real3* ext_dirs,             // extended node directions (3 per triangle)
                                const Real3* pos,            // positions of FEA 2-D triangle nodes
                                const uint3* flex2D_Nodes_D  // segment node indices
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFsiNodes1D)
        return;

    uint3 tri_nodes = flex2D_Nodes_D[index];  // indices of the 3 nodes on associated triangle
    Real3 P0 = pos[tri_nodes.x];              // (absolute) position of node 0
    Real3 P1 = pos[tri_nodes.y];              // (absolute) position of node 1
    Real3 P2 = pos[tri_nodes.z];              // (absolute) position of node 2

    Real3 d = cross(P1 - P0, P2 - P0);
#if NODAL_DIR_METHOD == 2
    normalize(d);
#endif

    ext_nodes[3 * index + 0] = tri_nodes.x;
    ext_nodes[3 * index + 1] = tri_nodes.y;
    ext_nodes[3 * index + 2] = tri_nodes.y;
    ext_dirs[3 * index + 0] = d;
    ext_dirs[3 * index + 1] = d;
    ext_dirs[3 * index + 2] = d;
}

void SphBceManager::CalcNodeDirections2D(thrust::device_vector<Real3>& dirs) {
    uint num_nodes = (uint)m_data_mgr.countersH->numFsiNodes2D;

    uint nBlocks, nThreads;
    computeGridSize(num_nodes, 256, nBlocks, nThreads);

    // Load triangle normal directions to each each adjacent node
    thrust::device_vector<uint> ext_nodes(3 * num_nodes);
    thrust::device_vector<Real3> ext_dirs(3 * num_nodes);
    CalcNodeDir2D_D<<<nBlocks, nThreads>>>(U1CAST(ext_nodes), mR3CAST(ext_dirs),
                                           mR3CAST(m_data_mgr.fsiMesh2DState_D->pos),
                                           U3CAST(m_data_mgr.flex2D_Nodes_D));

    //// TODO RADU - must sort by key before reductions

    // Sum directions from adjacent triangles
    {
        thrust::device_vector<uint> out_nodes(3 * num_nodes);
        thrust::reduce_by_key(ext_nodes.begin(), ext_nodes.end(), ext_dirs.begin(), out_nodes.begin(), dirs.begin(),
                              thrust::equal_to<uint>());
    }

#if NODAL_DIR_METHOD == 1
    // Average directions
    {
        thrust::device_vector<uint> out_nodes(3 * num_nodes);
        thrust::device_vector<uint> counts(3 * num_nodes);
        thrust::reduce_by_key(ext_nodes.begin(), ext_nodes.end(), thrust::make_constant_iterator(1), out_nodes.begin(),
                              counts.begin());

        ////thrust::copy(counts.begin(), counts.end(), std::ostream_iterator<uint>(std::cout, " "));
        ////std::cout << std::endl;

        thrust::transform(dirs.begin(), dirs.end(), counts.begin(), dirs.begin(), averageDirs());

        //// TODO
    }
#elif NODAL_DIR_METHOD == 2
    // Normalize nodal directions
    thrust::transform(dirs.begin(), dirs.end(), dirs.begin(), normalizeReal3());
#endif

    ////thrust::copy(dirs.begin(), dirs.end(), std::ostream_iterator<Real3>(std::cout, " | "));
    ////std::cout << std::endl;
}

//// TODO RADU - implement utility device function to interpolate position on triangle (linear or cubic)

__global__ void UpdateMeshMarker2DState_D(
    Real4* posRadD,                   // marker positions (output)
    Real3* velMasD,                   // marker velocities (output)
    const Real3* pos_fsi_fea_D,       // positions of FEA 2-D face nodes
    const Real3* vel_fsi_fea_D,       // velocities of FEA 2-D face nodes
    bool use_dirs,                    // use nodal directions
    const Real3* dirs,                // nodal directions
    const uint3* flex2D_Nodes_D,      // triangle node indices
    const uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
    const Real3* flex2D_BCEcoords_D,  // local coordinates of BCE markers on FEA 2-D faces
    const uint* mapOriginalToSorted   //
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers2D)
        return;

    uint flex_index = index + countersD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint3 flex_solid = flex2D_BCEsolids_D[index];            // associated flex mesh and face
    uint flex_tri = flex_solid.z;                            // index of triangle in global list
    ////uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    Real3 P0 = pos_fsi_fea_D[tri_nodes.x];      // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[tri_nodes.y];      // (absolute) position of node 1
    Real3 P2 = pos_fsi_fea_D[tri_nodes.z];      // (absolute) position of node 2
    Real3 V0 = vel_fsi_fea_D[tri_nodes.x];      // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[tri_nodes.y];      // (absolute) velocity of node 1
    Real3 V2 = vel_fsi_fea_D[tri_nodes.z];      // (absolute) velocity of node 2

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate
    Real z_val = flex2D_BCEcoords_D[index].z;    // off-face coordinate

    //// TODO RADU - calculate interpolated normal and P based on 'use_dirs'

    Real3 P = P0 * lambda0 + P1 * lambda1 + P2 * lambda2;  // centerline position
    Real3 D = cross(P1 - P0, P2 - P1);                     // local normal

    P += z_val * get_normalized(D);                        // BCE marker position
    Real3 V = V0 * lambda0 + V1 * lambda1 + V2 * lambda2;  // BCE marker velocity

    uint sorted_flex_index = mapOriginalToSorted[flex_index];
    Real h = posRadD[sorted_flex_index].w;
    posRadD[sorted_flex_index] = mR4(P, h);
    velMasD[sorted_flex_index] = V;
}

__global__ void UpdateMeshMarker2DStateUnsorted_D(
    Real4* posRadD,                   // marker positions (output)
    Real3* velMasD,                   // marker velocities (output)
    const Real3* pos_fsi_fea_D,       // positions of FEA 2-D face nodes
    const Real3* vel_fsi_fea_D,       // velocities of FEA 2-D face nodes
    bool use_dirs,                    // use nodal directions
    const Real3* dirs,                // nodal directions
    const uint3* flex2D_Nodes_D,      // triangle node indices
    const uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
    const Real3* flex2D_BCEcoords_D   // local coordinates of BCE markers on FEA 2-D faces
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFlexMarkers2D)
        return;

    uint flex_index = index + countersD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint3 flex_solid = flex2D_BCEsolids_D[index];            // associated flex mesh and face
    uint flex_tri = flex_solid.z;                            // index of triangle in global list
    ////uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    Real3 P0 = pos_fsi_fea_D[tri_nodes.x];      // (absolute) position of node 0
    Real3 P1 = pos_fsi_fea_D[tri_nodes.y];      // (absolute) position of node 1
    Real3 P2 = pos_fsi_fea_D[tri_nodes.z];      // (absolute) position of node 2
    Real3 V0 = vel_fsi_fea_D[tri_nodes.x];      // (absolute) velocity of node 0
    Real3 V1 = vel_fsi_fea_D[tri_nodes.y];      // (absolute) velocity of node 1
    Real3 V2 = vel_fsi_fea_D[tri_nodes.z];      // (absolute) velocity of node 2

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate
    Real z_val = flex2D_BCEcoords_D[index].z;    // off-face coordinate

    //// TODO RADU - calculate interpolated normal and P based on 'use_dirs'

    Real3 P = P0 * lambda0 + P1 * lambda1 + P2 * lambda2;  // centerline position
    Real3 D = cross(P1 - P0, P2 - P1);                     // local normal

    P += z_val * get_normalized(D);                        // BCE marker position
    Real3 V = V0 * lambda0 + V1 * lambda1 + V2 * lambda2;  // BCE marker velocity

    Real h = posRadD[flex_index].w;
    posRadD[flex_index] = mR4(P, h);
    velMasD[flex_index] = V;
}

void SphBceManager::UpdateMeshMarker2DState() {
    if (m_data_mgr.countersH->numFsiElements2D == 0)
        return;

    // If needed, calculate current node directions as averages
    if (m_node_directions_mode == NodeDirections::AVERAGE) {
        CalcNodeDirections2D(m_data_mgr.fsiMesh2DState_D->dir);
    }

    bool use_node_directions = (m_node_directions_mode != NodeDirections::NONE);

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers2D, 256, nBlocks, nThreads);

    UpdateMeshMarker2DState_D<<<nBlocks, nThreads>>>(                                                        //
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),  //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->pos), mR3CAST(m_data_mgr.fsiMesh2DState_D->vel),                //
        use_node_directions, mR3CAST(m_data_mgr.fsiMesh2DState_D->dir),                                      //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                                                                   //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                                                               //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D),                                                              //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)                                           //
    );

    if (m_check_errors) {
        cudaCheckError();
    }
}

void SphBceManager::UpdateMeshMarker2DStateInitial() {
    if (m_data_mgr.countersH->numFsiElements2D == 0)
        return;

    // If needed, calculate current node directions as averages
    if (m_node_directions_mode == NodeDirections::AVERAGE) {
        CalcNodeDirections2D(m_data_mgr.fsiMesh2DState_D->dir);
    }

    bool use_node_directions = (m_node_directions_mode != NodeDirections::NONE);

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFlexMarkers2D, 256, nBlocks, nThreads);

    UpdateMeshMarker2DStateUnsorted_D<<<nBlocks, nThreads>>>(                                  //
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),  //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->pos), mR3CAST(m_data_mgr.fsiMesh2DState_D->vel),  //
        use_node_directions, mR3CAST(m_data_mgr.fsiMesh2DState_D->dir),                        //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                                                     //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                                                 //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D)                                                 //
    );

    if (m_check_errors) {
        cudaCheckError();
    }
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
