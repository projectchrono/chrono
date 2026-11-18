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
#include <limits>
#include <stdexcept>

#include <thrust/iterator/constant_iterator.h>
#include <thrust/reduce.h>
#include <thrust/transform.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/scatter.h>
#include <thrust/sort.h>
#include <thrust/iterator/discard_iterator.h>

#include "chrono_fsi/sph/physics/SphBceManager.cuh"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"

namespace chrono {
namespace fsi {
namespace sph {

// Reduction operators.
// Defined locally rather than using thrust::minimum / thrust::plus /
// thrust::equal_to: those are deprecated in CCCL 3.1 (CUDA 13.x) in favor of
// cuda::minimum / cuda::std::plus / cuda::std::equal_to, which are libcu++ and
// therefore unavailable under rocThrust (the Thrust device system used on AMD).

struct same_key_uint {
    __host__ __device__ bool operator()(const uint& a, const uint& b) const { return a == b; }
};

void CopyParametersToDevice_SphBceManager(std::shared_ptr<ChFsiParamsSPH> paramsH, std::shared_ptr<Counters> countersH) {
    gpuMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(ChFsiParamsSPH));
    gpuCheckError();
    gpuMemcpyToSymbolAsync(countersD, countersH.get(), sizeof(Counters));
    gpuCheckError();
}

SphBceManager::SphBceManager(FsiDataManager& data_mgr, NodeDirections node_directions_mode, bool verbose, bool check_errors)
    : m_data_mgr(data_mgr), m_node_directions_mode(node_directions_mode), m_verbose(verbose), m_check_errors(check_errors) {
    m_rigid_block_size = 512;
    m_rigid_grid_size = 0;
}

SphBceManager::~SphBceManager() {}

// -----------------------------------------------------------------------------

template <class Connectivity>
void SetMeshForceMap(size_t num_nodes,
                     const thrust::host_vector<Connectivity>& elements,
                     const thrust::host_vector<uint3>& solids,
                     thrust::device_vector<uint>& node_offsets,
                     thrust::device_vector<uint>& contributions) {
    constexpr uint nodes_per_marker = std::is_same_v<Connectivity, int2> ? 2 : 3;
    if (num_nodes >= std::numeric_limits<uint>::max() || solids.size() > std::numeric_limits<uint>::max() / nodes_per_marker)
        throw std::overflow_error("FSI mesh force map exceeds the supported index range");

    thrust::host_vector<uint> offsets(num_nodes + 1, 0);
    thrust::host_vector<uint> nodes(solids.size() * nodes_per_marker);
    for (size_t marker = 0; marker < solids.size(); marker++) {
        auto element = elements[solids[marker].z];
        nodes[nodes_per_marker * marker] = element.x;
        nodes[nodes_per_marker * marker + 1] = element.y;
        if constexpr (nodes_per_marker == 3)
            nodes[nodes_per_marker * marker + 2] = element.z;
    }
    for (auto node : nodes)
        offsets[node + 1]++;
    for (size_t node = 0; node < num_nodes; node++)
        offsets[node + 1] += offsets[node];

    // Build once in original marker order; shared nodes need not be adjacent in the element list.
    auto cursor = offsets;
    thrust::host_vector<uint> ordered(nodes.size());
    for (size_t i = 0; i < nodes.size(); i++)
        ordered[cursor[nodes[i]]++] = (uint)i;
    node_offsets = offsets;
    contributions = ordered;
}

void SphBceManager::Initialize(std::vector<int> fsiBodyBceNum) {
    gpuMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(ChFsiParamsSPH));
    gpuMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));

    ////int haveGhost = (m_data_mgr.countersH->numGhostMarkers > 0) ? 1 : 0;
    ////int haveHelper = (m_data_mgr.countersH->numHelperMarkers > 0) ? 1 : 0;
    int haveRigid = (m_data_mgr.countersH->numFsiBodies > 0) ? 1 : 0;
    int haveMesh1D = (m_data_mgr.countersH->numFsiElements1D > 0) ? 1 : 0;
    int haveMesh2D = (m_data_mgr.countersH->numFsiElements2D > 0) ? 1 : 0;

    // Populate local position of BCE markers - on rigid bodies
    if (haveRigid) {
        SetForceAccumulationBlocks(fsiBodyBceNum);

        m_data_mgr.rigid_BCEcoords_D = m_data_mgr.rigid_BCEcoords_H;
        m_data_mgr.rigid_BCEsolids_D = m_data_mgr.rigid_BCEsolids_H;
        //// TODO (Huzaifa): Try to see if this additional function is needed
        UpdateBodyMarkerStateInitial();
    }

    // Populate local position of BCE markers - on flexible bodies
    if (haveMesh1D) {
        SetMeshForceMap(m_data_mgr.countersH->numFsiNodes1D, m_data_mgr.flex1D_Nodes_H, m_data_mgr.flex1D_BCEsolids_H, m_flex1D_node_offsets, m_flex1D_contributions);
        m_data_mgr.flex1D_Nodes_D = m_data_mgr.flex1D_Nodes_H;
        m_data_mgr.flex1D_Normals_D = m_data_mgr.flex1D_Normals_H;
        m_data_mgr.flex1D_BCEsolids_D = m_data_mgr.flex1D_BCEsolids_H;
        m_data_mgr.flex1D_BCEcoords_D = m_data_mgr.flex1D_BCEcoords_H;
        //// TODO (Huzaifa): Try to see if this additional function is needed
        UpdateMeshMarker1DStateInitial();
    }

    if (haveMesh2D) {
        SetMeshForceMap(m_data_mgr.countersH->numFsiNodes2D, m_data_mgr.flex2D_Nodes_H, m_data_mgr.flex2D_BCEsolids_H, m_flex2D_node_offsets, m_flex2D_contributions);
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
    thrust::host_vector<uint> rigid_body_offsets(1, 0);

    m_rigid_grid_size = 0;
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
        rigid_body_offsets.push_back(m_rigid_grid_size);
    }

    // Copy vectors to device
    m_rigid_valid_threads = rigid_valid_threads;
    m_rigid_accumulated_threads = rigid_accumulated_threads;
    m_rigid_body_offsets = rigid_body_offsets;
    m_rigid_block_forces.resize(m_rigid_grid_size);
    m_rigid_block_torques.resize(m_rigid_grid_size);
}

// -----------------------------------------------------------------------------
// CalcRigidBodyBceAcceleration
// CalcFeaMesh1DBceAcceleration
// CalcFeaMesh2DBceAcceleration
// -----------------------------------------------------------------------------

__global__ void CalcRigidBceAcceleration_D(Real3* accelerations,             // BCE marker accelerations (output)
                                           const Real3* BCE_pos_local,       // BCE body-local coordinates
                                           const uint* body_IDs,             // rigid body ID for each BCE marker
                                           const Real4* body_rot,            // body orientation (relative to global frame)
                                           const Real3* body_angvel,         // body ang. vels. (relative to global frame)
                                           const Real3* body_linacc,         // body lin. acels. (relative to global frame)
                                           const Real3* body_angacc,         // body ang. acels. (relative to global frame)
                                           const uint* mapOriginalToSorted,  // sort map (if null, output in original order)
                                           uint out_offset) {                // original index of first output entry
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numRigidMarkers)
        return;

    uint marker_index = index + countersD.startRigidMarkers;
    uint out_index = mapOriginalToSorted ? mapOriginalToSorted[marker_index] : marker_index - out_offset;

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

    accelerations[out_index] = acc;
}

__global__ void CalcFeaMesh1DBceAcceleration_D(Real3* accelerations,             // [num BCEs on all solids]  BCE marker accelerations (output)
                                               const Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 1D nodes
                                               const uint2* flex1D_Nodes_D,      // [num segments]            node indices for each 1D segment
                                               const uint3* flex1D_BCEsolids_D,  // [num BCEs on 1D segments] association of flex BCEs with a mesh and segment
                                               const Real3* flex1D_BCEcoords_D,  // [num BCEs on 1D segments] local coordinates of BCE markers on FEA 1-D segments
                                               const uint* mapOriginalToSorted,  // sort map (if null, output in original order)
                                               uint out_offset) {                // original index of first output entry
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numMesh1DMarkers)
        return;

    uint marker_index = index + countersD.startMesh1DMarkers;
    uint out_index = mapOriginalToSorted ? mapOriginalToSorted[marker_index] : marker_index - out_offset;

    uint3 flex_solid = flex1D_BCEsolids_D[index];  // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;             // index of segment in associated mesh
    uint flex_seg = flex_solid.z;  // index of segment in global list

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 A0 = acc_fsi_fea_D[seg_nodes.x];       // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[seg_nodes.y];       // (absolute) acceleration of node 1

    Real t = flex1D_BCEcoords_D[index].x;
    accelerations[out_index] = A0 * (1 - t) + A1 * t;
}

__global__ void CalcFeaMesh2DBceAcceleration_D(Real3* accelerations,             // [num BCEs on all solids]  BCE marker accelerations (output)
                                               const Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 2D nodes
                                               const uint3* flex2D_Nodes_D,      // [num triangles]           triangle node indices
                                               const uint3* flex2D_BCEsolids_D,  // [num BCEs on 2D faces]    association of flex BCEs with a mesh and face
                                               const Real3* flex2D_BCEcoords_D,  // [num BCEs on 2D faces]    local coordinates of BCE markers on FEA 2-D faces
                                               const uint* mapOriginalToSorted,  // sort map (if null, output in original order)
                                               uint out_offset) {                // original index of first output entry
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numMesh2DMarkers)
        return;

    uint marker_index = index + countersD.startMesh2DMarkers;
    uint out_index = mapOriginalToSorted ? mapOriginalToSorted[marker_index] : marker_index - out_offset;

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

    accelerations[out_index] = A0 * lambda0 + A1 * lambda1 + A2 * lambda2;
}

void SphBceManager::CalcRigidBodyBceAcceleration(Real3* accelerations, const uint* map, uint offset) {
    uint numThreads, numBlocks;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, numBlocks, numThreads);

    CalcRigidBceAcceleration_D<<<numBlocks, numThreads>>>(                                         //
        accelerations,                                                                             //
        mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D),               //
        mR4CAST(m_data_mgr.fsiBodyState_D->rot), mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel),      //
        mR3CAST(m_data_mgr.fsiBodyState_D->lin_acc), mR3CAST(m_data_mgr.fsiBodyState_D->ang_acc),  //
        map, offset                                                                                //
    );
    if (m_check_errors) {
        gpuCheckError();
    }
}

void SphBceManager::CalcFeaMesh1DBceAcceleration(Real3* accelerations, const uint* map, uint offset) {
    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numMesh1DMarkers, 256, nBlocks, nThreads);

    CalcFeaMesh1DBceAcceleration_D<<<nBlocks, nThreads>>>(  //
        accelerations,                                      //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->acc),          //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                  //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),              //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D),             //
        map, offset                                         //
    );
    if (m_check_errors) {
        gpuCheckError();
    }
}

void SphBceManager::CalcFeaMesh2DBceAcceleration(Real3* accelerations, const uint* map, uint offset) {
    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numMesh2DMarkers, 256, nBlocks, nThreads);

    CalcFeaMesh2DBceAcceleration_D<<<nBlocks, nThreads>>>(  //
        accelerations,                                      //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->acc),          //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                  //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),              //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D),             //
        map, offset                                         //
    );
    if (m_check_errors) {
        gpuCheckError();
    }
}

// -----------------------------------------------------------------------------
// Calculate accelerations of solid BCE markers -> load m_data_mgr.bceAcc
// -----------------------------------------------------------------------------

void SphBceManager::updateBCEAcc() {
    auto acc = mR3CAST(m_data_mgr.bceAcc);
    auto map = U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted);

    if (m_data_mgr.countersH->numRigidMarkers > 0)
        CalcRigidBodyBceAcceleration(acc, map, 0);

    if (m_data_mgr.countersH->numMesh1DMarkers > 0)
        CalcFeaMesh1DBceAcceleration(acc, map, 0);

    if (m_data_mgr.countersH->numMesh2DMarkers > 0)
        CalcFeaMesh2DBceAcceleration(acc, map, 0);
}

std::vector<Real3> SphBceManager::GetSolidMarkerAccelerations() {
    const auto& c = *m_data_mgr.countersH;
    uint offset = (uint)c.startRigidMarkers;
    thrust::device_vector<Real3> acc_D(c.numAllMarkers - c.startRigidMarkers, mR3(0));

    if (c.numRigidMarkers > 0)
        CalcRigidBodyBceAcceleration(mR3CAST(acc_D), nullptr, offset);

    if (c.numMesh1DMarkers > 0)
        CalcFeaMesh1DBceAcceleration(mR3CAST(acc_D), nullptr, offset);

    if (c.numMesh2DMarkers > 0)
        CalcFeaMesh2DBceAcceleration(mR3CAST(acc_D), nullptr, offset);

    std::vector<Real3> acc_H(acc_D.size());
    thrust::copy(acc_D.begin(), acc_D.end(), acc_H.begin());
    return acc_H;
}

// -----------------------------------------------------------------------------
// CalcRigidBodyForces
// CalcFeaMesh1DForces
// CalcFeaMesh2DForces
// -----------------------------------------------------------------------------

__global__ void CalcRigidForces_D(Real3* __restrict__ block_forces,
                                  Real3* __restrict__ block_torques,
                                  const uint* __restrict__ rigidBodyBlockValidThreads,
                                  const uint* __restrict__ rigidBodyAccumulatedPaddedThreads,
                                  const Real4* __restrict__ derivatives,
                                  const Real4* __restrict__ positions,
                                  const uint* __restrict__ body_IDs,
                                  const Real3* __restrict__ body_pos,
                                  const uint* __restrict__ mapOriginalToSorted,
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

    Real3 Force = make_Real3(0.0f, 0.0f, 0.0f);
    Real3 Torque = make_Real3(0.0f, 0.0f, 0.0f);
    if (threadIdx_x < validThreads) {
        uint marker_index = global_index_padded + startRigidMarkers;
        uint sorted_index = mapOriginalToSorted[marker_index];
        uint body_ID = body_IDs[global_index_padded];
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

    // Standard block wise reduction - each block only contains elements from a single rigid body.
    // Padding threads hold zero force and torque, so they can take part in the reduction.
    for (uint stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (threadIdx_x < stride) {
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
        block_forces[blockIdx.x] = sharedForces[0];
        block_torques[blockIdx.x] = sharedTorques[0];
    }
}

__global__ void SumRigidForces_D(Real3* __restrict__ body_forces,
                                 Real3* __restrict__ body_torques,
                                 const Real3* __restrict__ block_forces,
                                 const Real3* __restrict__ block_torques,
                                 const uint* __restrict__ body_offsets) {
    extern __shared__ char sharedMem[];
    Real3* shared_forces = reinterpret_cast<Real3*>(sharedMem);
    Real3* shared_torques = shared_forces + blockDim.x;
    Real3 force = mR3(0);
    Real3 torque = mR3(0);
    for (uint i = body_offsets[blockIdx.x] + threadIdx.x; i < body_offsets[blockIdx.x + 1]; i += blockDim.x) {
        force += block_forces[i];
        torque += block_torques[i];
    }
    shared_forces[threadIdx.x] = force;
    shared_torques[threadIdx.x] = torque;
    __syncthreads();

    // A fixed reduction tree makes the sum independent of marker-block scheduling.
    for (uint stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (threadIdx.x < stride) {
            shared_forces[threadIdx.x] += shared_forces[threadIdx.x + stride];
            shared_torques[threadIdx.x] += shared_torques[threadIdx.x + stride];
        }
        __syncthreads();
    }
    if (threadIdx.x == 0) {
        body_forces[blockIdx.x] = shared_forces[0];
        body_torques[blockIdx.x] = shared_torques[0];
    }
}

template <uint nodes_per_marker>
__global__ void CalcMeshForces_D(Real3* __restrict__ node_forces,
                                 const uint* __restrict__ node_offsets,
                                 const uint* __restrict__ contributions,
                                 const Real4* __restrict__ derivatives,
                                 const Real3* __restrict__ coordinates,
                                 const uint* __restrict__ mapOriginalToSorted,
                                 uint start_markers,
                                 Real marker_mass) {
    extern __shared__ Real3 shared_forces[];
    uint node = blockIdx.x;
    Real3 force = mR3(0);
    // Each node owns a fixed sequence of marker contributions, independent of the sorted particle order.
    for (size_t i = size_t(node_offsets[node]) + threadIdx.x; i < node_offsets[node + 1]; i += blockDim.x) {
        uint contribution = contributions[i];
        uint marker = contribution / nodes_per_marker;
        uint local_node = contribution % nodes_per_marker;
        Real3 coord = coordinates[marker];
        Real weight;
        if constexpr (nodes_per_marker == 2)
            weight = local_node == 0 ? 1 - coord.x : coord.x;
        else
            weight = local_node == 0 ? coord.x : (local_node == 1 ? coord.y : 1 - coord.x - coord.y);
        uint sorted = mapOriginalToSorted[start_markers + marker];
        Real3 marker_force = mR3(derivatives[sorted]);
        if (paramsD.integration_scheme != IntegrationScheme::IMPLICIT_SPH)
            marker_force *= marker_mass;
        force += marker_force * weight;
    }
    shared_forces[threadIdx.x] = force;
    __syncthreads();
    for (uint stride = blockDim.x / 2; stride > 0; stride >>= 1) {
        if (threadIdx.x < stride)
            shared_forces[threadIdx.x] += shared_forces[threadIdx.x + stride];
        __syncthreads();
    }
    if (threadIdx.x == 0)
        node_forces[node] = shared_forces[0];
}

void SphBceManager::CalcRigidBodyForces() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    // Nothing to do if no FSI body has BCE markers (e.g., a body added only to carry an active domain)
    if (m_rigid_grid_size == 0) {
        thrust::fill(m_data_mgr.rigid_FSI_ForcesD.begin(), m_data_mgr.rigid_FSI_ForcesD.end(), mR3(0));
        thrust::fill(m_data_mgr.rigid_FSI_TorquesD.begin(), m_data_mgr.rigid_FSI_TorquesD.end(), mR3(0));
        return;
    }

    // Calculate shared memory size
    size_t sharedMemSize = 2 * m_rigid_block_size * sizeof(Real3);

    CalcRigidForces_D<<<m_rigid_grid_size, m_rigid_block_size, sharedMemSize>>>(
        mR3CAST(m_rigid_block_forces), mR3CAST(m_rigid_block_torques), U1CAST(m_rigid_valid_threads), U1CAST(m_rigid_accumulated_threads), mR4CAST(m_data_mgr.derivVelRhoD),
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), U1CAST(m_data_mgr.rigid_BCEsolids_D), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted), m_data_mgr.paramsH->markerMass, (uint)m_data_mgr.countersH->startRigidMarkers);

    const uint sum_block_size = 128;
    size_t sum_shared_size = 2 * sum_block_size * sizeof(Real3);
    SumRigidForces_D<<<m_data_mgr.countersH->numFsiBodies, sum_block_size, sum_shared_size>>>(
        mR3CAST(m_data_mgr.rigid_FSI_ForcesD), mR3CAST(m_data_mgr.rigid_FSI_TorquesD), mR3CAST(m_rigid_block_forces), mR3CAST(m_rigid_block_torques), U1CAST(m_rigid_body_offsets));

    if (m_check_errors) {
        gpuCheckError();
    }
}

void SphBceManager::CalcFeaMesh1DForces() {
    // The node-to-marker map is built only for meshes with elements (see Initialize)
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;
    auto num_nodes = m_data_mgr.countersH->numFsiNodes1D;

    constexpr uint block_size = 256;
    CalcMeshForces_D<2><<<num_nodes, block_size, block_size * sizeof(Real3)>>>(
        TCAST(m_data_mgr.flex1D_FSIforces_D), TCAST(m_flex1D_node_offsets), TCAST(m_flex1D_contributions), TCAST(m_data_mgr.derivVelRhoD), TCAST(m_data_mgr.flex1D_BCEcoords_D),
        TCAST(m_data_mgr.markersProximity_D->mapOriginalToSorted), (uint)m_data_mgr.countersH->startMesh1DMarkers, m_data_mgr.paramsH->markerMass);
    if (m_check_errors)
        gpuCheckError();
}

void SphBceManager::CalcFeaMesh2DForces() {
    // The node-to-marker map is built only for meshes with elements (see Initialize)
    if (m_data_mgr.countersH->numFsiElements2D == 0)
        return;
    auto num_nodes = m_data_mgr.countersH->numFsiNodes2D;

    constexpr uint block_size = 256;
    CalcMeshForces_D<3><<<num_nodes, block_size, block_size * sizeof(Real3)>>>(
        TCAST(m_data_mgr.flex2D_FSIforces_D), TCAST(m_flex2D_node_offsets), TCAST(m_flex2D_contributions), TCAST(m_data_mgr.derivVelRhoD), TCAST(m_data_mgr.flex2D_BCEcoords_D),
        TCAST(m_data_mgr.markersProximity_D->mapOriginalToSorted), (uint)m_data_mgr.countersH->startMesh2DMarkers, m_data_mgr.paramsH->markerMass);
    if (m_check_errors)
        gpuCheckError();
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
    velocities[sorted_index] = body_linvel[body_ID] + mR3(dot(u, omega_cross), dot(v, omega_cross), dot(w, omega_cross));
}

__global__ void UpdateBodyMarkerStateUnsorted_D(Real4* positions,            // global marker positions (original)
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
    velocities[marker_index] = body_linvel[body_ID] + mR3(dot(u, omega_cross), dot(v, omega_cross), dot(w, omega_cross));
}

void SphBceManager::UpdateBodyMarkerState() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    if (m_data_mgr.sortedSphMarkers2_D->size() == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerState_D<<<nBlocks, nThreads>>>(mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),
                                                   mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
                                                   mR4CAST(m_data_mgr.fsiBodyState_D->rot), mR3CAST(m_data_mgr.fsiBodyState_D->lin_vel),
                                                   mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel), U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted));

    if (m_check_errors) {
        gpuCheckError();
    }
}

void SphBceManager::UpdateBodyMarkerStateInitial() {
    if (m_data_mgr.countersH->numFsiBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerStateUnsorted_D<<<nBlocks, nThreads>>>(
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD), mR3CAST(m_data_mgr.rigid_BCEcoords_D), U1CAST(m_data_mgr.rigid_BCEsolids_D),
        mR3CAST(m_data_mgr.fsiBodyState_D->pos), mR4CAST(m_data_mgr.fsiBodyState_D->rot), mR3CAST(m_data_mgr.fsiBodyState_D->lin_vel), mR3CAST(m_data_mgr.fsiBodyState_D->ang_vel));

    if (m_check_errors) {
        gpuCheckError();
    }
}

// -----------------------------------------------------------------------------
// Options for nodal direction calculation for 2-D meshes
// (1-D node directions are always unit vectors, calculated as normalized sums of adjacent segment directions)
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
    __host__ __device__ void operator()(const int2& v) const { printf("%d %d   ", v.x, v.y); }
};

struct normalizeReal3 {
    __host__ __device__ Real3 operator()(const Real3& v) const { return get_normalized(v); }
};

struct averageDirs {
    __host__ __device__ Real3 operator()(const Real3& v, uint count) const { return v / Real(count); }
};

// Combine the element directions loaded to each element node into node directions.
// The extended arrays hold one entry per element node (in arbitrary node order); they are sorted in place by node index.
// The direction of each node is the normalized sum (if 'normalize' is true) or the average of the directions of all
// elements that share the node. Nodes not in any element are assigned a zero direction.
void CombineNodeDirections(thrust::device_vector<uint>& ext_nodes, thrust::device_vector<Real3>& ext_dirs, thrust::device_vector<Real3>& dirs, bool normalize) {
    // Sort by node index, so that all contributions to a given node are consecutive
    thrust::sort_by_key(ext_nodes.begin(), ext_nodes.end(), ext_dirs.begin());

    // Sum directions from adjacent elements
    thrust::device_vector<uint> out_nodes(ext_nodes.size());
    thrust::device_vector<Real3> out_dirs(ext_nodes.size());
    auto out_end = thrust::reduce_by_key(ext_nodes.begin(), ext_nodes.end(), ext_dirs.begin(), out_nodes.begin(), out_dirs.begin(), same_key_uint());
    auto num_out = out_end.first - out_nodes.begin();

    if (normalize) {
        // Normalize nodal directions
        thrust::transform(out_dirs.begin(), out_dirs.begin() + num_out, out_dirs.begin(), normalizeReal3());
    } else {
        // Average directions
        thrust::device_vector<uint> counts(num_out);
        thrust::reduce_by_key(ext_nodes.begin(), ext_nodes.end(), thrust::make_constant_iterator(1u), thrust::make_discard_iterator(), counts.begin(), same_key_uint());
        thrust::transform(out_dirs.begin(), out_dirs.begin() + num_out, counts.begin(), out_dirs.begin(), averageDirs());
    }

    // Scatter the combined directions to the corresponding nodes
    thrust::fill(dirs.begin(), dirs.end(), mR3(0, 0, 0));
    thrust::scatter(out_dirs.begin(), out_dirs.begin() + num_out, out_nodes.begin(), dirs.begin());
}

// Load the unit direction of each 1-D segment to its two nodes.
__global__ void CalcNodeDir1D_D(uint* ext_nodes,             // extended node indices (2 per segment)
                                Real3* ext_dirs,             // extended node directions (2 per segment)
                                const Real3* pos,            // positions of FEA 1-D segment nodes
                                const uint2* flex1D_Nodes_D  // segment node indices
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFsiElements1D)
        return;

    uint2 seg_nodes = flex1D_Nodes_D[index];  // indices of the 2 nodes on associated segment
    Real3 P0 = pos[seg_nodes.x];              // (absolute) position of node 0
    Real3 P1 = pos[seg_nodes.y];              // (absolute) position of node 1

    Real3 d = get_normalized(P1 - P0);

    ext_nodes[2 * index + 0] = seg_nodes.x;
    ext_nodes[2 * index + 1] = seg_nodes.y;
    ext_dirs[2 * index + 0] = d;
    ext_dirs[2 * index + 1] = d;
}

void SphBceManager::CalcNodeDirections1D(thrust::device_vector<Real3>& dirs) {
    uint num_segments = (uint)m_data_mgr.countersH->numFsiElements1D;

    uint nBlocks, nThreads;
    computeGridSize(num_segments, 256, nBlocks, nThreads);

    // Load segment directions to each adjacent node
    thrust::device_vector<uint> ext_nodes(2 * num_segments);
    thrust::device_vector<Real3> ext_dirs(2 * num_segments);
    CalcNodeDir1D_D<<<nBlocks, nThreads>>>(U1CAST(ext_nodes), mR3CAST(ext_dirs), mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), U2CAST(m_data_mgr.flex1D_Nodes_D));

    if (m_check_errors) {
        gpuCheckError();
    }

    // Combine the directions from adjacent segments into unit node directions (normalized sums).
    // Node directions are unit vectors; they are scaled by the segment length where used as Hermite tangents.
    CombineNodeDirections(ext_nodes, ext_dirs, dirs, true);
}

// Carry the segment reference normals over to the current segment configurations.
// The reference normal of a segment is projected onto the plane orthogonal to the current segment direction. As long as
// a segment rotates by less than 90 degrees between two calls, this results in a normal that varies continuously, with
// no rotation about the segment direction. The update is idempotent (repeated calls with the same node positions leave
// the normals unchanged).
__global__ void UpdateSegmentNormals1D_D(Real3* normals,              // segment reference normals (input/output)
                                         const Real3* pos,            // positions of FEA 1-D segment nodes
                                         const uint2* flex1D_Nodes_D  // segment node indices
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFsiElements1D)
        return;

    uint2 seg_nodes = flex1D_Nodes_D[index];  // indices of the 2 nodes on associated segment
    Real3 x_dir = get_normalized(pos[seg_nodes.y] - pos[seg_nodes.x]);
    Real3 y_dir;
    Real3 z_dir;
    get_projected_axes(x_dir, normals[index], y_dir, z_dir);

    normals[index] = y_dir;
}

void SphBceManager::UpdateSegmentNormals1D() {
    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numFsiElements1D, 256, nBlocks, nThreads);

    UpdateSegmentNormals1D_D<<<nBlocks, nThreads>>>(mR3CAST(m_data_mgr.flex1D_Normals_D), mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), U2CAST(m_data_mgr.flex1D_Nodes_D));

    if (m_check_errors) {
        gpuCheckError();
    }
}

__global__ void UpdateMeshMarker1DState_D(Real4* posRadD,                   // marker positions (output)
                                          Real3* velMasD,                   // marker velocities (output)
                                          const Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
                                          const Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
                                          bool use_dirs,                    // use nodal directions
                                          const Real3* dirs,                // nodal directions
                                          const uint2* flex1D_Nodes_D,      // segment node indices
                                          const Real3* flex1D_Normals_D,    // segment reference normals
                                          const uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
                                          const Real3* flex1D_BCEcoords_D,  // local coordinates of BCE markers on FEA 1-D segments
                                          const uint* mapOriginalToSorted   //
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numMesh1DMarkers)
        return;

    uint flex_index = index + countersD.startMesh1DMarkers;  // index for current 1-D flex BCE marker
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
        // Cubic Hermite interpolation; the Hermite tangents are the unit node directions scaled by the segment length
        Real len = length(P1 - P0);
        Real3 T0 = dirs[seg_nodes.x] * len;
        Real3 T1 = dirs[seg_nodes.y] * len;

        auto t2 = t * t;
        auto t3 = t2 * t;

        auto a0 = 2 * t3 - 3 * t2 + 1;
        auto a1 = -2 * t3 + 3 * t2;
        auto b0 = t3 - 2 * t2 + t;
        auto b1 = t3 - t2;
        P = P0 * a0 + P1 * a1 + T0 * b0 + T1 * b1;

        auto a0d = 6 * t2 - 6 * t;
        auto a1d = -6 * t2 + 6 * t;
        auto b0d = 3 * t2 - 4 * t + 1;
        auto b1d = 3 * t2 - 2 * t;
        D = P0 * a0d + P1 * a1d + T0 * b0d + T1 * b1d;
    } else {
        P = P0 * (1 - t) + P1 * t;
        D = P1 - P0;
    }

    // Create local frame (from the segment reference normal, for continuity in time and along the segment)
    Real3 x_dir = get_normalized(D);
    Real3 y_dir;
    Real3 z_dir;
    get_projected_axes(x_dir, flex1D_Normals_D[flex_seg], y_dir, z_dir);

    P += y_val * y_dir + z_val * z_dir;  // BCE marker position
    Real3 V = V0 * (1 - t) + V1 * t;     // BCE marker velocity

    uint sorted_flex_index = mapOriginalToSorted[flex_index];
    Real h = posRadD[sorted_flex_index].w;
    posRadD[sorted_flex_index] = mR4(P, h);
    velMasD[sorted_flex_index] = V;
}

__global__ void UpdateMeshMarker1DStateUnsorted_D(Real4* posRadD,                   // marker positions (output)
                                                  Real3* velMasD,                   // marker velocities (output)
                                                  const Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
                                                  const Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
                                                  bool use_dirs,                    // use nodal directions
                                                  const Real3* dirs,                // nodal directions
                                                  const uint2* flex1D_Nodes_D,      // segment node indices
                                                  const Real3* flex1D_Normals_D,    // segment reference normals
                                                  const uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
                                                  const Real3* flex1D_BCEcoords_D   // local coordinates of BCE markers on FEA 1-D segments
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numMesh1DMarkers)
        return;

    uint flex_index = index + countersD.startMesh1DMarkers;  // index for current 1-D flex BCE marker
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
        // Cubic Hermite interpolation; the Hermite tangents are the unit node directions scaled by the segment length
        Real len = length(P1 - P0);
        Real3 T0 = dirs[seg_nodes.x] * len;
        Real3 T1 = dirs[seg_nodes.y] * len;

        auto t2 = t * t;
        auto t3 = t2 * t;

        auto a0 = 2 * t3 - 3 * t2 + 1;
        auto a1 = -2 * t3 + 3 * t2;
        auto b0 = t3 - 2 * t2 + t;
        auto b1 = t3 - t2;
        P = P0 * a0 + P1 * a1 + T0 * b0 + T1 * b1;

        auto a0d = 6 * t2 - 6 * t;
        auto a1d = -6 * t2 + 6 * t;
        auto b0d = 3 * t2 - 4 * t + 1;
        auto b1d = 3 * t2 - 2 * t;
        D = P0 * a0d + P1 * a1d + T0 * b0d + T1 * b1d;
    } else {
        P = P0 * (1 - t) + P1 * t;
        D = P1 - P0;
    }

    // Create local frame (from the segment reference normal, for continuity in time and along the segment)
    Real3 x_dir = get_normalized(D);
    Real3 y_dir;
    Real3 z_dir;
    get_projected_axes(x_dir, flex1D_Normals_D[flex_seg], y_dir, z_dir);

    P += y_val * y_dir + z_val * z_dir;  // BCE marker position
    Real3 V = V0 * (1 - t) + V1 * t;     // BCE marker velocity

    Real h = posRadD[flex_index].w;
    posRadD[flex_index] = mR4(P, h);
    velMasD[flex_index] = V;
}

void SphBceManager::UpdateMeshMarker1DState() {
    if (m_data_mgr.countersH->numFsiElements1D == 0)
        return;

    if (m_data_mgr.sortedSphMarkers2_D->size() == 0)
        return;

    // If needed, calculate current node directions as averages
    if (m_node_directions_mode == NodeDirections::AVERAGE) {
        CalcNodeDirections1D(m_data_mgr.fsiMesh1DState_D->dir);
    }

    bool use_node_directions = (m_node_directions_mode != NodeDirections::NONE);

    // Carry the segment reference normals over to the current configuration
    UpdateSegmentNormals1D();

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numMesh1DMarkers, 256, nBlocks, nThreads);

    UpdateMeshMarker1DState_D<<<nBlocks, nThreads>>>(                                                        //
        mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD),  //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), mR3CAST(m_data_mgr.fsiMesh1DState_D->vel),                //
        use_node_directions, mR3CAST(m_data_mgr.fsiMesh1DState_D->dir),                                      //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                                                                   //
        mR3CAST(m_data_mgr.flex1D_Normals_D),                                                                //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                                                               //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D),                                                              //
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted)                                           //
    );

    if (m_check_errors) {
        gpuCheckError();
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

    // Carry the segment reference normals over to the current configuration
    UpdateSegmentNormals1D();

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numMesh1DMarkers, 256, nBlocks, nThreads);

    UpdateMeshMarker1DStateUnsorted_D<<<nBlocks, nThreads>>>(                                  //
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),  //
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), mR3CAST(m_data_mgr.fsiMesh1DState_D->vel),  //
        use_node_directions, mR3CAST(m_data_mgr.fsiMesh1DState_D->dir),                        //
        U2CAST(m_data_mgr.flex1D_Nodes_D),                                                     //
        mR3CAST(m_data_mgr.flex1D_Normals_D),                                                  //
        U3CAST(m_data_mgr.flex1D_BCEsolids_D),                                                 //
        mR3CAST(m_data_mgr.flex1D_BCEcoords_D)                                                 //
    );

    if (m_check_errors) {
        gpuCheckError();
    }
}

// -----------------------------------------------------------------------------
// CalcNodeDir2D
// UpdateMeshMarker2DState
// UpdateMeshMarker2DStateInitial
// -----------------------------------------------------------------------------

__global__ void CalcNodeDir2D_D(uint* ext_nodes,             // extended node indices (3 per triangle)
                                Real3* ext_dirs,             // extended node directions (3 per triangle)
                                const Real3* pos,            // positions of FEA 2-D triangle nodes
                                const uint3* flex2D_Nodes_D  // triangle node indices
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numFsiElements2D)
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
    ext_nodes[3 * index + 2] = tri_nodes.z;
    ext_dirs[3 * index + 0] = d;
    ext_dirs[3 * index + 1] = d;
    ext_dirs[3 * index + 2] = d;
}

void SphBceManager::CalcNodeDirections2D(thrust::device_vector<Real3>& dirs) {
    uint num_triangles = (uint)m_data_mgr.countersH->numFsiElements2D;

    uint nBlocks, nThreads;
    computeGridSize(num_triangles, 256, nBlocks, nThreads);

    // Load triangle normal directions to each adjacent node
    thrust::device_vector<uint> ext_nodes(3 * num_triangles);
    thrust::device_vector<Real3> ext_dirs(3 * num_triangles);
    CalcNodeDir2D_D<<<nBlocks, nThreads>>>(U1CAST(ext_nodes), mR3CAST(ext_dirs), mR3CAST(m_data_mgr.fsiMesh2DState_D->pos), U3CAST(m_data_mgr.flex2D_Nodes_D));

    if (m_check_errors) {
        gpuCheckError();
    }

    // Combine the directions from adjacent triangles
    CombineNodeDirections(ext_nodes, ext_dirs, dirs, NODAL_DIR_METHOD == 2);
}

//// TODO RADU - implement utility device function to interpolate position on triangle (linear or cubic)

__global__ void UpdateMeshMarker2DState_D(Real4* posRadD,                   // marker positions (output)
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
    if (index >= countersD.numMesh2DMarkers)
        return;

    uint flex_index = index + countersD.startMesh2DMarkers;  // index for current 2-D flex BCE marker
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

__global__ void UpdateMeshMarker2DStateUnsorted_D(Real4* posRadD,                   // marker positions (output)
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
    if (index >= countersD.numMesh2DMarkers)
        return;

    uint flex_index = index + countersD.startMesh2DMarkers;  // index for current 2-D flex BCE marker
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

    if (m_data_mgr.sortedSphMarkers2_D->size() == 0)
        return;

    // If needed, calculate current node directions as averages
    if (m_node_directions_mode == NodeDirections::AVERAGE) {
        CalcNodeDirections2D(m_data_mgr.fsiMesh2DState_D->dir);
    }

    bool use_node_directions = (m_node_directions_mode != NodeDirections::NONE);

    uint nBlocks, nThreads;
    computeGridSize((uint)m_data_mgr.countersH->numMesh2DMarkers, 256, nBlocks, nThreads);

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
        gpuCheckError();
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
    computeGridSize((uint)m_data_mgr.countersH->numMesh2DMarkers, 256, nBlocks, nThreads);

    UpdateMeshMarker2DStateUnsorted_D<<<nBlocks, nThreads>>>(                                  //
        mR4CAST(m_data_mgr.sphMarkers_D->posRadD), mR3CAST(m_data_mgr.sphMarkers_D->velMasD),  //
        mR3CAST(m_data_mgr.fsiMesh2DState_D->pos), mR3CAST(m_data_mgr.fsiMesh2DState_D->vel),  //
        use_node_directions, mR3CAST(m_data_mgr.fsiMesh2DState_D->dir),                        //
        U3CAST(m_data_mgr.flex2D_Nodes_D),                                                     //
        U3CAST(m_data_mgr.flex2D_BCEsolids_D),                                                 //
        mR3CAST(m_data_mgr.flex2D_BCEcoords_D)                                                 //
    );

    if (m_check_errors) {
        gpuCheckError();
    }
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
