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

#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"
#include <type_traits>

namespace chrono {
namespace fsi {

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
    if (index >= numObjectsD.numRigidMarkers)
        return;

    int rigidIndex = rigid_BCEsolids_D[index];
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
    Real4 q4 = qD[rigidIndex];
    Real3 a1, a2, a3;
    RotationMatirixFromQuaternion(a1, a2, a3, q4);
    Real3 dist3 = mR3(posRadD[rigidMarkerIndex]) - posRigidD[rigidIndex];
    Real3 dist3LF = InverseRotate_By_RotationMatrix_DeviceHost(a1, a2, a3, dist3);

    // Save the coordinates in the local reference of a rigid body
    rigid_BCEcoords_D[index] = dist3LF;
}

// -----------------------------------------------------------------------------

__global__ void CalcRigidForces_D(Real3* rigid_FSI_ForcesD,
                                  Real3* rigid_FSI_TorquesD,
                                  Real4* derivVelRhoD,
                                  Real4* posRadD,
                                  uint* rigid_BCEsolids_D,
                                  Real3* posRigidD,
                                  Real3* rigid_BCEcoords_D,
                                  uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigidMarkers)
        return;

    int RigidIndex = rigid_BCEsolids_D[index];
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
    uint sortedIndex = mapOriginalToSorted[rigidMarkerIndex];

    Real3 Force;

    if (paramsD.sph_method == SPHMethod::WCSPH) {
        Force = mR3(derivVelRhoD[sortedIndex]) * paramsD.markerMass;

    }
    // TODO: check IISPH
    else {
        Force = mR3(derivVelRhoD[sortedIndex]);
    }

    if (std::is_same<Real, double>::value) {
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].x), Force.x);
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].y), Force.y);
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].z), Force.z);
    } else {
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].x), Force.x);
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].y), Force.y);
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].z), Force.z);
    }

    Real3 dist3 = mR3(posRadD[sortedIndex]) - posRigidD[RigidIndex];
    Real3 mtorque = cross(dist3, Force);

    if (std::is_same<Real, double>::value) {
        atomicAdd_double((double*)&(rigid_FSI_TorquesD[RigidIndex].x), mtorque.x);
        atomicAdd_double((double*)&(rigid_FSI_TorquesD[RigidIndex].y), mtorque.y);
        atomicAdd_double((double*)&(rigid_FSI_TorquesD[RigidIndex].z), mtorque.z);
    } else {
        atomicAdd((float*)&(rigid_FSI_TorquesD[RigidIndex].x), mtorque.x);
        atomicAdd((float*)&(rigid_FSI_TorquesD[RigidIndex].y), mtorque.y);
        atomicAdd((float*)&(rigid_FSI_TorquesD[RigidIndex].z), mtorque.z);
    }
}

__global__ void CalcFlex1DForces_D(Real3* flex1D_FSIforces_D,  // FEA node forces (output)
                                   Real4* derivVelRhoD,        // dv/dt
                                   uint2* flex1D_Nodes_D,      // segment node indices
                                   uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
                                   Real3* flex1D_BCEcoords_D,   // local coordinates of BCE markers on FEA 1-D segments
                                   const uint* mapOriginalToSorted
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers1D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint sortedIndex = mapOriginalToSorted[flex_index];
    uint3 flex_solid = flex1D_BCEsolids_D[index];              // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    uint flex_seg = flex_solid.z;                              // index of segment in global list

    // Fluid force on BCE marker
    Real3 Force;
    if (paramsD.sph_method == SPHMethod::WCSPH) {
        Force = mR3(derivVelRhoD[sortedIndex]) * paramsD.markerMass;
    } else {
        Force = mR3(derivVelRhoD[sortedIndex]);
    }

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    uint n0 = seg_nodes.x;
    uint n1 = seg_nodes.y;

    Real lambda0 = flex1D_BCEcoords_D[index].x;  // segment coordinate
    Real lambda1 = 1 - lambda0;                  // segment coordinate

    // Split BCE marker force to the 2 nodes of the 1-D segment and accumulate
    if (std::is_same<Real, double>::value) {
        atomicAdd_double((double*)&(flex1D_FSIforces_D[n0].x), lambda0 * Force.x);
        atomicAdd_double((double*)&(flex1D_FSIforces_D[n0].y), lambda0 * Force.y);
        atomicAdd_double((double*)&(flex1D_FSIforces_D[n0].z), lambda0 * Force.z);

        atomicAdd_double((double*)&(flex1D_FSIforces_D[n1].x), lambda1 * Force.x);
        atomicAdd_double((double*)&(flex1D_FSIforces_D[n1].y), lambda1 * Force.y);
        atomicAdd_double((double*)&(flex1D_FSIforces_D[n1].z), lambda1 * Force.z);
    } else {
        atomicAdd((float*)&(flex1D_FSIforces_D[n0].x), lambda0 * Force.x);
        atomicAdd((float*)&(flex1D_FSIforces_D[n0].y), lambda0 * Force.y);
        atomicAdd((float*)&(flex1D_FSIforces_D[n0].z), lambda0 * Force.z);

        atomicAdd((float*)&(flex1D_FSIforces_D[n1].x), lambda1 * Force.x);
        atomicAdd((float*)&(flex1D_FSIforces_D[n1].y), lambda1 * Force.y);
        atomicAdd((float*)&(flex1D_FSIforces_D[n1].z), lambda1 * Force.z);
    }
}

__global__ void CalcFlex2DForces_D(Real3* flex2D_FSIforces_D,  // FEA node forces (output)
                                   Real4* derivVelRhoD,        // dv/dt
                                   uint3* flex2D_Nodes_D,      // triangle node indices
                                   uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
                                   Real3* flex2D_BCEcoords_D,   // local coordinates of BCE markers on FEA 2-D faces
                                   const uint* mapOriginalToSorted
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers2D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint sortedIndex = mapOriginalToSorted[flex_index];
    uint3 flex_solid = flex2D_BCEsolids_D[index];              // associated flex mesh and face
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;                              // index of triangle in global list

    // Fluid force on BCE marker
    Real3 Force;
    if (paramsD.sph_method == SPHMethod::WCSPH) {
        Force = mR3(derivVelRhoD[sortedIndex]) * paramsD.markerMass;
    } else {
        Force = mR3(derivVelRhoD[sortedIndex]);
    }

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    uint n0 = tri_nodes.x;
    uint n1 = tri_nodes.y;
    uint n2 = tri_nodes.z;

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate

    // Split BCE marker force to the 3 nodes of the 2-D face and accumulate
    if (std::is_same<Real, double>::value) {
        atomicAdd_double((double*)&(flex2D_FSIforces_D[n0].x), lambda0 * Force.x);
        atomicAdd_double((double*)&(flex2D_FSIforces_D[n0].y), lambda0 * Force.y);
        atomicAdd_double((double*)&(flex2D_FSIforces_D[n0].z), lambda0 * Force.z);

        atomicAdd_double((double*)&(flex2D_FSIforces_D[n1].x), lambda1 * Force.x);
        atomicAdd_double((double*)&(flex2D_FSIforces_D[n1].y), lambda1 * Force.y);
        atomicAdd_double((double*)&(flex2D_FSIforces_D[n1].z), lambda1 * Force.z);

        atomicAdd_double((double*)&(flex2D_FSIforces_D[n2].x), lambda2 * Force.x);
        atomicAdd_double((double*)&(flex2D_FSIforces_D[n2].y), lambda2 * Force.y);
        atomicAdd_double((double*)&(flex2D_FSIforces_D[n2].z), lambda2 * Force.z);
    } else {
        atomicAdd((float*)&(flex2D_FSIforces_D[n0].x), lambda0 * Force.x);
        atomicAdd((float*)&(flex2D_FSIforces_D[n0].y), lambda0 * Force.y);
        atomicAdd((float*)&(flex2D_FSIforces_D[n0].z), lambda0 * Force.z);

        atomicAdd((float*)&(flex2D_FSIforces_D[n1].x), lambda1 * Force.x);
        atomicAdd((float*)&(flex2D_FSIforces_D[n1].y), lambda1 * Force.y);
        atomicAdd((float*)&(flex2D_FSIforces_D[n1].z), lambda1 * Force.z);

        atomicAdd((float*)&(flex2D_FSIforces_D[n2].x), lambda2 * Force.x);
        atomicAdd((float*)&(flex2D_FSIforces_D[n2].y), lambda2 * Force.y);
        atomicAdd((float*)&(flex2D_FSIforces_D[n2].z), lambda2 * Force.z);
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
    if (bceIndex >= numObjectsD.numRigidMarkers)
        return;

    uint rigidMarkerIndex = bceIndex + numObjectsD.startRigidMarkers;
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

__global__ void CalcMeshMarker1DAcceleration_D(
    Real3* bceAcc,              // [num BCEs on all solids]  marker accelerations (output)
    Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 1D nodes
    uint2* flex1D_Nodes_D,      // [num segments]            node indices for each 1D segment
    uint3* flex1D_BCEsolids_D,  // [num BCEs on 1D segments] association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D,   // [num BCEs on 1D segments] local coordinates of BCE markers on FEA 1-D segments
    const uint* mapOriginalToSorted
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers1D)
        return;

    uint sortedIndex = mapOriginalToSorted[index + numObjectsD.startFlexMarkers1D];

    uint3 flex_solid = flex1D_BCEsolids_D[index];  // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;             // index of segment in associated mesh
    uint flex_seg = flex_solid.z;                  // index of segment in global list

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 A0 = acc_fsi_fea_D[seg_nodes.x];       // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[seg_nodes.y];       // (absolute) acceleration of node 1

    Real lambda0 = flex1D_BCEcoords_D[index].x;  // segment coordinate
    Real lambda1 = 1 - lambda0;                  // segment coordinate

    bceAcc[sortedIndex] = A0 * lambda0 + A1 * lambda1;
}

__global__ void CalcMeshMarker2DAcceleration_D(
    Real3* bceAcc,              // [num BCEs on all solids]  marker accelerations (output)
    Real3* acc_fsi_fea_D,       // [num nodes]               accelerations of FEA 2D nodes
    uint3* flex2D_Nodes_D,      // [num triangles]           triangle node indices
    uint3* flex2D_BCEsolids_D,  // [num BCEs on 1D segments] association of flex BCEs with a mesh and face
    Real3* flex2D_BCEcoords_D,   // [num BCEs on 1D segments] local coordinates of BCE markers on FEA 2-D faces
    const uint* mapOriginalToSorted
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers2D)
        return;

    uint sortedIndex = mapOriginalToSorted[index + numObjectsD.startFlexMarkers2D];

    uint3 flex_solid = flex2D_BCEsolids_D[index];  // associated flex mesh and face
    ////uint flex_mesh = flex_solid.x;                 // index of associated mesh
    ////uint flex_mesh_tri = flex_solid.y;             // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;                  // index of triangle in global list

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    Real3 A0 = acc_fsi_fea_D[tri_nodes.x];      // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[tri_nodes.y];      // (absolute) acceleration of node 1
    Real3 A2 = acc_fsi_fea_D[tri_nodes.z];      // (absolute) acceleration of node 2

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate

    bceAcc[sortedIndex] =
        A0 * lambda0 + A1 * lambda1 + A2 * lambda2;
}

// -----------------------------------------------------------------------------

__global__ void UpdateBodyMarkerState_D(Real4* posRadD,
                                       Real3* velMasD,
                                       Real3* rigid_BCEcoords_D,
                                       uint* rigid_BCEsolids_D,
                                       Real3* posRigidD,
                                       Real4* velMassRigidD,
                                       Real3* omegaLRF_D,
                                       Real4* qD,
                                       const uint* mapOriginalToSorted) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigidMarkers)
        return;

    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
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
    Real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
    Real3 omega3 = omegaLRF_D[rigidBodyIndex];
    Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
    velMasD[sortedIndex] = mR3(vM_Rigid) + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
}

__global__ void UpdateMeshMarker1DState_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
    uint2* flex1D_Nodes_D,      // segment node indices
    uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D,   // local coordinates of BCE markers on FEA 1-D segments
    const uint* mapOriginalToSorted
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers1D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint sortedIndex = mapOriginalToSorted[flex_index];
    uint3 flex_solid = flex1D_BCEsolids_D[index];              // associated flex mesh and segment
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    uint flex_seg = flex_solid.z;                              // index of segment in global list

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
    Real3* flex2D_BCEcoords_D,   // local coordinates of BCE markers on FEA 2-D faces
    const uint* mapOriginalToSorted
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers2D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint sortedIndex = mapOriginalToSorted[flex_index];
    uint3 flex_solid = flex2D_BCEsolids_D[index];              // associated flex mesh and face
    ////uint flex_mesh = flex_solid.x;                             // index of associated mesh
    ////uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;                              // index of triangle in global list

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
                                       Real4* velMassRigidD,
                                       Real3* omegaLRF_D,
                                       Real4* qD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigidMarkers)
        return;

    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
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
    Real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
    Real3 omega3 = omegaLRF_D[rigidBodyIndex];
    Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
    velMasD[rigidMarkerIndex] = mR3(vM_Rigid) + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS));
}

__global__ void UpdateMeshMarker1DStateUnsorted_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
    uint2* flex1D_Nodes_D,      // segment node indices
    uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D  // local coordinates of BCE markers on FEA 1-D segments
    ) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers1D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
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
    Real3* flex2D_BCEcoords_D  // local coordinates of BCE markers on FEA 2-D faces
    ) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers2D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
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

    Real h = posRadD[flex_index].w;
    posRadD[flex_index] = mR4(P, h);
    velMasD[flex_index] = V;
}

// =============================================================================

ChBce::ChBce(std::shared_ptr<SphMarkerDataD> sortedSphMarkers_D,
             std::shared_ptr<ProximityDataD> markersProximity_D,
             std::shared_ptr<FsiData> fsiData,
             std::shared_ptr<SimParams> paramsH,
             std::shared_ptr<ChCounters> numObjects,
             bool verbose)
    : ChFsiBase(paramsH, numObjects),
      m_sortedSphMarkersD(sortedSphMarkers_D),
      m_markersProximityD(markersProximity_D),
      m_fsiData(fsiData),
      m_verbose(verbose) {
    m_totalForceRigid.resize(0);
    m_totalTorqueRigid.resize(0);
}

ChBce::~ChBce() {}

// -----------------------------------------------------------------------------

void ChBce::Initialize(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                       std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                       std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                       std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D,
                       std::vector<int> fsiBodyBceNum) {
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(ChCounters));
    CopyParams_NumberOfObjects(paramsH, numObjectsH);

    // Resizing the arrays used to modify the BCE velocity and pressure according to Adami
    m_totalForceRigid.resize(numObjectsH->numRigidBodies);
    m_totalTorqueRigid.resize(numObjectsH->numRigidBodies);

    int haveGhost = (numObjectsH->numGhostMarkers > 0) ? 1 : 0;
    int haveHelper = (numObjectsH->numHelperMarkers > 0) ? 1 : 0;
    int haveRigid = (numObjectsH->numRigidBodies > 0) ? 1 : 0;
    int haveFlex1D = (numObjectsH->numFlexBodies1D > 0) ? 1 : 0;
    int haveFlex2D = (numObjectsH->numFlexBodies2D > 0) ? 1 : 0;

    auto numAllBce = numObjectsH->numBoundaryMarkers + numObjectsH->numRigidMarkers +  //
                     numObjectsH->numFlexMarkers1D + numObjectsH->numFlexMarkers2D;
    velMas_ModifiedBCE.resize(numAllBce);
    rhoPreMu_ModifiedBCE.resize(numAllBce);
    tauXxYyZz_ModifiedBCE.resize(numAllBce);
    tauXyXzYz_ModifiedBCE.resize(numAllBce);

    // Populate local position of BCE markers - on rigid bodies
    if (haveRigid) {
        Populate_RigidSPH_MeshPos_LRF(sphMarkers_D, fsiBodyState_D, fsiBodyBceNum);
        // TODO (Huzaifa): Try to see if this additional function is needed
        UpdateBodyMarkerStateInitial(sphMarkers_D, fsiBodyState_D);
    }

    // Populate local position of BCE markers - on flexible bodies
    if (haveFlex1D) {
        m_fsiData->flex1D_Nodes_D = m_fsiData->flex1D_Nodes_H;
        m_fsiData->flex1D_BCEsolids_D = m_fsiData->flex1D_BCEsolids_H;
        m_fsiData->flex1D_BCEcoords_D = m_fsiData->flex1D_BCEcoords_H;
        // TODO (Huzaifa): Try to see if this additional function is needed
        UpdateMeshMarker1DStateInitial(sphMarkers_D, fsiMesh1DState_D);
    }

    if (haveFlex2D) {
        m_fsiData->flex2D_Nodes_D = m_fsiData->flex2D_Nodes_H;
        m_fsiData->flex2D_BCEsolids_D = m_fsiData->flex2D_BCEsolids_H;
        m_fsiData->flex2D_BCEcoords_D = m_fsiData->flex2D_BCEcoords_H;
        // TODO (Huzaifa): Try to see if this additional function is needed
        UpdateMeshMarker2DStateInitial(sphMarkers_D, fsiMesh2DState_D);
    }
}

// -----------------------------------------------------------------------------

void ChBce::Populate_RigidSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                          std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                                          std::vector<int> fsiBodyBceNum) {
    // Create map between a BCE on a rigid body and the associated body ID
    uint start_bce = 0;
    for (int irigid = 0; irigid < fsiBodyBceNum.size(); irigid++) {
        uint end_bce = start_bce + fsiBodyBceNum[irigid];
        thrust::fill(m_fsiData->rigid_BCEsolids_D.begin() + start_bce, m_fsiData->rigid_BCEsolids_D.begin() + end_bce,
                     irigid);
        start_bce = end_bce;
    }

    uint nBlocks, nThreads;
    computeGridSize((uint)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    Populate_RigidSPH_MeshPos_LRF_D<<<nBlocks, nThreads>>>(
        mR3CAST(m_fsiData->rigid_BCEcoords_D), mR4CAST(sphMarkers_D->posRadD), U1CAST(m_fsiData->rigid_BCEsolids_D),
        mR3CAST(fsiBodyState_D->pos), mR4CAST(fsiBodyState_D->rot));

    cudaDeviceSynchronize();
    cudaCheckError();
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChBce::CalcRigidBceAcceleration(thrust::device_vector<Real3>& bceAcc,
                                     const thrust::device_vector<Real4>& q_fsiBodies_D,
                                     const thrust::device_vector<Real3>& accRigid_fsiBodies_D,
                                     const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
                                     const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,
                                     const thrust::device_vector<Real3>& rigid_BCEcoords_D,
                                     const thrust::device_vector<uint>& rigid_BCEsolids_D) {
    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize((uint)numObjectsH->numRigidMarkers, 256, numBlocks, numThreads);

    CalcRigidBceAccelerationD<<<numBlocks, numThreads>>>(
        mR3CAST(bceAcc), mR4CAST(q_fsiBodies_D), mR3CAST(accRigid_fsiBodies_D), mR3CAST(omegaVelLRF_fsiBodies_D),
        mR3CAST(omegaAccLRF_fsiBodies_D), mR3CAST(rigid_BCEcoords_D), U1CAST(rigid_BCEsolids_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted));

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::CalcMeshMarker1DAcceleration(thrust::device_vector<Real3>& bceAcc,
                                         std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D) {
    if (numObjectsH->numFlexBodies1D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers1D, 256, nBlocks, nThreads);

    CalcMeshMarker1DAcceleration_D<<<nBlocks, nThreads>>>(  //
        mR3CAST(bceAcc),                                    //
        mR3CAST(fsiMesh1DState_D->acc_fsi_fea_D),           //
        U2CAST(m_fsiData->flex1D_Nodes_D),                  //
        U3CAST(m_fsiData->flex1D_BCEsolids_D),              //
        mR3CAST(m_fsiData->flex1D_BCEcoords_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::CalcMeshMarker2DAcceleration(thrust::device_vector<Real3>& bceAcc,
                                         std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    if (numObjectsH->numFlexBodies2D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers2D, 256, nBlocks, nThreads);

    CalcMeshMarker2DAcceleration_D<<<nBlocks, nThreads>>>(  //
        mR3CAST(bceAcc),                                    //
        mR3CAST(fsiMesh2DState_D->acc_fsi_fea_D),           //
        U3CAST(m_fsiData->flex2D_Nodes_D),                  //
        U3CAST(m_fsiData->flex2D_BCEsolids_D),              //
        mR3CAST(m_fsiData->flex2D_BCEcoords_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::updateBCEAcc(std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                         std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                         std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    int size_ref = m_fsiData->referenceArray.size();
    int numBceMarkers = m_fsiData->referenceArray[size_ref - 1].y - m_fsiData->referenceArray[0].y;
    auto N_solid = numObjectsH->numRigidMarkers + numObjectsH->numFlexMarkers1D + numObjectsH->numFlexMarkers2D;
    auto N_all = N_solid + numObjectsH->numBoundaryMarkers;
    if (N_all != numBceMarkers) {
        throw std::runtime_error(
            "Error! Number of rigid, flexible and boundary markers are "
            "saved incorrectly. Thrown from updateBCEAcc!\n");
    }

    // Update portion set to boundary, rigid, and flexible BCE particles
    int4 updatePortion = mI4(m_fsiData->referenceArray[0].y, m_fsiData->referenceArray[1].y,
                             m_fsiData->referenceArray[2].y, m_fsiData->referenceArray[3].y);

    // Only update boundary BCE particles if no rigid/flexible particles
    if (size_ref == 2) {
        updatePortion.z = m_fsiData->referenceArray[1].y;
        updatePortion.w = m_fsiData->referenceArray[1].y;
    }

    // Update boundary and rigid/flexible BCE particles
    if (size_ref == 3)
        updatePortion.w = m_fsiData->referenceArray[2].y;

    // Calculate the acceleration of rigid/flexible BCE particles if exist, used for ADAMI BC
    // thrust::device_vector<Real3> bceAcc(numObjectsH->numRigidMarkers + numObjectsH->numFlexMarkers);

    // Acceleration of rigid BCE particles
    if (numObjectsH->numRigidMarkers > 0) {
        CalcRigidBceAcceleration(m_fsiData->bceAcc, fsiBodyState_D->rot, fsiBodyState_D->lin_acc, fsiBodyState_D->ang_vel,
                                 fsiBodyState_D->ang_acc, m_fsiData->rigid_BCEcoords_D, m_fsiData->rigid_BCEsolids_D);
    }

    if (numObjectsH->numFlexMarkers1D > 0) {
        CalcMeshMarker1DAcceleration(m_fsiData->bceAcc, fsiMesh1DState_D);
    }
    if (numObjectsH->numFlexMarkers2D > 0) {
        CalcMeshMarker1DAcceleration(m_fsiData->bceAcc, fsiMesh1DState_D);
    }
}

// -----------------------------------------------------------------------------

void ChBce::Rigid_Forces_Torques(std::shared_ptr<FsiBodyStateD> fsiBodyState_D) {
    if (numObjectsH->numRigidBodies == 0)
        return;

    thrust::fill(m_fsiData->rigid_FSI_ForcesD.begin(), m_fsiData->rigid_FSI_ForcesD.end(), mR3(0));
    thrust::fill(m_fsiData->rigid_FSI_TorquesD.begin(), m_fsiData->rigid_FSI_TorquesD.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((uint)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    CalcRigidForces_D<<<nBlocks, nThreads>>>(
        mR3CAST(m_fsiData->rigid_FSI_ForcesD), mR3CAST(m_fsiData->rigid_FSI_TorquesD), mR4CAST(m_fsiData->derivVelRhoD), mR4CAST(m_sortedSphMarkersD->posRadD),
        U1CAST(m_fsiData->rigid_BCEsolids_D), mR3CAST(fsiBodyState_D->pos), mR3CAST(m_fsiData->rigid_BCEcoords_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted));

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::Flex1D_Forces(std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D) {
    if (numObjectsH->numFlexBodies1D == 0)
        return;

    // Initialize accumulator to zero
    thrust::fill(m_fsiData->flex1D_FSIforces_D.begin(), m_fsiData->flex1D_FSIforces_D.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers1D, 256, nBlocks, nThreads);

    CalcFlex1DForces_D<<<nBlocks, nThreads>>>(                                   //
        mR3CAST(m_fsiData->flex1D_FSIforces_D),                                  //
        mR4CAST(m_fsiData->derivVelRhoD),                                        //
        U2CAST(m_fsiData->flex1D_Nodes_D),                                       //
        U3CAST(m_fsiData->flex1D_BCEsolids_D),                                   //
        mR3CAST(m_fsiData->flex1D_BCEcoords_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::Flex2D_Forces(std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    if (numObjectsH->numFlexBodies2D == 0)
        return;

    // Initialize accumulator to zero
    thrust::fill(m_fsiData->flex2D_FSIforces_D.begin(), m_fsiData->flex2D_FSIforces_D.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers2D, 256, nBlocks, nThreads);

    CalcFlex2DForces_D<<<nBlocks, nThreads>>>(                                   //
        mR3CAST(m_fsiData->flex2D_FSIforces_D),                                  //
        mR4CAST(m_fsiData->derivVelRhoD),  //
        U3CAST(m_fsiData->flex2D_Nodes_D),                                       //
        U3CAST(m_fsiData->flex2D_BCEsolids_D),                                   //
        mR3CAST(m_fsiData->flex2D_BCEcoords_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

// -----------------------------------------------------------------------------

void ChBce::UpdateBodyMarkerState(std::shared_ptr<FsiBodyStateD> fsiBodyState_D) {
    if (numObjectsH->numRigidBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerState_D<<<nBlocks, nThreads>>>(
        mR4CAST(m_sortedSphMarkersD->posRadD), mR3CAST(m_sortedSphMarkersD->velMasD),
        mR3CAST(m_fsiData->rigid_BCEcoords_D), U1CAST(m_fsiData->rigid_BCEsolids_D), mR3CAST(fsiBodyState_D->pos),
        mR4CAST(fsiBodyState_D->lin_vel), mR3CAST(fsiBodyState_D->ang_vel), mR4CAST(fsiBodyState_D->rot),
        U1CAST(m_markersProximityD->mapOriginalToSorted));

    cudaDeviceSynchronize();
    cudaCheckError();
}

// This is applied only during BCE initialization
void ChBce::UpdateBodyMarkerStateInitial(
    std::shared_ptr<SphMarkerDataD> sphMarkers_D, std::shared_ptr<FsiBodyStateD> fsiBodyState_D) {
    if (numObjectsH->numRigidBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerStateUnsorted_D<<<nBlocks, nThreads>>>(
        mR4CAST(sphMarkers_D->posRadD), mR3CAST(sphMarkers_D->velMasD),
        mR3CAST(m_fsiData->rigid_BCEcoords_D), U1CAST(m_fsiData->rigid_BCEsolids_D), mR3CAST(fsiBodyState_D->pos),
        mR4CAST(fsiBodyState_D->lin_vel), mR3CAST(fsiBodyState_D->ang_vel), mR4CAST(fsiBodyState_D->rot));

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::UpdateMeshMarker1DState(std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D) {
    if (numObjectsH->numFlexBodies1D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers1D, 256, nBlocks, nThreads);

    UpdateMeshMarker1DState_D<<<nBlocks, nThreads>>>(                                        //
        mR4CAST(m_sortedSphMarkersD->posRadD), mR3CAST(m_sortedSphMarkersD->velMasD),        //
        mR3CAST(fsiMesh1DState_D->pos_fsi_fea_D), mR3CAST(fsiMesh1DState_D->vel_fsi_fea_D),  //
        U2CAST(m_fsiData->flex1D_Nodes_D),                                                   //
        U3CAST(m_fsiData->flex1D_BCEsolids_D),                                               //
        mR3CAST(m_fsiData->flex1D_BCEcoords_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::UpdateMeshMarker1DStateInitial(
    std::shared_ptr<SphMarkerDataD> sphMarkers_D, std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D) {
    if (numObjectsH->numFlexBodies1D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers1D, 256, nBlocks, nThreads);

    UpdateMeshMarker1DStateUnsorted_D<<<nBlocks, nThreads>>>(                                        //
        mR4CAST(sphMarkers_D->posRadD), mR3CAST(sphMarkers_D->velMasD),        //
        mR3CAST(fsiMesh1DState_D->pos_fsi_fea_D), mR3CAST(fsiMesh1DState_D->vel_fsi_fea_D),  //
        U2CAST(m_fsiData->flex1D_Nodes_D),                                                   //
        U3CAST(m_fsiData->flex1D_BCEsolids_D),                                               //
        mR3CAST(m_fsiData->flex1D_BCEcoords_D)                                               //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::UpdateMeshMarker2DState(std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    if (numObjectsH->numFlexBodies2D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers2D, 256, nBlocks, nThreads);

    UpdateMeshMarker2DState_D<<<nBlocks, nThreads>>>(                                        //
        mR4CAST(m_sortedSphMarkersD->posRadD), mR3CAST(m_sortedSphMarkersD->velMasD),        //
        mR3CAST(fsiMesh2DState_D->pos_fsi_fea_D), mR3CAST(fsiMesh2DState_D->vel_fsi_fea_D),  //
        U3CAST(m_fsiData->flex2D_Nodes_D),                                                   //
        U3CAST(m_fsiData->flex2D_BCEsolids_D),                                               //
        mR3CAST(m_fsiData->flex2D_BCEcoords_D),
        U1CAST(m_markersProximityD->mapOriginalToSorted)  //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::UpdateMeshMarker2DStateInitial(
    std::shared_ptr<SphMarkerDataD> sphMarkers_D, std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    if (numObjectsH->numFlexBodies2D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers2D, 256, nBlocks, nThreads);

    UpdateMeshMarker2DStateUnsorted_D<<<nBlocks, nThreads>>>(                                        //
        mR4CAST(sphMarkers_D->posRadD), mR3CAST(sphMarkers_D->velMasD),        //
        mR3CAST(fsiMesh2DState_D->pos_fsi_fea_D), mR3CAST(fsiMesh2DState_D->vel_fsi_fea_D),  //
        U3CAST(m_fsiData->flex2D_Nodes_D),                                                   //
        U3CAST(m_fsiData->flex2D_BCEsolids_D),                                               //
        mR3CAST(m_fsiData->flex2D_BCEcoords_D)                                               //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

}  // end namespace fsi
}  // end namespace chrono
