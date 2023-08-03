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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Base class for processing boundary condition enforcing (bce) markers forces
// in FSI system.
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
                                  Real4* derivVelRhoD_old,
                                  Real4* posRadD,
                                  uint* rigid_BCEsolids_D,
                                  Real3* posRigidD,
                                  Real3* rigid_BCEcoords_D) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numRigidMarkers)
        return;

    int RigidIndex = rigid_BCEsolids_D[index];
    uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
    Real3 Force = (mR3(derivVelRhoD[rigidMarkerIndex]) * paramsD.Beta +
                   mR3(derivVelRhoD_old[rigidMarkerIndex]) * (1 - paramsD.Beta)) *
                  paramsD.markerMass;

    if (std::is_same<Real, double>::value) {
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].x), Force.x);
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].y), Force.y);
        atomicAdd_double((double*)&(rigid_FSI_ForcesD[RigidIndex].z), Force.z);
    } else {
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].x), Force.x);
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].y), Force.y);
        atomicAdd((float*)&(rigid_FSI_ForcesD[RigidIndex].z), Force.z);
    }
    Real3 dist3 = Distance(mR3(posRadD[rigidMarkerIndex]), posRigidD[RigidIndex]);
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
                                   Real4* derivVelRhoD_old,    // dv/dt
                                   uint2* flex1D_Nodes_D,      // segment node indices
                                   uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
                                   Real3* flex1D_BCEcoords_D   // local coordinates of BCE markers on FEA 1-D segments
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers1D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint3 flex_solid = flex1D_BCEsolids_D[index];              // associated flex mesh and segment
    uint flex_mesh = flex_solid.x;                             // index of associated mesh
    uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    uint flex_seg = flex_solid.z;                              // index of segment in global list

    // Fluid force on BCE marker
    Real3 Force =
        (mR3(derivVelRhoD[flex_index]) * paramsD.Beta + mR3(derivVelRhoD_old[flex_index]) * (1 - paramsD.Beta)) *
        paramsD.markerMass;

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
                                   Real4* derivVelRhoD_old,    // dv/dt
                                   uint3* flex2D_Nodes_D,      // triangle node indices
                                   uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
                                   Real3* flex2D_BCEcoords_D   // local coordinates of BCE markers on FEA 2-D faces
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers2D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint3 flex_solid = flex2D_BCEsolids_D[index];              // associated flex mesh and face
    uint flex_mesh = flex_solid.x;                             // index of associated mesh
    uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;                              // index of triangle in global list

    // Fluid force on BCE marker
    Real3 Force =
        (mR3(derivVelRhoD[flex_index]) * paramsD.Beta + mR3(derivVelRhoD_old[flex_index]) * (1 - paramsD.Beta)) *
        paramsD.markerMass;

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

__device__ void BCE_modification_Share(Real3& sumVW,
                                       Real3& sumRhoRW,
                                       Real& sumPW,
                                       Real3& sumTauXxYyZzW,
                                       Real3& sumTauXyXzYzW,
                                       Real& sumWFluid,
                                       int& isAffectedV,
                                       int& isAffectedP,
                                       int3 gridPos,
                                       Real3 posRadA,
                                       Real4* sortedPosRad,
                                       Real3* sortedVelMas,
                                       Real4* sortedRhoPreMu,
                                       Real3* sortedTauXxYyZz,
                                       Real3* sortedTauXyXzYz,
                                       uint* cellStart,
                                       uint* cellEnd) {
    uint gridHash = calcGridHash(gridPos);
    // get start of bucket for this cell
    uint startIndex = cellStart[gridHash];
    uint endIndex = cellEnd[gridHash];

    for (uint j = startIndex; j < endIndex; j++) {
        Real3 posRadB = mR3(sortedPosRad[j]);
        Real3 dist3 = Distance(posRadA, posRadB);
        Real dd = dist3.x * dist3.x + dist3.y * dist3.y + dist3.z * dist3.z;
        Real4 rhoPresMuB = sortedRhoPreMu[j];
        Real kernel_radius = RESOLUTION_LENGTH_MULT * paramsD.HSML;
        if (dd > kernel_radius * kernel_radius || rhoPresMuB.w > -0.5)
            continue;
        Real d = length(dist3);
        Real Wd = W3h(d, sortedPosRad[j].w);
        Real3 velMasB = sortedVelMas[j];
        sumVW += velMasB * Wd;
        sumRhoRW += rhoPresMuB.x * dist3 * Wd;
        sumPW += rhoPresMuB.y * Wd;
        sumWFluid += Wd;
        sumTauXxYyZzW += sortedTauXxYyZz[j] * Wd;
        sumTauXyXzYzW += sortedTauXyXzYz[j] * Wd;
    }
}

__global__ void BCE_VelocityPressureStress(Real3* velMas_ModifiedBCE,
                                           Real4* rhoPreMu_ModifiedBCE,
                                           Real3* tauXxYyZz_ModifiedBCE,
                                           Real3* tauXyXzYz_ModifiedBCE,
                                           Real4* sortedPosRad,
                                           Real3* sortedVelMas,
                                           Real4* sortedRhoPreMu,
                                           Real3* sortedTauXxYyZz,
                                           Real3* sortedTauXyXzYz,
                                           uint* cellStart,
                                           uint* cellEnd,
                                           uint* mapOriginalToSorted,
                                           uint* extendedActivityIdD,
                                           Real3* bceAcc,
                                           int2 newPortion,
                                           volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    uint sphIndex = index + newPortion.x;
    if (index >= newPortion.y - newPortion.x)
        return;

    // no need to do anything if it is not an active particle
    uint originalIndex = sphIndex;
    uint activity = extendedActivityIdD[originalIndex];
    if (activity == 0)
        return;

    uint bceIndex = index;
    if (paramsD.bceTypeWall == BceVersion::ORIGINAL)
        bceIndex = index + numObjectsD.numBoundaryMarkers;

    uint idA = mapOriginalToSorted[sphIndex];

    Real4 rhoPreMuA = sortedRhoPreMu[idA];
    Real3 posRadA = mR3(sortedPosRad[idA]);
    Real3 velMasA = sortedVelMas[idA];
    int isAffectedV = 0;
    int isAffectedP = 0;

    Real3 sumVW = mR3(0);
    Real3 sumRhoRW = mR3(0);
    Real sumPW = 0;
    Real sumWFluid = 0;
    Real3 sumTauXxYyZzW = mR3(0);
    Real3 sumTauXyXzYzW = mR3(0);

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    // examine neighbouring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                BCE_modification_Share(sumVW, sumRhoRW, sumPW, sumTauXxYyZzW, sumTauXyXzYzW, sumWFluid, isAffectedV,
                                       isAffectedP, neighbourPos, posRadA, sortedPosRad, sortedVelMas, sortedRhoPreMu,
                                       sortedTauXxYyZz, sortedTauXyXzYz, cellStart, cellEnd);
            }
        }
    }

    if (abs(sumWFluid) > EPSILON) {
        // modify velocity
        Real3 modifiedBCE_v = 2 * velMasA - sumVW / sumWFluid;
        velMas_ModifiedBCE[bceIndex] = modifiedBCE_v;

        // modify pressure and stress
        Real3 aW = mR3(0.0);
        if (rhoPreMuA.w > 0.5 && rhoPreMuA.w < 1.5) {  // type = 1
            int rigidBceIndex = sphIndex - numObjectsD.startRigidMarkers;
            if (rigidBceIndex < 0 || rigidBceIndex >= numObjectsD.numRigidMarkers) {
                printf("Error! particle index out of bound: thrown from ChBce.cu, BCE_VelocityPressureStress !\n");
                *isErrorD = true;
                return;
            }
            // rigid BCE marker acceleration
            aW = bceAcc[rigidBceIndex];
        }
        if (rhoPreMuA.w > 1.5 && rhoPreMuA.w < 2.5) {  // type = 2
            int flexBceIndex = sphIndex - numObjectsD.startFlexMarkers1D;
            if (flexBceIndex < 0 || flexBceIndex >= numObjectsD.numFlexMarkers1D) {
                printf("Error! particle index out of bound: thrown from ChBce.cu, BCE_VelocityPressureStress !\n");
                *isErrorD = true;
                return;
            }
            // flex1D BCE marker acceleration
            aW = bceAcc[flexBceIndex + numObjectsD.numRigidMarkers];
        }
        if (rhoPreMuA.w > 2.5 && rhoPreMuA.w < 3.5) {  // type = 3
            int flexBceIndex = sphIndex - numObjectsD.startFlexMarkers2D;
            if (flexBceIndex < 0 || flexBceIndex >= numObjectsD.numFlexMarkers2D) {
                printf("Error! particle index out of bound: thrown from ChBce.cu, BCE_VelocityPressureStress !\n");
                *isErrorD = true;
                return;
            }
            // flex2D BCE marker acceleration
            aW = bceAcc[flexBceIndex + numObjectsD.numRigidMarkers + numObjectsD.numFlexMarkers1D];
        }

        Real pressure = (sumPW + dot(paramsD.gravity - aW, sumRhoRW)) / sumWFluid;
        Real density = InvEos(pressure);
        rhoPreMu_ModifiedBCE[bceIndex] = mR4(density, pressure, rhoPreMuA.z, rhoPreMuA.w);
        if (paramsD.elastic_SPH) {
            Real3 tauXxYyZz = (sumTauXxYyZzW + dot(paramsD.gravity - aW, sumRhoRW)) / sumWFluid;
            Real3 tauXyXzYz = sumTauXyXzYzW / sumWFluid;
            tauXxYyZz_ModifiedBCE[bceIndex] = mR3(tauXxYyZz.x, tauXxYyZz.y, tauXxYyZz.z);
            tauXyXzYz_ModifiedBCE[bceIndex] = mR3(tauXyXzYz.x, tauXyXzYz.y, tauXyXzYz.z);
        }
    } else {
        rhoPreMu_ModifiedBCE[bceIndex] = mR4(paramsD.rho0, paramsD.BASEPRES, paramsD.mu0, rhoPreMuA.w);
        velMas_ModifiedBCE[bceIndex] = mR3(0.0);
        if (paramsD.elastic_SPH) {
            tauXxYyZz_ModifiedBCE[bceIndex] = mR3(0.0);
            tauXyXzYz_ModifiedBCE[bceIndex] = mR3(0.0);
        }
    }

    sortedVelMas[idA] = velMas_ModifiedBCE[bceIndex];
    sortedRhoPreMu[idA] = rhoPreMu_ModifiedBCE[bceIndex];
    if (paramsD.elastic_SPH) {
        sortedTauXxYyZz[idA] = tauXxYyZz_ModifiedBCE[bceIndex];
        sortedTauXyXzYz[idA] = tauXyXzYz_ModifiedBCE[bceIndex];
    }
}

// -----------------------------------------------------------------------------

__global__ void CalcRigidBceAccelerationD(Real3* bceAcc,
                                          Real4* q_fsiBodies_D,
                                          Real3* accRigid_fsiBodies_D,
                                          Real3* omegaVelLRF_fsiBodies_D,
                                          Real3* omegaAccLRF_fsiBodies_D,
                                          Real3* rigid_BCEcoords_D,
                                          const uint* rigid_BCEsolids_D) {
    uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (bceIndex >= numObjectsD.numRigidMarkers)
        return;

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

    bceAcc[bceIndex] = acc3;
}

__global__ void CalcMeshMarker1DAcceleration_D(
    Real3* bceAcc,              // marker accelerations (output)
    Real3* acc_fsi_fea_D,       // accelerations of FEA 1-D segment nodes
    uint2* flex1D_Nodes_D,      // segment node indices
    uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D   // local coordinates of BCE markers on FEA 1-D segments
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers1D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint3 flex_solid = flex1D_BCEsolids_D[index];              // associated flex mesh and segment
    uint flex_mesh = flex_solid.x;                             // index of associated mesh
    uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
    uint flex_seg = flex_solid.z;                              // index of segment in global list

    uint2 seg_nodes = flex1D_Nodes_D[flex_seg];  // indices of the 2 nodes on associated segment
    Real3 A0 = acc_fsi_fea_D[seg_nodes.x];       // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[seg_nodes.y];       // (absolute) acceleration of node 1

    Real lambda0 = flex1D_BCEcoords_D[index].x;  // segment coordinate
    Real lambda1 = 1 - lambda0;                  // segment coordinate

    bceAcc[flex_index] = A0 * lambda0 + A1 * lambda1;
}

__global__ void CalcMeshMarker2DAcceleration_D(
    Real3* bceAcc,              // marker accelerations (output)
    Real3* acc_fsi_fea_D,       // accelerations of FEA 2-D face nodes
    uint3* flex2D_Nodes_D,      // triangle node indices
    uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
    Real3* flex2D_BCEcoords_D   // local coordinates of BCE markers on FEA 2-D faces
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers2D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint3 flex_solid = flex2D_BCEsolids_D[index];              // associated flex mesh and face
    uint flex_mesh = flex_solid.x;                             // index of associated mesh
    uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
    uint flex_tri = flex_solid.z;                              // index of triangle in global list

    auto tri_nodes = flex2D_Nodes_D[flex_tri];  // indices of the 3 nodes on associated face
    Real3 A0 = acc_fsi_fea_D[tri_nodes.x];      // (absolute) acceleration of node 0
    Real3 A1 = acc_fsi_fea_D[tri_nodes.y];      // (absolute) acceleration of node 1
    Real3 A2 = acc_fsi_fea_D[tri_nodes.z];      // (absolute) acceleration of node 2

    Real lambda0 = flex2D_BCEcoords_D[index].x;  // barycentric coordinate
    Real lambda1 = flex2D_BCEcoords_D[index].y;  // barycentric coordinate
    Real lambda2 = 1 - lambda0 - lambda1;        // barycentric coordinate

    bceAcc[flex_index] = A0 * lambda0 + A1 * lambda1 + A2 * lambda2;
}

// -----------------------------------------------------------------------------

__global__ void UpdateBodyMarkerStateD(Real4* posRadD,
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

__global__ void UpdateMeshMarker1DState_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 1-D segment nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 1-D segment nodes
    uint2* flex1D_Nodes_D,      // segment node indices
    uint3* flex1D_BCEsolids_D,  // association of flex BCEs with a mesh and segment
    Real3* flex1D_BCEcoords_D   // local coordinates of BCE markers on FEA 1-D segments
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers1D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers1D;  // index for current 1-D flex BCE marker
    uint3 flex_solid = flex1D_BCEsolids_D[index];              // associated flex mesh and segment
    uint flex_mesh = flex_solid.x;                             // index of associated mesh
    uint flex_mesh_seg = flex_solid.y;                         // index of segment in associated mesh
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

    Real h = posRadD[flex_index].w;
    posRadD[flex_index] = mR4(P, h);
    velMasD[flex_index] = V;
}

__global__ void UpdateMeshMarker2DState_D(
    Real4* posRadD,             // marker positions (output)
    Real3* velMasD,             // marker velocities (output)
    Real3* pos_fsi_fea_D,       // positions of FEA 2-D face nodes
    Real3* vel_fsi_fea_D,       // velocities of FEA 2-D face nodes
    uint3* flex2D_Nodes_D,      // triangle node indices
    uint3* flex2D_BCEsolids_D,  // association of flex BCEs with a mesh and face
    Real3* flex2D_BCEcoords_D   // local coordinates of BCE markers on FEA 2-D faces
) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numFlexMarkers2D)
        return;

    uint flex_index = index + numObjectsD.startFlexMarkers2D;  // index for current 2-D flex BCE marker
    uint3 flex_solid = flex2D_BCEsolids_D[index];              // associated flex mesh and face
    uint flex_mesh = flex_solid.x;                             // index of associated mesh
    uint flex_mesh_tri = flex_solid.y;                         // index of triangle in associated mesh
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

    // Resizing the arrays used to modify the BCE velocity and pressure according to ADAMI
    m_totalForceRigid.resize(numObjectsH->numRigidBodies);
    m_totalTorqueRigid.resize(numObjectsH->numRigidBodies);

    int haveGhost = (numObjectsH->numGhostMarkers > 0) ? 1 : 0;
    int haveHelper = (numObjectsH->numHelperMarkers > 0) ? 1 : 0;
    int haveRigid = (numObjectsH->numRigidBodies > 0) ? 1 : 0;
    int haveFlex1D = (numObjectsH->numFlexBodies1D > 0) ? 1 : 0;
    int haveFlex2D = (numObjectsH->numFlexBodies2D > 0) ? 1 : 0;

    int num = haveHelper + haveGhost + haveRigid + haveFlex1D + haveFlex2D + 1;
    int numFlexRigidBoundaryMarkers =
        m_fsiData->referenceArray[num].y - m_fsiData->referenceArray[haveHelper + haveGhost].y;

    if (m_verbose) {
        printf("Total number of BCE particles = %d\n", numFlexRigidBoundaryMarkers);
        if (paramsH->bceType == BceVersion::ADAMI)
            printf("Boundary condition for rigid and flexible body is: ADAMI\n");
        if (paramsH->bceType == BceVersion::ORIGINAL)
            printf("Boundary condition for rigid and flexible body is: ORIGINAL\n");
        if (paramsH->bceTypeWall == BceVersion::ADAMI)
            printf("Boundary condition for fixed wall is: ADAMI\n");
        if (paramsH->bceTypeWall == BceVersion::ORIGINAL)
            printf("Boundary condition for fixed wall is: ORIGINAL\n");
    }

    auto numAllBce = numObjectsH->numBoundaryMarkers + numObjectsH->numRigidMarkers +  //
                     numObjectsH->numFlexMarkers1D + numObjectsH->numFlexMarkers2D;
    if ((int)numAllBce != numFlexRigidBoundaryMarkers) {
        throw std::runtime_error(
            "Error! number of flex and rigid and "
            "boundary markers are saved incorrectly!\n");
    }
    velMas_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);
    rhoPreMu_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);
    tauXxYyZz_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);
    tauXyXzYz_ModifiedBCE.resize(numFlexRigidBoundaryMarkers);

    // Populate local position of BCE markers - on rigid bodies
    if (haveRigid)
        Populate_RigidSPH_MeshPos_LRF(sphMarkers_D, fsiBodyState_D, fsiBodyBceNum);

    // Populate local position of BCE markers - on flexible bodies
    if (haveFlex1D) {
        m_fsiData->flex1D_BCEcoords_D = m_fsiData->flex1D_BCEcoords_H;
        UpdateMeshMarker1DState(sphMarkers_D, fsiMesh1DState_D);
    }
    if (haveFlex2D) {
        m_fsiData->flex2D_BCEcoords_D = m_fsiData->flex2D_BCEcoords_H;
        UpdateMeshMarker2DState(sphMarkers_D, fsiMesh2DState_D);
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

    UpdateBodyMarkerState(sphMarkers_D, fsiBodyState_D);
}

// -----------------------------------------------------------------------------

void ChBce::ReCalcVelocityPressureStress_BCE(thrust::device_vector<Real3>& velMas_ModifiedBCE,
                                             thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,
                                             thrust::device_vector<Real3>& tauXxYyZz_ModifiedBCE,
                                             thrust::device_vector<Real3>& tauXyXzYz_ModifiedBCE,
                                             const thrust::device_vector<Real4>& sortedPosRad,
                                             const thrust::device_vector<Real3>& sortedVelMas,
                                             const thrust::device_vector<Real4>& sortedRhoPreMu,
                                             const thrust::device_vector<Real3>& sortedTauXxYyZz,
                                             const thrust::device_vector<Real3>& sortedTauXyXzYz,
                                             const thrust::device_vector<uint>& cellStart,
                                             const thrust::device_vector<uint>& cellEnd,
                                             const thrust::device_vector<uint>& mapOriginalToSorted,
                                             const thrust::device_vector<uint>& extendedActivityIdD,
                                             const thrust::device_vector<Real3>& bceAcc,
                                             int4 updatePortion) {
    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    // thread per particle
    int2 newPortion = mI2(updatePortion.x, updatePortion.w);
    if (paramsH->bceTypeWall == BceVersion::ORIGINAL) {
        // Only implement ADAMI BC for rigid body boundary.
        // Implement a simple BC for fixed wall to avoid unnecessary cost.
        newPortion = mI2(updatePortion.y, updatePortion.w);
    }
    uint numBCE = newPortion.y - newPortion.x;
    uint numThreads, numBlocks;
    computeGridSize(numBCE, 256, numBlocks, numThreads);

    BCE_VelocityPressureStress<<<numBlocks, numThreads>>>(
        mR3CAST(velMas_ModifiedBCE), mR4CAST(rhoPreMu_ModifiedBCE), mR3CAST(tauXxYyZz_ModifiedBCE),
        mR3CAST(tauXyXzYz_ModifiedBCE), mR4CAST(sortedPosRad), mR3CAST(sortedVelMas), mR4CAST(sortedRhoPreMu),
        mR3CAST(sortedTauXxYyZz), mR3CAST(sortedTauXyXzYz), U1CAST(cellStart), U1CAST(cellEnd),
        U1CAST(mapOriginalToSorted), U1CAST(extendedActivityIdD), mR3CAST(bceAcc), newPortion, isErrorD);

    cudaDeviceSynchronize();
    cudaCheckError();

    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true)
        throw std::runtime_error("Error! program crashed in new_BCE_VelocityPressure!\n");

    cudaFree(isErrorD);
    free(isErrorH);
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
        mR3CAST(omegaAccLRF_fsiBodies_D), mR3CAST(rigid_BCEcoords_D), U1CAST(rigid_BCEsolids_D));

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
        mR3CAST(m_fsiData->flex1D_BCEcoords_D)              //
    );
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
        mR3CAST(m_fsiData->flex2D_BCEcoords_D)              //
    );
}

// -----------------------------------------------------------------------------

void ChBce::ModifyBceVelocityPressureStress(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                            std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                                            std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                                            std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    auto size_ref = m_fsiData->referenceArray.size();
    auto numBceMarkers = m_fsiData->referenceArray[size_ref - 1].y - m_fsiData->referenceArray[0].y;

    auto N_solid = numObjectsH->numRigidMarkers + numObjectsH->numFlexMarkers1D + numObjectsH->numFlexMarkers2D;
    auto N_all = N_solid + numObjectsH->numBoundaryMarkers;

    if ((int)N_all != numBceMarkers) {
        throw std::runtime_error(
            "Error! Number of rigid, flexible and boundary markers are "
            "saved incorrectly. Thrown from ModifyBceVelocityPressureStress!\n");
    }

    if (!(velMas_ModifiedBCE.size() == numBceMarkers && rhoPreMu_ModifiedBCE.size() == numBceMarkers &&
          tauXxYyZz_ModifiedBCE.size() == numBceMarkers && tauXyXzYz_ModifiedBCE.size() == numBceMarkers)) {
        throw std::runtime_error(
            "Error! Size error velMas_ModifiedBCE and "
            "tauXxYyZz_ModifiedBCE and tauXyXzYz_ModifiedBCE and "
            "rhoPreMu_ModifiedBCE. Thrown from ModifyBceVelocityPressureStress!\n");
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

    if (paramsH->bceType == BceVersion::ADAMI) {
        // ADAMI boundary condition (wall, rigid, flexible)

        // Calculate the acceleration of rigid/flexible BCE particles if exist, used for ADAMI BC
        thrust::device_vector<Real3> bceAcc(N_solid);

        // Acceleration of rigid BCE particles
        if (numObjectsH->numRigidMarkers > 0) {
            CalcRigidBceAcceleration(bceAcc, fsiBodyState_D->rot, fsiBodyState_D->lin_acc, fsiBodyState_D->ang_vel,
                                     fsiBodyState_D->ang_acc, m_fsiData->rigid_BCEcoords_D,
                                     m_fsiData->rigid_BCEsolids_D);
        }
        // Acceleration of flexible BCE particles
        if (numObjectsH->numFlexMarkers1D > 0) {
            CalcMeshMarker1DAcceleration(bceAcc, fsiMesh1DState_D);
        }
        if (numObjectsH->numFlexMarkers2D > 0) {
            CalcMeshMarker1DAcceleration(bceAcc, fsiMesh2DState_D);
        }

        if (paramsH->bceTypeWall == BceVersion::ORIGINAL) {
            // ADAMI BC for rigid/flexible body, ORIGINAL BC for fixed wall
            thrust::copy(sphMarkers_D->velMasD.begin() + updatePortion.x,
                         sphMarkers_D->velMasD.begin() + updatePortion.y, velMas_ModifiedBCE.begin());
            thrust::copy(sphMarkers_D->rhoPresMuD.begin() + updatePortion.x,
                         sphMarkers_D->rhoPresMuD.begin() + updatePortion.y, rhoPreMu_ModifiedBCE.begin());
            if (paramsH->elastic_SPH) {
                thrust::copy(sphMarkers_D->tauXxYyZzD.begin() + updatePortion.x,
                             sphMarkers_D->tauXxYyZzD.begin() + updatePortion.y, tauXxYyZz_ModifiedBCE.begin());
                thrust::copy(sphMarkers_D->tauXyXzYzD.begin() + updatePortion.x,
                             sphMarkers_D->tauXyXzYzD.begin() + updatePortion.y, tauXyXzYz_ModifiedBCE.begin());
            }
            if (N_solid > 0) {
                ReCalcVelocityPressureStress_BCE(
                    velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, tauXxYyZz_ModifiedBCE, tauXyXzYz_ModifiedBCE,
                    m_sortedSphMarkersD->posRadD, m_sortedSphMarkersD->velMasD, m_sortedSphMarkersD->rhoPresMuD,
                    m_sortedSphMarkersD->tauXxYyZzD, m_sortedSphMarkersD->tauXyXzYzD, m_markersProximityD->cellStartD,
                    m_markersProximityD->cellEndD, m_markersProximityD->mapOriginalToSorted,
                    m_fsiData->extendedActivityIdD, bceAcc, updatePortion);
            }
        } else if (paramsH->bceTypeWall == BceVersion::ADAMI) {
            // ADAMI BC for both rigid/flexible body and fixed wall

            ReCalcVelocityPressureStress_BCE(
                velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, tauXxYyZz_ModifiedBCE, tauXyXzYz_ModifiedBCE,
                m_sortedSphMarkersD->posRadD, m_sortedSphMarkersD->velMasD, m_sortedSphMarkersD->rhoPresMuD,
                m_sortedSphMarkersD->tauXxYyZzD, m_sortedSphMarkersD->tauXyXzYzD, m_markersProximityD->cellStartD,
                m_markersProximityD->cellEndD, m_markersProximityD->mapOriginalToSorted, m_fsiData->extendedActivityIdD,
                bceAcc, updatePortion);
        }

        bceAcc.clear();
    } else {
        // ORIGINAL boundary condition for all boundaries (wall, rigid, flexible)

        thrust::copy(sphMarkers_D->velMasD.begin() + updatePortion.x, sphMarkers_D->velMasD.begin() + updatePortion.w,
                     velMas_ModifiedBCE.begin());
        thrust::copy(sphMarkers_D->rhoPresMuD.begin() + updatePortion.x,
                     sphMarkers_D->rhoPresMuD.begin() + updatePortion.w, rhoPreMu_ModifiedBCE.begin());
        if (paramsH->elastic_SPH) {
            thrust::copy(sphMarkers_D->tauXxYyZzD.begin() + updatePortion.x,
                         sphMarkers_D->tauXxYyZzD.begin() + updatePortion.w, tauXxYyZz_ModifiedBCE.begin());
            thrust::copy(sphMarkers_D->tauXyXzYzD.begin() + updatePortion.x,
                         sphMarkers_D->tauXyXzYzD.begin() + updatePortion.w, tauXyXzYz_ModifiedBCE.begin());
        }
    }
}

// -----------------------------------------------------------------------------

void ChBce::Rigid_Forces_Torques(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                 std::shared_ptr<FsiBodyStateD> fsiBodyState_D) {
    if (numObjectsH->numRigidBodies == 0)
        return;

    thrust::fill(m_fsiData->rigid_FSI_ForcesD.begin(), m_fsiData->rigid_FSI_ForcesD.end(), mR3(0));
    thrust::fill(m_fsiData->rigid_FSI_TorquesD.begin(), m_fsiData->rigid_FSI_TorquesD.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((uint)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    CalcRigidForces_D<<<nBlocks, nThreads>>>(
        mR3CAST(m_fsiData->rigid_FSI_ForcesD), mR3CAST(m_fsiData->rigid_FSI_TorquesD), mR4CAST(m_fsiData->derivVelRhoD),
        mR4CAST(m_fsiData->derivVelRhoD_old), mR4CAST(sphMarkers_D->posRadD), U1CAST(m_fsiData->rigid_BCEsolids_D),
        mR3CAST(fsiBodyState_D->pos), mR3CAST(m_fsiData->rigid_BCEcoords_D));

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::Flex1D_Forces(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                          std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D) {
    if (numObjectsH->numFlexBodies1D == 0)
        return;

    // Initialize accumulator to zero
    thrust::fill(m_fsiData->flex1D_FSIforces_D.begin(), m_fsiData->flex1D_FSIforces_D.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers1D, 256, nBlocks, nThreads);

    CalcFlex1DForces_D<<<nBlocks, nThreads>>>(                                   //
        mR3CAST(m_fsiData->flex1D_FSIforces_D),                                  //
        mR4CAST(m_fsiData->derivVelRhoD), mR4CAST(m_fsiData->derivVelRhoD_old),  //
        U2CAST(m_fsiData->flex1D_Nodes_D),                                       //
        U3CAST(m_fsiData->flex1D_BCEsolids_D),                                   //
        mR3CAST(m_fsiData->flex1D_BCEcoords_D)                                   //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::Flex2D_Forces(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                          std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    if (numObjectsH->numFlexBodies2D == 0)
        return;

    // Initialize accumulator to zero
    thrust::fill(m_fsiData->flex2D_FSIforces_D.begin(), m_fsiData->flex2D_FSIforces_D.end(), mR3(0));

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers2D, 256, nBlocks, nThreads);

    CalcFlex2DForces_D<<<nBlocks, nThreads>>>(                                   //
        mR3CAST(m_fsiData->flex2D_FSIforces_D),                                  //
        mR4CAST(m_fsiData->derivVelRhoD), mR4CAST(m_fsiData->derivVelRhoD_old),  //
        U3CAST(m_fsiData->flex2D_Nodes_D),                                       //
        U3CAST(m_fsiData->flex2D_BCEsolids_D),                                   //
        mR3CAST(m_fsiData->flex2D_BCEcoords_D)                                   //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

// -----------------------------------------------------------------------------

void ChBce::UpdateBodyMarkerState(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                  std::shared_ptr<FsiBodyStateD> fsiBodyState_D) {
    if (numObjectsH->numRigidBodies == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numRigidMarkers, 256, nBlocks, nThreads);

    UpdateBodyMarkerStateD<<<nBlocks, nThreads>>>(
        mR4CAST(sphMarkers_D->posRadD), mR3CAST(sphMarkers_D->velMasD), mR3CAST(m_fsiData->rigid_BCEcoords_D),
        U1CAST(m_fsiData->rigid_BCEsolids_D), mR3CAST(fsiBodyState_D->pos), mR4CAST(fsiBodyState_D->lin_vel),
        mR3CAST(fsiBodyState_D->ang_vel), mR4CAST(fsiBodyState_D->rot));

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::UpdateMeshMarker1DState(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                    std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D) {
    if (numObjectsH->numFlexBodies1D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers1D, 256, nBlocks, nThreads);

    UpdateMeshMarker1DState_D<<<nBlocks, nThreads>>>(                                        //
        mR4CAST(sphMarkers_D->posRadD), mR3CAST(sphMarkers_D->velMasD),                      //
        mR3CAST(fsiMesh1DState_D->pos_fsi_fea_D), mR3CAST(fsiMesh1DState_D->vel_fsi_fea_D),  //
        U2CAST(m_fsiData->flex1D_Nodes_D),                                                   //
        U3CAST(m_fsiData->flex1D_BCEsolids_D),                                               //
        mR3CAST(m_fsiData->flex1D_BCEcoords_D)                                               //
    );

    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChBce::UpdateMeshMarker2DState(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                    std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D) {
    if (numObjectsH->numFlexBodies2D == 0)
        return;

    uint nBlocks, nThreads;
    computeGridSize((int)numObjectsH->numFlexMarkers2D, 256, nBlocks, nThreads);

    UpdateMeshMarker2DState_D<<<nBlocks, nThreads>>>(                                        //
        mR4CAST(sphMarkers_D->posRadD), mR3CAST(sphMarkers_D->velMasD),                      //
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
