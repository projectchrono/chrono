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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
#include <thrust/extrema.h>
#include <thrust/sort.h>
#include "chrono_fsi/ChFsiForceExplicitSPH.cuh"

//==========================================================================================================================================
namespace chrono {
namespace fsi {
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ Real3 deltaVShare(int3 gridPos,
                             uint index,
                             Real3 posRadA,
                             Real3 velMasA,
                             Real4 rhoPresMuA,
                             Real4* sortedPosRad,
                             Real3* sortedVelMas,
                             Real4* sortedRhoPreMu,
                             uint* cellStart,
                             uint* cellEnd) {
    uint gridHash = calcGridHash(gridPos);
    // get start of bucket for this cell
    Real3 deltaV = mR3(0.0f);

    uint startIndex = cellStart[gridHash];
    if (startIndex != 0xffffffff) {  // cell is not empty
        // iterate over particles in this cell
        uint endIndex = cellEnd[gridHash];

        for (uint j = startIndex; j < endIndex; j++) {
            if (j != index) {  // check not colliding with self
                Real3 posRadB = mR3(sortedPosRad[j]);
                Real3 dist3 = Distance(posRadA, posRadB);
                Real d = length(dist3);
                if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
                    continue;
                Real4 rhoPresMuB = sortedRhoPreMu[j];

                if (rhoPresMuB.w != -1.0)
                    continue;
                //# B must be fluid (A was checked originally and it is
                // fluid at this point), accoring to
                // colagrossi (2003), the other phase (i.e. rigid) should not be
                // considered)

                Real rho_bar = 0.5 * (rhoPresMuA.x + rhoPresMuB.x);

                Real3 velMasB = sortedVelMas[j];
                deltaV += paramsD.markerMass * (velMasB - velMasA) * W3(d) / rho_bar;
            }
        }
    }
    return deltaV;
}
//--------------------------------------------------------------------------------------------------------------------------------
// modify pressure for body force
__device__ __inline__ void modifyPressure(Real4& rhoPresMuB, const Real3& dist3Alpha) {
    // body force in x direction
    rhoPresMuB.y = (dist3Alpha.x > 0.5 * paramsD.boxDims.x) ? (rhoPresMuB.y - paramsD.deltaPress.x) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.x < -0.5 * paramsD.boxDims.x) ? (rhoPresMuB.y + paramsD.deltaPress.x) : rhoPresMuB.y;
    // body force in x direction
    rhoPresMuB.y = (dist3Alpha.y > 0.5 * paramsD.boxDims.y) ? (rhoPresMuB.y - paramsD.deltaPress.y) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.y < -0.5 * paramsD.boxDims.y) ? (rhoPresMuB.y + paramsD.deltaPress.y) : rhoPresMuB.y;
    // body force in x direction
    rhoPresMuB.y = (dist3Alpha.z > 0.5 * paramsD.boxDims.z) ? (rhoPresMuB.y - paramsD.deltaPress.z) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.z < -0.5 * paramsD.boxDims.z) ? (rhoPresMuB.y + paramsD.deltaPress.z) : rhoPresMuB.y;
}
//--------------------------------------------------------------------------------------------------------------------------------
/**
 * @brief DifVelocityRho
 * @details  See SDKCollisionSystem.cuh
 */
__device__ inline Real4 DifVelocityRho(Real3& dist3,
                                       Real& d,
                                       Real3 posRadA,
                                       Real3 posRadB,
                                       Real3& velMasA,
                                       Real3& vel_XSPH_A,
                                       Real3& velMasB,
                                       Real3& vel_XSPH_B,
                                       Real4& rhoPresMuA,
                                       Real4& rhoPresMuB,
                                       Real multViscosity) {
    Real3 gradW = GradW(dist3);

    //    Real vAB_Dot_rAB = dot(velMasA - velMasB, dist3);
    //
    //    //	//*** Artificial viscosity type 1.1
    //    Real alpha = .001;
    //    Real c_ab = 10 * paramsD.v_Max;  // Ma = .1;//sqrt(7.0f * 10000 /
    //                                     //    ((rhoPresMuA.x + rhoPresMuB.x) / 2.0f));
    //                                     // Real h = paramsD.HSML;
    //    Real rho = .5f * (rhoPresMuA.x + rhoPresMuB.x);
    //    Real nu = alpha * paramsD.HSML * c_ab / rho;
    //
    //    //*** Artificial viscosity type 1.2
    //    //    Real nu = 22.8f * paramsD.mu0 / 2.0f / (rhoPresMuA.x * rhoPresMuB.x);
    //    Real3 derivV = -paramsD.markerMass *
    //                   (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x) -
    //                    nu * vAB_Dot_rAB / (d * d + paramsD.epsMinMarkersDis * paramsD.HSML * paramsD.HSML)) *
    //                   gradW;
    //    return mR4(derivV, rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));

    //*** Artificial viscosity type 2
    Real rAB_Dot_GradW = dot(dist3, gradW);
    Real rAB_Dot_GradW_OverDist = rAB_Dot_GradW / (d * d + paramsD.epsMinMarkersDis * paramsD.HSML * paramsD.HSML);
    Real3 derivV = -paramsD.markerMass *
                       (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) *
                       gradW +
                   paramsD.markerMass * (8.0f * multViscosity) * paramsD.mu0 *
                       pow(rhoPresMuA.x + rhoPresMuB.x, Real(-2)) * rAB_Dot_GradW_OverDist * (velMasA - velMasB);
    Real derivRho = rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW);
    //	Real zeta = 0;//.05;//.1;
    //	Real derivRho = rhoPresMuA.x * paramsD.markerMass * invrhoPresMuBx *
    //(dot(vel_XSPH_A - vel_XSPH_B, gradW)
    //			+ zeta * paramsD.HSML * (10 * paramsD.v_Max) * 2 * (rhoPresMuB.x
    /// rhoPresMuA.x - 1) *
    // rAB_Dot_GradW_OverDist
    //			);

    //--------------------------------
    // Ferrari Modification
    derivRho = paramsD.markerMass * dot(vel_XSPH_A - vel_XSPH_B, gradW);
    Real cA = FerrariCi(rhoPresMuA.x);
    Real cB = FerrariCi(rhoPresMuB.x);
    if (d > EPSILON)
        derivRho -= rAB_Dot_GradW / (d + paramsD.epsMinMarkersDis * paramsD.HSML) * max(cA, cB) *
                    (rhoPresMuB.x - rhoPresMuA.x) / rhoPresMuB.x;

    //    --------------------------------
    return mR4(derivV, derivRho);

    //	//*** Artificial viscosity type 1.3
    //    Real rAB_Dot_GradW = dot(dist3, gradW);
    //    Real3 derivV = -paramsD.markerMass *
    //                       (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x *
    //                       rhoPresMuB.x)) * gradW +
    //                   paramsD.markerMass / (rhoPresMuA.x * rhoPresMuB.x) * 2.0f * paramsD.mu0 * rAB_Dot_GradW /
    //                       (d * d + paramsD.epsMinMarkersDis * paramsD.HSML * paramsD.HSML) * (velMasA - velMasB);
    //    return mR4(derivV, rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ Real4 collideCell(int3 gridPos,
                             uint index,
                             Real3 posRadA,
                             Real3 velMasA,
                             Real3 vel_XSPH_A,
                             Real4 rhoPresMuA,
                             Real4* sortedPosRad,
                             Real3* sortedVelMas,
                             Real3* vel_XSPH_Sorted_D,
                             Real4* sortedRhoPreMu,
                             Real3* velMas_ModifiedBCE,
                             Real4* rhoPreMu_ModifiedBCE,
                             uint* gridMarkerIndex,
                             uint* cellStart,
                             uint* cellEnd) {
    uint gridHash = calcGridHash(gridPos);
    // get start of bucket for this cell
    Real4 derivVelRho = mR4(0);

    uint startIndex = cellStart[gridHash];
    if (startIndex == 0xffffffff) {  // cell is not empty
        return derivVelRho;
    }
    // iterate over particles in this cell
    uint endIndex = cellEnd[gridHash];

    for (uint j = startIndex; j < endIndex; j++) {
        if (j != index) {  // check not colliding with self
            Real3 posRadB = mR3(sortedPosRad[j]);
            Real3 dist3Alpha = posRadA - posRadB;
            //			Real3 dist3 = Distance(posRadA, posRadB);
            Real3 dist3 = Modify_Local_PosB(posRadB, posRadA);
            Real d = length(dist3);
            if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
                continue;

            Real4 rhoPresMuB = sortedRhoPreMu[j];
            //			// old version. When rigid-rigid contact used to
            // be handled within fluid
            //			if ((fabs(rhoPresMuB.w - rhoPresMuA.w) < .1)
            //					&& rhoPresMuA.w > -.1) {
            //				continue;
            //			}
            if (rhoPresMuA.w > -.1 && rhoPresMuB.w > -.1) {  // no rigid-rigid force
                continue;
            }
            modifyPressure(rhoPresMuB, dist3Alpha);
            if (!(isfinite(rhoPresMuB.x) && isfinite(rhoPresMuB.y) && isfinite(rhoPresMuB.z))) {
                printf("Error! particle rhoPresMuB is NAN: thrown from modifyPressure !\n");
            }

            Real3 velMasB = sortedVelMas[j];
            if (rhoPresMuB.w > -1.0) {
                int bceIndexB = gridMarkerIndex[j] - (numObjectsD.numFluidMarkers);
                if (!(bceIndexB >= 0 && bceIndexB < numObjectsD.numBoundaryMarkers + numObjectsD.numRigid_SphMarkers)) {
                    printf("Error! bceIndex out of bound, collideCell !\n");
                }
                rhoPresMuB = rhoPreMu_ModifiedBCE[bceIndexB];
                velMasB = velMas_ModifiedBCE[bceIndexB];
            }
            Real multViscosit = 1;
            Real4 derivVelRhoAB = mR4(0.0f);
            Real3 vel_XSPH_B = vel_XSPH_Sorted_D[j];

            if (!(isfinite(rhoPresMuB.x) && isfinite(rhoPresMuB.y) && isfinite(rhoPresMuB.z))) {
                printf("Error! particle rhoPresMuB is NAN: thrown from collideCell ! type=%f\n", rhoPresMuB.w);
            }
            derivVelRhoAB = DifVelocityRho(dist3, d, posRadA, posRadB, velMasA, vel_XSPH_A, velMasB, vel_XSPH_B,
                                           rhoPresMuA, rhoPresMuB, multViscosit);
            derivVelRho += derivVelRhoAB;
        }
    }

    return derivVelRho;
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void newVel_XSPH_D(Real3* vel_XSPH_Sorted_D,  // output: new velocity
                              Real4* sortedPosRad,       // input: sorted positions
                              Real3* sortedVelMas,       // input: sorted velocities
                              Real4* sortedRhoPreMu,
                              uint* gridMarkerIndex,  // input: sorted particle indices
                              uint* cellStart,
                              uint* cellEnd,
                              uint numAllMarkers,
                              volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers)
        return;

    // read particle data from sorted arrays
    sortedRhoPreMu[index].y = Eos(sortedRhoPreMu[index].x, sortedRhoPreMu[index].w);  //

    Real4 rhoPreMuA = sortedRhoPreMu[index];
    Real3 velMasA = sortedVelMas[index];
    //    if (rhoPreMuA.w > -0.1) {  // v_XSPH is calculated only for fluid markers. Keep
    //                               // unchanged if not fluid.
    //        vel_XSPH_Sorted_D[index] = velMasA;
    //        return;
    //    }

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real3 deltaV = mR3(0);

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    /// if (gridPos.x == paramsD.gridSize.x-1) printf("****aha %d %d\n",
    /// gridPos.x, paramsD.gridSize.x);

    // examine neighbouring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                deltaV += deltaVShare(neighbourPos, index, posRadA, velMasA, rhoPreMuA, sortedPosRad, sortedVelMas,
                                      sortedRhoPreMu, cellStart, cellEnd);
            }
        }
    }
    //   // write new velocity back to original unsorted location
    // sortedVel_XSPH[index] = velMasA + paramsD.EPS_XSPH * deltaV;

    // write new velocity back to original unsorted location
    // uint originalIndex = gridMarkerIndex[index];
    Real3 vXSPH = velMasA + paramsD.EPS_XSPH * deltaV;
    if (!(isfinite(vXSPH.x) && isfinite(vXSPH.y) && isfinite(vXSPH.z))) {
        printf("Error! particle vXSPH is NAN: thrown from ChFsiForceExplicitSPH.cu, newVel_XSPH_D !\n");
        *isErrorD = true;
    }
    vel_XSPH_Sorted_D[index] = vXSPH;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void collideD(Real4* sortedDerivVelRho_fsi_D,  // output: new velocity
                         Real4* sortedPosRad,             // input: sorted positions
                         Real3* sortedVelMas,             // input: sorted velocities
                         Real3* vel_XSPH_Sorted_D,
                         Real4* sortedRhoPreMu,
                         Real3* velMas_ModifiedBCE,
                         Real4* rhoPreMu_ModifiedBCE,
                         uint* gridMarkerIndex,
                         uint* cellStart,
                         uint* cellEnd,
                         uint numAllMarkers,
                         volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers)
        return;

    // read particle data from sorted arrays
    Real3 posRadA = mR3(sortedPosRad[index]);
    Real3 velMasA = sortedVelMas[index];
    Real4 rhoPreMuA = sortedRhoPreMu[index];

    // *** comment these couple of lines since we don't want the force on the
    // rigid (or boundary) be influenced by ADAMi
    // *** method since it would cause large forces. ADAMI method is used only to
    // calculate forces on the fluid markers
    // (A)
    // *** near the boundary or rigid (B).
    //	if (rhoPreMuA.w > -.1) {
    //		int bceIndex = gridMarkerIndex[index] -
    //(numObjectsD.numFluidMarkers);
    //		if (!(bceIndex >= 0 && bceIndex < numObjectsD.numBoundaryMarkers +
    // numObjectsD.numRigid_SphMarkers)) {
    //			printf("Error! bceIndex out of bound, collideD !\n");
    //			*isErrorD = true;
    //		}
    //		rhoPreMuA = rhoPreMu_ModifiedBCE[bceIndex];
    //		velMasA = velMas_ModifiedBCE[bceIndex];
    //	}

    //	uint originalIndex = gridMarkerIndex[index];
    Real3 vel_XSPH_A = vel_XSPH_Sorted_D[index];
    Real4 derivVelRho = sortedDerivVelRho_fsi_D[index];

    if (!(isfinite(derivVelRho.x) && isfinite(derivVelRho.y) && isfinite(derivVelRho.z), isfinite(derivVelRho.w))) {
        printf("Error 0! particle derivVel is NAN: thrown from ChFsiForceExplicitSPH.cu, collideD !\n");
        *isErrorD = true;
    }

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    // examine neighbouring cells
    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            for (int z = -1; z <= 1; z++) {
                derivVelRho +=
                    collideCell(gridPos + mI3(x, y, z), index, posRadA, velMasA, vel_XSPH_A, rhoPreMuA, sortedPosRad,
                                sortedVelMas, vel_XSPH_Sorted_D, sortedRhoPreMu, velMas_ModifiedBCE,
                                rhoPreMu_ModifiedBCE, gridMarkerIndex, cellStart, cellEnd);
            }
        }
    }

    if (!(isfinite(derivVelRho.x) && isfinite(derivVelRho.y) && isfinite(derivVelRho.z))) {
        printf("Error! particle derivVel is NAN: thrown from ChFsiForceExplicitSPH.cu, collideD !\n");
        *isErrorD = true;
    }
    if (!(isfinite(derivVelRho.w))) {
        printf("Error! particle derivRho is NAN: thrown from ChFsiForceExplicitSPH.cu, collideD !\n");
        *isErrorD = true;
    }
    sortedDerivVelRho_fsi_D[index] = derivVelRho;
}

//--------------------------------------------------------------------------------------------------------------------------------
ChFsiForceExplicitSPH::ChFsiForceExplicitSPH(
    ChBce* otherBceWorker,                   ///< Pointer to the ChBce object that handles BCE markers
    SphMarkerDataD* otherSortedSphMarkersD,  ///< Information of markers in the sorted array on device
    ProximityDataD*
        otherMarkersProximityD,           ///< Pointer to the object that holds the proximity of the markers on device
    FsiGeneralData* otherFsiGeneralData,  ///< Pointer to the sph general data
    SimParams* otherParamsH,              ///< Pointer to the simulation parameters on host
    NumberOfObjects* otherNumObjects      ///< Pointer to number of objects, fluid and boundary markers, etc.
    )
    : ChFsiForce(otherBceWorker,
                 otherSortedSphMarkersD,
                 otherMarkersProximityD,
                 otherFsiGeneralData,
                 otherParamsH,
                 otherNumObjects) {
    CopyParams_NumberOfObjects(paramsH, numObjectsH);
}
//--------------------------------------------------------------------------------------------------------------------------------
ChFsiForceExplicitSPH::~ChFsiForceExplicitSPH() {}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::Finalize() {
    ChFsiForce::Finalize();
    cudaMemcpyToSymbolAsync(paramsD, paramsH, sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH, sizeof(NumberOfObjects));
    cudaMemcpyFromSymbol(paramsH, paramsD, sizeof(SimParams));
    cudaDeviceSynchronize();
}

void ChFsiForceExplicitSPH::ForceSPH(SphMarkerDataD* otherSphMarkersD,
                                     FsiBodiesDataD* otherFsiBodiesD,
                                     FsiMeshDataD* otherFsiMeshD) {
    sphMarkersD = otherSphMarkersD;
    fsiCollisionSystem->ArrangeData(sphMarkersD);
    bceWorker->ModifyBceVelocity(sphMarkersD, otherFsiBodiesD);
    CalculateXSPH_velocity();
    CollideWrapper();
    AddGravityToFluid();
}

void ChFsiForceExplicitSPH::CalculateXSPH_velocity() {
    /* Calculate vel_XSPH */
    if (vel_XSPH_Sorted_D.size() != numObjectsH->numAllMarkers) {
        printf("vel_XSPH_Sorted_D.size() %u numObjectsH->numAllMarkers %d \n", vel_XSPH_Sorted_D.size(),
               numObjectsH->numAllMarkers);
        throw std::runtime_error(
            "Error! size error vel_XSPH_Sorted_D Thrown from "
            "CalculateXSPH_velocity!\n");
    }

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    //------------------------------------------------------------------------
    /* thread per particle */
    uint numThreads, numBlocks;
    computeGridSize(numObjectsH->numAllMarkers, 64, numBlocks, numThreads);

    /* Execute the kernel */
    newVel_XSPH_D<<<numBlocks, numThreads>>>(
        mR3CAST(vel_XSPH_Sorted_D), mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), U1CAST(markersProximityD->gridMarkerIndexD),
        U1CAST(markersProximityD->cellStartD), U1CAST(markersProximityD->cellEndD), numObjectsH->numAllMarkers,
        isErrorD);

    cudaDeviceSynchronize();
    cudaCheckError();
    //------------------------------------------------------------------------
    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true) {
        throw std::runtime_error("Error! program crashed in  newVel_XSPH_D!\n");
    }
    cudaFree(isErrorD);
    free(isErrorH);
}

//--------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Wrapper function for collide
 * @details
 * 		See SDKCollisionSystem.cuh for informaton on collide
 */
void ChFsiForceExplicitSPH::collide(

    thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,
    thrust::device_vector<Real4>& sortedPosRad,
    thrust::device_vector<Real3>& sortedVelMas,
    thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
    thrust::device_vector<Real4>& sortedRhoPreMu,
    thrust::device_vector<Real3>& velMas_ModifiedBCE,
    thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,

    thrust::device_vector<uint>& gridMarkerIndex,
    thrust::device_vector<uint>& cellStart,
    thrust::device_vector<uint>& cellEnd) {
    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    //------------------------------------------------------------------------
    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize(numObjectsH->numAllMarkers, 64, numBlocks, numThreads);

    // execute the kernel
    collideD<<<numBlocks, numThreads>>>(
        mR4CAST(sortedDerivVelRho_fsi_D), mR4CAST(sortedPosRad), mR3CAST(sortedVelMas), mR3CAST(vel_XSPH_Sorted_D),
        mR4CAST(sortedRhoPreMu), mR3CAST(velMas_ModifiedBCE), mR4CAST(rhoPreMu_ModifiedBCE), U1CAST(gridMarkerIndex),
        U1CAST(cellStart), U1CAST(cellEnd), numObjectsH->numAllMarkers, isErrorD);

    cudaDeviceSynchronize();
    cudaCheckError();
    //------------------------------------------------------------------------
    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true) {
        throw std::runtime_error("Error! program crashed in  collideD!\n");
    }
    cudaFree(isErrorD);
    free(isErrorH);

    //					// unroll sorted index to have the location of original
    // particles in the sorted
    // arrays
    //					thrust::device_vector<uint> dummyIndex =
    // gridMarkerIndex;
    //					thrust::sort_by_key(dummyIndex.begin(),
    // dummyIndex.end(),
    //							derivVelRhoD.begin());
    //					dummyIndex.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceExplicitSPH::CollideWrapper() {
    thrust::device_vector<Real4> m_dSortedDerivVelRho_fsi_D(
        numObjectsH->numAllMarkers);  // Store Rho, Pressure, Mu of each particle
                                      // in the device memory
    thrust::fill(m_dSortedDerivVelRho_fsi_D.begin(), m_dSortedDerivVelRho_fsi_D.end(), mR4(0.0));

    collide(m_dSortedDerivVelRho_fsi_D, sortedSphMarkersD->posRadD, sortedSphMarkersD->velMasD, vel_XSPH_Sorted_D,
            sortedSphMarkersD->rhoPresMuD, bceWorker->velMas_ModifiedBCE, bceWorker->rhoPreMu_ModifiedBCE,
            markersProximityD->gridMarkerIndexD, markersProximityD->cellStartD, markersProximityD->cellEndD);

    CopySortedToOriginal_NonInvasive_R3(fsiGeneralData->vel_XSPH_D, vel_XSPH_Sorted_D,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R4(fsiGeneralData->derivVelRhoD, m_dSortedDerivVelRho_fsi_D,
                                        markersProximityD->gridMarkerIndexD);

    m_dSortedDerivVelRho_fsi_D.clear();
    // vel_XSPH_Sorted_D.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::AddGravityToFluid() {
    // add gravity to fluid markers
    /* Add outside forces. Don't add gravity to rigids, BCE, and boundaries, it is
     * added in ChSystem */
    Real3 totalFluidBodyForce3 = paramsH->bodyForce3 + paramsH->gravity;
    thrust::device_vector<Real4> bodyForceD(numObjectsH->numAllMarkers);
    thrust::fill(bodyForceD.begin(), bodyForceD.end(), mR4(totalFluidBodyForce3));
    thrust::transform(fsiGeneralData->derivVelRhoD.begin() + fsiGeneralData->referenceArray[0].x,
                      fsiGeneralData->derivVelRhoD.begin() + fsiGeneralData->referenceArray[0].y, bodyForceD.begin(),
                      fsiGeneralData->derivVelRhoD.begin() + fsiGeneralData->referenceArray[0].x,
                      thrust::plus<Real4>());
    bodyForceD.clear();
}

//--------------------------------------------------------------------------------------------------------------------------------

}  // namespace fsi
}  // namespace chrono
