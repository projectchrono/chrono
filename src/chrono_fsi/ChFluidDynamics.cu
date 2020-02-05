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
//
// Class for performing time integration in fluid system.//
// =============================================================================

#include "chrono_fsi/ChFluidDynamics.cuh"

namespace chrono {
namespace fsi {

// -----------------------------------------------------------------------------
/// Device function to calculate the share of density influence on a given
/// marker from all other markers in a given cell
__device__ void collideCellDensityReInit(Real& numerator,
                                         Real& denominator,
                                         int3 gridPos,
                                         uint index,
                                         Real3 posRadA,
                                         Real4* sortedPosRad,
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         uint* cellStart,
                                         uint* cellEnd) {
    //?c2 printf("grid pos %d %d %d \n", gridPos.x, gridPos.y, gridPos.z);
    uint gridHash = calcGridHash(gridPos);
    // get start of bucket for this cell

    uint startIndex = cellStart[gridHash];
    if (startIndex != 0xffffffff) {  // cell is not empty
        // iterate over particles in this cell
        uint endIndex = cellEnd[gridHash];

        for (uint j = startIndex; j < endIndex; j++) {
            Real3 posRadB = mR3(sortedPosRad[j]);
            Real4 rhoPreMuB = sortedRhoPreMu[j];
            Real3 dist3 = Distance(posRadA, posRadB);
            Real d = length(dist3);
            if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
                continue;
            numerator += paramsD.markerMass * W3(d);
            denominator += paramsD.markerMass / rhoPreMuB.x * W3(d);
        }
    }
}

// -----------------------------------------------------------------------------
/// Kernel to apply periodic BC along x
__global__ void ApplyPeriodicBoundaryXKernel(Real4* posRadD, Real4* rhoPresMuD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers) {
        return;
    }
    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1) {
        return;
    }  // no need to do anything if it is a boundary particle
    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMuD[index].y += paramsD.deltaPress.x;
        }
        return;
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMuD[index].y -= paramsD.deltaPress.x;
        }
        return;
    }
}

// -----------------------------------------------------------------------------
/// Kernel to apply inlet/outlet BC along x
__global__ void ApplyInletBoundaryXKernel(Real4* posRadD, Real3* VelMassD, Real4* rhoPresMuD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers) {
        return;
    }
    Real4 rhoPresMu = rhoPresMuD[index];
    if (rhoPresMu.w > 0.0) {
        return;
    }  // no need to do anything if it is a boundary particle
    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w <= 0.0) {
            rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.x;
            rhoPresMuD[index] = rhoPresMu;
        }
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);

        if (rhoPresMu.w <= -.1) {
            rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.x;
            rhoPresMuD[index] = rhoPresMu;
        }
    }

    if (posRad.x > -paramsD.x_in)
        rhoPresMuD[index].y = 0;

    if (posRad.x < paramsD.x_in) {
        //        Real vel = paramsD.V_in * 4 * (posRadD[index].z) * (0.41 - posRadD[index].z) / (0.41 * 0.41);
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);
    }
}

// -----------------------------------------------------------------------------
/// Kernel to apply periodic BC along y
__global__ void ApplyPeriodicBoundaryYKernel(Real4* posRadD, Real4* rhoPresMuD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers) {
        return;
    }
    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1) {
        return;
    }  // no need to do anything if it is a boundary particle
    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.y > paramsD.cMax.y) {
        posRad.y -= (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.y;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
    if (posRad.y < paramsD.cMin.y) {
        posRad.y += (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.y;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
}

// -----------------------------------------------------------------------------
/// Kernel to apply periodic BC along z
__global__ void ApplyPeriodicBoundaryZKernel(Real4* posRadD, Real4* rhoPresMuD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers) {
        return;
    }
    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1) {
        return;
    }  // no need to do anything if it is a boundary particle
    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.z > paramsD.cMax.z) {
        posRad.z -= (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.z;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
    if (posRad.z < paramsD.cMin.z) {
        posRad.z += (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.z;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
}
// -----------------------------------------------------------------------------
/// Kernel to update the fluid properities.
/// It updates the density, velocity and position relying on explicit Euler
/// scheme. Pressure is obtained from the density and an Equation of State.
__global__ void UpdateFluidD(Real4* posRadD,
                             Real3* velMasD,
                             Real3* vel_XSPH_D,
                             Real4* rhoPresMuD,
                             Real4* derivVelRhoD,
                             int2 updatePortion,
                             Real dT,
                             volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    index += updatePortion.x;  // updatePortion = [start, end] index of the update portion
    if (index >= updatePortion.y) {
        return;
    }
    Real4 derivVelRho = derivVelRhoD[index];
    Real4 rhoPresMu = rhoPresMuD[index];

    if (rhoPresMu.w < 0) {
        //-------------
        // ** position
        //-------------
        Real3 vel_XSPH = vel_XSPH_D[index];
        Real3 posRad = mR3(posRadD[index]);
        Real h = posRadD[index].x;
        Real3 updatedPositon = posRad + vel_XSPH * dT;
        if (!(isfinite(updatedPositon.x) && isfinite(updatedPositon.y) && isfinite(updatedPositon.z))) {
            printf("Error! particle position is NAN: thrown from ChFluidDynamics.cu, UpdateFluidDKernel !\n");
            *isErrorD = true;
            return;
        }
        posRadD[index] = mR4(updatedPositon, h);

        //-------------
        // ** velocity
        //-------------

        Real3 velMas = velMasD[index];
        Real3 updatedVelocity = velMas + mR3(derivVelRho) * dT;
        velMasD[index] = updatedVelocity;
    }
    Real rho2 = rhoPresMu.x + derivVelRho.w * dT;
    rhoPresMu.y = Eos(rho2, rhoPresMu.w);
    rhoPresMu.x = rho2;
    if (!(isfinite(rhoPresMu.x) && isfinite(rhoPresMu.y) && isfinite(rhoPresMu.z) && isfinite(rhoPresMu.w))) {
        printf("Error! particle rho pressure is NAN: thrown from ChFluidDynamics.cu, UpdateFluidDKernel !\n");
        *isErrorD = true;
        return;
    }
    rhoPresMuD[index] = rhoPresMu;  // rhoPresMuD updated
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Update_Fluid_State(Real3* XSPH_Vel,  // input: sorted velocities,
                                   Real4* posRad,    // input: sorted positions
                                   Real3* velMas,
                                   Real4* rhoPreMu,
                                   int4 updatePortion,
                                   uint numAllMarkers,
                                   double dT,
                                   volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (i_idx >= updatePortion.y)
        return;

    //    velMas[i_idx] = paramsD.EPS_XSPH * vis_vel[i_idx] + (1 - paramsD.EPS_XSPH) * new_vel[i_idx];
    velMas[i_idx] += paramsD.EPS_XSPH * XSPH_Vel[i_idx];

    Real3 newpos = mR3(posRad[i_idx]) + dT * velMas[i_idx];
    Real h = posRad[i_idx].w;
    posRad[i_idx] = mR4(newpos, h);

    if (!(isfinite(posRad[i_idx].x) && isfinite(posRad[i_idx].y) && isfinite(posRad[i_idx].z))) {
        printf("Error! particle %d position is NAN: thrown from ChFluidDynamics.cu, UpdateFluidDKernel  %f,%f,%f,%f\n",
               i_idx, posRad[i_idx].x, posRad[i_idx].y, posRad[i_idx].z, posRad[i_idx].w);
    }
    if (!(isfinite(rhoPreMu[i_idx].x) && isfinite(rhoPreMu[i_idx].y) && isfinite(rhoPreMu[i_idx].z))) {
        printf("Error! particle %d rhoPreMu is NAN: thrown from ChFluidDynamics.cu, UpdateFluidDKernel ! %f,%f,%f,%f\n",
               i_idx, rhoPreMu[i_idx].x, rhoPreMu[i_idx].y, rhoPreMu[i_idx].z, rhoPreMu[i_idx].w);
    }

    if (!(isfinite(velMas[i_idx].x) && isfinite(velMas[i_idx].y) && isfinite(velMas[i_idx].z))) {
        printf("Error! particle %d velocity is NAN: thrown from ChFluidDynamics.cu, UpdateFluidDKernel !%f,%f,%f\n",
               i_idx, velMas[i_idx].x, velMas[i_idx].y, velMas[i_idx].z);
    }
}

// -----------------------------------------------------------------------------
/// Kernel for updating the density.
/// It calculates the density of the markers. It does include the normalization
/// close to the boundaries and free surface.
__global__ void ReCalcDensityD_F1(Real4* dummySortedRhoPreMu,
                                  Real4* sortedPosRad,
                                  Real3* sortedVelMas,
                                  Real4* sortedRhoPreMu,
                                  uint* gridMarkerIndex,
                                  uint* cellStart,
                                  uint* cellEnd,
                                  uint numAllMarkers) {
    uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
    if (index >= numAllMarkers)
        return;

    // read particle data from sorted arrays
    Real3 posRadA = mR3(sortedPosRad[index]);
    Real4 rhoPreMuA = sortedRhoPreMu[index];

    /// If density initialization should only be applied to fluid markers
    //    if (rhoPreMuA.w > -.1)
    //        return;

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    Real numerator = 0.0;
    Real denominator = 0.0;
    // examine neighbouring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                collideCellDensityReInit(numerator, denominator, neighbourPos, index, posRadA, sortedPosRad,
                                         sortedVelMas, sortedRhoPreMu, cellStart, cellEnd);
            }
        }
    }

    rhoPreMuA.x = numerator;  /// denominator;
    //    rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
    dummySortedRhoPreMu[index] = rhoPreMuA;
}

// -----------------------------------------------------------------------------
// CLASS FOR FLUID DYNAMICS SYSTEM
// -----------------------------------------------------------------------------

ChFluidDynamics::ChFluidDynamics(ChBce* otherBceWorker,
                                 ChFsiDataManager* otherFsiData,
                                 SimParams* otherParamsH,
                                 NumberOfObjects* otherNumObjects,
                                 ChFluidDynamics::Integrator type)
    : fsiData(otherFsiData), paramsH(otherParamsH), numObjectsH(otherNumObjects) {
    myIntegrator = type;
    switch (myIntegrator) {
        case ChFluidDynamics::Integrator::IISPH:
            forceSystem =
                new ChFsiForceIISPH(otherBceWorker, &(fsiData->sortedSphMarkersD), &(fsiData->markersProximityD),
                                    &(fsiData->fsiGeneralData), paramsH, numObjectsH);
            printf("Created an IISPH framework.\n");
            break;

        case ChFluidDynamics::Integrator::ExplicitSPH:
            forceSystem =
                new ChFsiForceExplicitSPH(otherBceWorker, &(fsiData->sortedSphMarkersD), &(fsiData->markersProximityD),
                                          &(fsiData->fsiGeneralData), paramsH, numObjectsH);
            printf("Created an ExplicitSPHframe work.\n");
            break;

            /// Extend this function with your own linear solvers
        default:
            forceSystem =
                new ChFsiForceIISPH(otherBceWorker, &(fsiData->sortedSphMarkersD), &(fsiData->markersProximityD),
                                    &(fsiData->fsiGeneralData), paramsH, numObjectsH);
            std::cout << "The ChFsiForce you chose has not been implemented, reverting back to "
                         "ChFsiForceIISPH\n";
    }
}

// -----------------------------------------------------------------------------

void ChFluidDynamics::Finalize() {
    printf("ChFluidDynamics::Finalize()\n");
    forceSystem->Finalize();
    cudaMemcpyToSymbolAsync(paramsD, paramsH, sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH, sizeof(NumberOfObjects));

    cudaMemcpyFromSymbol(paramsH, paramsD, sizeof(SimParams));
    printf("Finished  ChFluidDynamics paramsD was=%f,%f,%f\n", paramsH->cellSize.x, paramsH->cellSize.y,
           paramsH->cellSize.z);
}

// -----------------------------------------------------------------------------

ChFluidDynamics::~ChFluidDynamics() {
    delete forceSystem;
}
// -----------------------------------------------------------------------------

void ChFluidDynamics::IntegrateSPH(SphMarkerDataD* sphMarkersD2,
                                   SphMarkerDataD* sphMarkersD1,
                                   FsiBodiesDataD* fsiBodiesD,
                                   FsiMeshDataD* fsiMeshD,
                                   Real dT) {
    if (GetIntegratorType() == ChFluidDynamics::Integrator::ExplicitSPH)
        forceSystem->ForceSPH(sphMarkersD2, fsiBodiesD, fsiMeshD);
    else
        forceSystem->ForceSPH(sphMarkersD1, fsiBodiesD, fsiMeshD);

    if (myIntegrator == ChFluidDynamics::Integrator::IISPH)
        this->UpdateFluid_Implicit(sphMarkersD2);
    else if (GetIntegratorType() == ChFluidDynamics::Integrator::ExplicitSPH)
        this->UpdateFluid(sphMarkersD2, dT);

    this->ApplyBoundarySPH_Markers(sphMarkersD2);
}

// -----------------------------------------------------------------------------

void ChFluidDynamics::UpdateFluid(SphMarkerDataD* sphMarkersD, Real dT) {
    //	int4 referencePortion = referenceArray[0];
    //	if (referennamespace chrono {
    //		printf("error in UpdateFluid, accessing non fluid\n");
    //		return;
    //	}
    //	int2 updatePortion = mI2(referencePortion);
    int2 updatePortion =
        mI2(0, fsiData->fsiGeneralData.referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1].y);
    // int2 updatePortion = mI2(referenceArray[0].x, referenceArray[0].y);

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    //------------------------
    uint nBlock_UpdateFluid, nThreads;
    computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid, nThreads);
    UpdateFluidD<<<nBlock_UpdateFluid, nThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR3CAST(fsiData->fsiGeneralData.vel_XSPH_D),
        mR4CAST(sphMarkersD->rhoPresMuD), mR4CAST(fsiData->fsiGeneralData.derivVelRhoD), updatePortion, dT, isErrorD);
    cudaDeviceSynchronize();
    cudaCheckError();
    //------------------------
    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true) {
        throw std::runtime_error("Error! program crashed in  UpdateFluidD!\n");
    }
    cudaFree(isErrorD);
    free(isErrorH);
}
void ChFluidDynamics::UpdateFluid_Implicit(SphMarkerDataD* sphMarkersD) {
    uint numThreads, numBlocks;
    computeGridSize(numObjectsH->numAllMarkers, 256, numBlocks, numThreads);

    int haveGhost = (numObjectsH->numGhostMarkers > 0) ? 1 : 0;
    int haveHelper = (numObjectsH->numHelperMarkers > 0) ? 1 : 0;

    int4 updatePortion = mI4(fsiData->fsiGeneralData.referenceArray[haveHelper].x,
                             fsiData->fsiGeneralData.referenceArray[haveHelper + haveGhost].y, 0, 0);
    //    std::cout << "Skipping the markers greater than "
    //              << fsiData->fsiGeneralData.referenceArray[haveHelper + haveGhost].y << " in position update\n";

    std::cout << "time step in UpdateFluid_Implicit " << paramsH->dT << std::endl;
    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    Update_Fluid_State<<<numBlocks, numThreads>>>(
        mR3CAST(fsiData->fsiGeneralData.vel_XSPH_D), mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
        mR4CAST(sphMarkersD->rhoPresMuD), updatePortion, numObjectsH->numAllMarkers, paramsH->dT, isErrorD);
    cudaDeviceSynchronize();
    cudaCheckError();

    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true) {
        throw std::runtime_error("Error! program crashed in  Update_Fluid_State!\n");
    }
    //------------------------------------------------------------------------

    cudaFree(isErrorD);
    free(isErrorH);
}
// -----------------------------------------------------------------------------
/**
 * @brief ApplyBoundarySPH_Markers
 * @details
 * 		applies periodic boundary conditions in x,y, and z directions
 */
void ChFluidDynamics::ApplyBoundarySPH_Markers(SphMarkerDataD* sphMarkersD) {
    uint nBlock_NumSpheres, nThreads_SphMarkers;
    computeGridSize(numObjectsH->numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
    ApplyPeriodicBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(mR4CAST(sphMarkersD->posRadD),
                                                                             mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();
    //    // these are useful anyway for out of bound particles
    ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(mR4CAST(sphMarkersD->posRadD),
                                                                             mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();
    ApplyPeriodicBoundaryZKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(mR4CAST(sphMarkersD->posRadD),
                                                                             mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();
    //    SetOutputPressureToZero_X<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(mR3CAST(posRadD), mR4CAST(rhoPresMuD));
    //    cudaDeviceSynchronize();
    //    cudaCheckError();
}

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------

/**
 * @brief ApplyBoundarySPH_Markers
 * @details
 * 		applies periodic boundary conditions in y, and z. The inlet/outlet BC is applied in the x direction.
 * 		This functions needs to be tested.
 */
void ChFluidDynamics::ApplyModifiedBoundarySPH_Markers(SphMarkerDataD* sphMarkersD) {
    uint nBlock_NumSpheres, nThreads_SphMarkers;
    computeGridSize(numObjectsH->numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
    ApplyInletBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();
    // these are useful anyway for out of bound particles
    ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(mR4CAST(sphMarkersD->posRadD),
                                                                             mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();
    ApplyPeriodicBoundaryZKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(mR4CAST(sphMarkersD->posRadD),
                                                                             mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();
}
// -----------------------------------------------------------------------------

void ChFluidDynamics::DensityReinitialization() {
    uint nBlock_NumSpheres, nThreads_SphMarkers;
    computeGridSize(numObjectsH->numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);

    //    thrust::device_vector<Real4> dummySortedRhoPreMu = fsiData->sortedSphMarkersD.rhoPresMuD;

    thrust::device_vector<Real4> dummySortedRhoPreMu(numObjectsH->numAllMarkers);
    thrust::fill(dummySortedRhoPreMu.begin(), dummySortedRhoPreMu.end(), mR4(0.0));

    ReCalcDensityD_F1<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
        mR4CAST(dummySortedRhoPreMu), mR4CAST(fsiData->sortedSphMarkersD.posRadD),
        mR3CAST(fsiData->sortedSphMarkersD.velMasD), mR4CAST(fsiData->sortedSphMarkersD.rhoPresMuD),
        U1CAST(fsiData->markersProximityD.gridMarkerIndexD), U1CAST(fsiData->markersProximityD.cellStartD),
        U1CAST(fsiData->markersProximityD.cellEndD), numObjectsH->numAllMarkers);

    cudaDeviceSynchronize();
    cudaCheckError();
    ChFsiForce::CopySortedToOriginal_NonInvasive_R4(fsiData->sphMarkersD1.rhoPresMuD, dummySortedRhoPreMu,
                                                    fsiData->markersProximityD.gridMarkerIndexD);
    ChFsiForce::CopySortedToOriginal_NonInvasive_R4(fsiData->sphMarkersD2.rhoPresMuD, dummySortedRhoPreMu,
                                                    fsiData->markersProximityD.gridMarkerIndexD);
    dummySortedRhoPreMu.clear();
}

}  // namespace fsi
}  // end namespace chrono
