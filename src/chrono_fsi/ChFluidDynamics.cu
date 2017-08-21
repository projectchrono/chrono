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
// Author: Arman Pazouki
// =============================================================================
//
// Class for performing time integration in fluid system.//
// =============================================================================

#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChFluidDynamics.cuh"
#include "chrono_fsi/ChSphGeneral.cuh"

namespace chrono {
namespace fsi {

// -----------------------------------------------------------------------------
/// Device function to calculate the share of density influence on a given
/// marker from
/// all other markers in a given cell

__device__ void collideCellDensityReInit(Real &densityShare, Real &denominator,
                                         int3 gridPos, uint index,
                                         Real3 posRadA, Real3 *sortedPosRad,
                                         Real3 *sortedVelMas,
                                         Real4 *sortedRhoPreMu, uint *cellStart,
                                         uint *cellEnd) {
  //?c2 printf("grid pos %d %d %d \n", gridPos.x, gridPos.y, gridPos.z);
  uint gridHash = calcGridHash(gridPos);
  // get start of bucket for this cell
  Real densityShare2 = 0.0f;
  Real denominator2 = 0.0f;

  uint startIndex = cellStart[gridHash];
  if (startIndex != 0xffffffff) { // cell is not empty
    // iterate over particles in this cell
    uint endIndex = cellEnd[gridHash];

    for (uint j = startIndex; j < endIndex; j++) {
      if (j != index) { // check not colliding with self
        Real3 posRadB = sortedPosRad[j];
        Real4 rhoPreMuB = sortedRhoPreMu[j];
        Real3 dist3 = Distance(posRadA, posRadB);
        Real d = length(dist3);
        if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
          continue;
        Real partialDensity = paramsD.markerMass * W3(d); // optimize it ?$
        densityShare2 += partialDensity;
        denominator2 += partialDensity / rhoPreMuB.x;
        // if (fabs(W3(d)) < .00000001) {printf("good evening, distance %f %f
        // %f\n", dist3.x, dist3.y, dist3.z);
        // printf("posRadA %f %f %f, posRadB, %f %f %f\n", posRadA.x, posRadA.y,
        // posRadA.z, posRadB.x, posRadB.y,
        // posRadB.z);
        //}
      }
    }
  }
  densityShare += densityShare2;
  denominator += denominator2;
}

// -----------------------------------------------------------------------------
/// Kernel to apply periodic BC along x

__global__ void ApplyPeriodicBoundaryXKernel(Real3 *posRadD,
                                             Real4 *rhoPresMuD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= numObjectsD.numAllMarkers) {
    return;
  }
  Real4 rhoPresMu = rhoPresMuD[index];
  if (fabs(rhoPresMu.w) < .1) {
    return;
  } // no need to do anything if it is a boundary particle
  Real3 posRad = posRadD[index];
  if (posRad.x > paramsD.cMax.x) {
    posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
    posRadD[index] = posRad;
    if (rhoPresMu.w < -.1) {
      rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.x;
      rhoPresMuD[index] = rhoPresMu;
    }
    return;
  }
  if (posRad.x < paramsD.cMin.x) {
    posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
    posRadD[index] = posRad;
    if (rhoPresMu.w < -.1) {
      rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.x;
      rhoPresMuD[index] = rhoPresMu;
    }
    return;
  }
}

// -----------------------------------------------------------------------------
/// Kernel to apply periodic BC along y

__global__ void ApplyPeriodicBoundaryYKernel(Real3 *posRadD,
                                             Real4 *rhoPresMuD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= numObjectsD.numAllMarkers) {
    return;
  }
  Real4 rhoPresMu = rhoPresMuD[index];
  if (fabs(rhoPresMu.w) < .1) {
    return;
  } // no need to do anything if it is a boundary particle
  Real3 posRad = posRadD[index];
  if (posRad.y > paramsD.cMax.y) {
    posRad.y -= (paramsD.cMax.y - paramsD.cMin.y);
    posRadD[index] = posRad;
    if (rhoPresMu.w < -.1) {
      rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.y;
      rhoPresMuD[index] = rhoPresMu;
    }
    return;
  }
  if (posRad.y < paramsD.cMin.y) {
    posRad.y += (paramsD.cMax.y - paramsD.cMin.y);
    posRadD[index] = posRad;
    if (rhoPresMu.w < -.1) {
      rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.y;
      rhoPresMuD[index] = rhoPresMu;
    }
    return;
  }
}

// -----------------------------------------------------------------------------
/// Kernel to apply periodic BC along z

__global__ void ApplyPeriodicBoundaryZKernel(Real3 *posRadD,
                                             Real4 *rhoPresMuD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= numObjectsD.numAllMarkers) {
    return;
  }
  Real4 rhoPresMu = rhoPresMuD[index];
  if (fabs(rhoPresMu.w) < .1) {
    return;
  } // no need to do anything if it is a boundary particle
  Real3 posRad = posRadD[index];
  if (posRad.z > paramsD.cMax.z) {
    posRad.z -= (paramsD.cMax.z - paramsD.cMin.z);
    posRadD[index] = posRad;
    if (rhoPresMu.w < -.1) {
      rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.z;
      rhoPresMuD[index] = rhoPresMu;
    }
    return;
  }
  if (posRad.z < paramsD.cMin.z) {
    posRad.z += (paramsD.cMax.z - paramsD.cMin.z);
    posRadD[index] = posRad;
    if (rhoPresMu.w < -.1) {
      rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.z;
      rhoPresMuD[index] = rhoPresMu;
    }
    return;
  }
}
// -----------------------------------------------------------------------------
/// Kernel to update the fluid properities.
///
/// It updates the density, velocity and position relying on explicit Euler
/// scheme.
/// Pressure is obtained from the density and an Equation of State.
__global__ void UpdateFluidD(Real3 *posRadD, Real3 *velMasD, Real3 *vel_XSPH_D,
                             Real4 *rhoPresMuD, Real4 *derivVelRhoD,
                             int2 updatePortion, Real dT,
                             volatile bool *isErrorD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  index += updatePortion
               .x; // updatePortion = [start, end] index of the update portion
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
    // 0** if you have rigid BCE, make sure to apply same tweaks to them, to
    // satify action/reaction. Or apply tweak to
    // force in advance
    // 1*** let's tweak a little bit :)
    if (!(isfinite(vel_XSPH.x) && isfinite(vel_XSPH.y) &&
          isfinite(vel_XSPH.z))) {
      if (paramsD.enableAggressiveTweak) {
        vel_XSPH = mR3(0);
      } else {
        printf("Error! particle vel_XSPH is NAN: thrown from "
               "SDKCollisionSystem.cu, UpdateFluidDKerner !\n");
        *isErrorD = true;
        return;
      }
    }
    if (length(vel_XSPH) > paramsD.tweakMultV * paramsD.HSML / paramsD.dT &&
        paramsD.enableTweak) {
      vel_XSPH *=
          (paramsD.tweakMultV * paramsD.HSML / paramsD.dT) / length(vel_XSPH);
    }
    // 1*** end tweak

    Real3 posRad = posRadD[index];
    Real3 updatedPositon = posRad + vel_XSPH * dT;
    if (!(isfinite(updatedPositon.x) && isfinite(updatedPositon.y) &&
          isfinite(updatedPositon.z))) {
      printf("Error! particle position is NAN: thrown from "
             "SDKCollisionSystem.cu, UpdateFluidDKernel !\n");
      *isErrorD = true;
      return;
    }
    posRadD[index] = updatedPositon; // posRadD updated

    //-------------
    // ** velocity
    //-------------

    Real3 velMas = velMasD[index];
    Real3 updatedVelocity = velMas + mR3(derivVelRho) * dT;

    if (!(isfinite(updatedVelocity.x) && isfinite(updatedVelocity.y) &&
          isfinite(updatedVelocity.z))) {
      if (paramsD.enableAggressiveTweak) {
        updatedVelocity = mR3(0);
      } else {
        printf("Error! particle updatedVelocity is NAN: thrown from "
               "SDKCollisionSystem.cu, UpdateFluidDKernel !\n");
        *isErrorD = true;
        return;
      }
    }
    // 2*** let's tweak a little bit :)
    if (length(updatedVelocity) >
            paramsD.tweakMultV * paramsD.HSML / paramsD.dT &&
        paramsD.enableTweak) {
      updatedVelocity *= (paramsD.tweakMultV * paramsD.HSML / paramsD.dT) /
                         length(updatedVelocity);
    }
    // 2*** end tweak

    velMasD[index] = updatedVelocity;
  }
  // 3*** let's tweak a littlChTimeIntegrateFsi();e bit :)
  if (!(isfinite(derivVelRho.w))) {
    if (paramsD.enableAggressiveTweak) {
      derivVelRho.w = 0;
    } else {
      printf("Error! particle derivVelRho.w is NAN: thrown from "
             "SDKCollisionSystem.cu, UpdateFluidDKernel !\n");
      *isErrorD = true;
      return;
    }
  }
  if (fabs(derivVelRho.w) > paramsD.tweakMultRho * paramsD.rho0 / paramsD.dT &&
      paramsD.enableTweak) {
    derivVelRho.w *= (paramsD.tweakMultRho * paramsD.rho0 / paramsD.dT) /
                     fabs(derivVelRho.w); // to take care of the sign as well
  }
  // 2*** end tweak
  Real rho2 = rhoPresMu.x + derivVelRho.w * dT; // rho update. (i.e.
                                                // rhoPresMu.x), still not
                                                // wriiten to global matrix
  rhoPresMu.y = Eos(rho2, rhoPresMu.w);
  rhoPresMu.x = rho2;
  if (!(isfinite(rhoPresMu.x) && isfinite(rhoPresMu.y) &&
        isfinite(rhoPresMu.z) && isfinite(rhoPresMu.w))) {
    printf("Error! particle rho pressure is NAN: thrown from "
           "SDKCollisionSystem.cu, UpdateFluidDKernel !\n");
    *isErrorD = true;
    return;
  }
  rhoPresMuD[index] = rhoPresMu; // rhoPresMuD updated
}

// -----------------------------------------------------------------------------
/// Kernel for updating the density.
///
/// It calculates the density of the markers. It does include the normalization
/// close to the boundaries and free surface.
__global__ void ReCalcDensityD_F1(Real4 *dummySortedRhoPreMu,
                                  Real3 *sortedPosRad, Real3 *sortedVelMas,
                                  Real4 *sortedRhoPreMu, uint *gridMarkerIndex,
                                  uint *cellStart, uint *cellEnd,
                                  uint numAllMarkers) {
  uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
  if (index >= numAllMarkers)
    return;

  // read particle data from sorted arrays
  Real3 posRadA = sortedPosRad[index];
  Real4 rhoPreMuA = sortedRhoPreMu[index];

  if (rhoPreMuA.w > -.1)
    return;

  // get address in grid
  int3 gridPos = calcGridPos(posRadA);

  Real densityShare = 0.0f;
  Real denominator = 0.0f;
  // examine neighbouring cells
  for (int z = -1; z <= 1; z++) {
    for (int y = -1; y <= 1; y++) {
      for (int x = -1; x <= 1; x++) {
        int3 neighbourPos = gridPos + mI3(x, y, z);
        collideCellDensityReInit(densityShare, denominator, neighbourPos, index,
                                 posRadA, sortedPosRad, sortedVelMas,
                                 sortedRhoPreMu, cellStart, cellEnd);
      }
    }
  }
  // write new velocity back to original unsorted location

  Real newDensity =
      densityShare +
      paramsD.markerMass *
          W3(0); //?$ include the particle in its summation as well
  Real newDenominator = denominator + paramsD.markerMass * W3(0) / rhoPreMuA.x;
  if (rhoPreMuA.w < 0) {
    //		rhoPreMuA.x = newDensity; // old version
    rhoPreMuA.x = newDensity / newDenominator; // correct version
  }
  rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
  dummySortedRhoPreMu[index] = rhoPreMuA;
}

// -----------------------------------------------------------------------------
// CLASS FOR FLUID DYNAMICS SYSTEM
// -----------------------------------------------------------------------------

ChFluidDynamics::ChFluidDynamics(ChBce *otherBceWorker,
                                 ChFsiDataManager *otherFsiData,
                                 SimParams *otherParamsH,
                                 NumberOfObjects *otherNumObjects)
    : fsiData(otherFsiData), paramsH(otherParamsH),
      numObjectsH(otherNumObjects) {
  forceSystem =
      new ChFsiForceParallel(otherBceWorker, &(fsiData->sortedSphMarkersD),
                             &(fsiData->markersProximityD),
                             &(fsiData->fsiGeneralData), paramsH, numObjectsH);
}

// -----------------------------------------------------------------------------

void ChFluidDynamics::Finalize() {
  cudaMemcpyToSymbolAsync(paramsD, paramsH, sizeof(SimParams));
  cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH, sizeof(NumberOfObjects));
  forceSystem->Finalize();
}

// -----------------------------------------------------------------------------

ChFluidDynamics::~ChFluidDynamics() { delete forceSystem; }

// -----------------------------------------------------------------------------

void ChFluidDynamics::IntegrateSPH(SphMarkerDataD *sphMarkersD2,
                                   SphMarkerDataD *sphMarkersD1,
                                   FsiBodiesDataD *fsiBodiesD1, Real dT) {
  forceSystem->ForceSPH(sphMarkersD1, fsiBodiesD1);
  this->UpdateFluid(sphMarkersD2, dT);
  this->ApplyBoundarySPH_Markers(sphMarkersD2);
}

// -----------------------------------------------------------------------------

void ChFluidDynamics::UpdateFluid(SphMarkerDataD *sphMarkersD, Real dT) {
  //	int4 referencePortion = referenceArray[0];
  //	if (referennamespace chrono {
  //		printf("error in UpdateFluid, accessing non fluid\n");
  //		return;
  //	}
  //	int2 updatePortion = mI2(referencePortion);
  int2 updatePortion = mI2(
      0, fsiData->fsiGeneralData
             .referenceArray[fsiData->fsiGeneralData.referenceArray.size() - 1]
             .y);
  // int2 updatePortion = mI2(referenceArray[0].x, referenceArray[0].y);

  bool *isErrorH, *isErrorD;
  isErrorH = (bool *)malloc(sizeof(bool));
  cudaMalloc((void **)&isErrorD, sizeof(bool));
  *isErrorH = false;
  cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
  //------------------------
  uint nBlock_UpdateFluid, nThreads;
  computeGridSize(updatePortion.y - updatePortion.x, 128, nBlock_UpdateFluid,
                  nThreads);
  UpdateFluidD<<<nBlock_UpdateFluid, nThreads>>>(
      mR3CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
      mR3CAST(fsiData->fsiGeneralData.vel_XSPH_D),
      mR4CAST(sphMarkersD->rhoPresMuD),
      mR4CAST(fsiData->fsiGeneralData.derivVelRhoD), updatePortion, dT,
      isErrorD);
  cudaThreadSynchronize();
  cudaCheckError();
  //------------------------
  cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
  if (*isErrorH == true) {
    throw std::runtime_error("Error! program crashed in  UpdateFluidD!\n");
  }
  cudaFree(isErrorD);
  free(isErrorH);
}

// -----------------------------------------------------------------------------

/**
 * @brief ApplyBoundarySPH_Markers
 * @details
 * 		See SDKCollisionSystem.cuh for more info
 */
void ChFluidDynamics::ApplyBoundarySPH_Markers(SphMarkerDataD *sphMarkersD) {
  uint nBlock_NumSpheres, nThreads_SphMarkers;
  computeGridSize(numObjectsH->numAllMarkers, 256, nBlock_NumSpheres,
                  nThreads_SphMarkers);
  ApplyPeriodicBoundaryXKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
      mR3CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD));
  cudaThreadSynchronize();
  cudaCheckError();
  // these are useful anyway for out of bound particles
  ApplyPeriodicBoundaryYKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
      mR3CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD));
  cudaThreadSynchronize();
  cudaCheckError();
  ApplyPeriodicBoundaryZKernel<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
      mR3CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD));
  cudaThreadSynchronize();
  cudaCheckError();

  //	SetOutputPressureToZero_X<<<nBlock_NumSpheres,
  //nThreads_SphMarkers>>>(mR3CAST(posRadD), mR4CAST(rhoPresMuD));
  //    cudaThreadSynchronize();
  //    cudaCheckError();
}

// -----------------------------------------------------------------------------

void ChFluidDynamics::DensityReinitialization() {
  uint nBlock_NumSpheres, nThreads_SphMarkers;
  computeGridSize(numObjectsH->numAllMarkers, 256, nBlock_NumSpheres,
                  nThreads_SphMarkers);

  thrust::device_vector<Real4> dummySortedRhoPreMu =
      fsiData->sortedSphMarkersD.rhoPresMuD;
  ReCalcDensityD_F1<<<nBlock_NumSpheres, nThreads_SphMarkers>>>(
      mR4CAST(dummySortedRhoPreMu), mR3CAST(fsiData->sortedSphMarkersD.posRadD),
      mR3CAST(fsiData->sortedSphMarkersD.velMasD),
      mR4CAST(fsiData->sortedSphMarkersD.rhoPresMuD),

      U1CAST(fsiData->markersProximityD.gridMarkerIndexD),
      U1CAST(fsiData->markersProximityD.cellStartD),
      U1CAST(fsiData->markersProximityD.cellEndD), numObjectsH->numAllMarkers);

  cudaThreadSynchronize();
  cudaCheckError();
  ChFsiForceParallel::CopySortedToOriginal_Invasive_R4(
      fsiData->sphMarkersD1.rhoPresMuD, dummySortedRhoPreMu,
      fsiData->markersProximityD.gridMarkerIndexD);
  dummySortedRhoPreMu.clear();
}

} // end namespace fsi
} // end namespace chrono
