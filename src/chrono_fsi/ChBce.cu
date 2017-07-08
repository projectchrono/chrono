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
// Base class for processing boundary condition enforcing (bce) markers forces
// in fsi system.//
// =============================================================================

#include "chrono_fsi/ChBce.cuh" //for FsiGeneralData
#include "chrono_fsi/ChSphGeneral.cuh"

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void
Populate_RigidSPH_MeshPos_LRF_kernel(Real3 *rigidSPH_MeshPos_LRF_D,
                                     Real3 *posRadD, uint *rigidIdentifierD,
                                     Real3 *posRigidD, Real4 *qD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  int numRigid_SphMarkers = numObjectsD.numRigid_SphMarkers;
  if (index >= numObjectsD.numRigid_SphMarkers) {
    return;
  }
  int rigidIndex = rigidIdentifierD[index];
  uint rigidMarkerIndex =
      index + numObjectsD.startRigidMarkers; // updatePortion = [start, end]
                                             // index of the update portion
  Real4 q4 = qD[rigidIndex];
  ;
  Real3 a1, a2, a3;
  RotationMatirixFromQuaternion(a1, a2, a3, q4);
  Real3 dist3 = posRadD[rigidMarkerIndex] - posRigidD[rigidIndex];
  Real3 dist3LF = InverseRotate_By_RotationMatrix_DeviceHost(a1, a2, a3, dist3);
  rigidSPH_MeshPos_LRF_D[index] = dist3LF;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
// Arman : revisit equation 10 of tech report, is it only on fluid or it is on
// all markers
__device__ void
BCE_modification_Share(Real3 &sumVW, Real &sumWAll, Real3 &sumRhoRW,
                       Real &sumPW, Real &sumWFluid, int &isAffectedV,
                       int &isAffectedP, int3 gridPos, Real3 posRadA,
                       Real3 *sortedPosRad, Real3 *sortedVelMas,
                       Real4 *sortedRhoPreMu, uint *cellStart, uint *cellEnd) {
  uint gridHash = calcGridHash(gridPos);
  // get start of bucket for this cell
  uint startIndex = cellStart[gridHash];
  if (startIndex != 0xffffffff) { // cell is not empty
    // iterate over particles in this cell
    uint endIndex = cellEnd[gridHash];

    for (uint j = startIndex; j < endIndex; j++) {
      Real3 posRadB = sortedPosRad[j];
      Real3 dist3 = Distance(posRadA, posRadB);
      Real d = length(dist3);
      Real4 rhoPresMuB = sortedRhoPreMu[j];
      if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML || rhoPresMuB.w > -.1)
        continue;

      Real Wd = W3(d);
      Real WdOvRho = Wd / rhoPresMuB.x;
      isAffectedV = 1;
      Real3 velMasB = sortedVelMas[j];
      sumVW += velMasB * WdOvRho;
      sumWAll += WdOvRho;

      isAffectedP = 1;
      sumRhoRW += rhoPresMuB.x * dist3 * WdOvRho;
      sumPW += rhoPresMuB.y * WdOvRho;
      sumWFluid += WdOvRho;
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void new_BCE_VelocityPressure(
    Real3 *velMas_ModifiedBCE,   // input: sorted velocities
    Real4 *rhoPreMu_ModifiedBCE, // input: sorted velocities
    Real3 *sortedPosRad,         // input: sorted positions
    Real3 *sortedVelMas,         // input: sorted velocities
    Real4 *sortedRhoPreMu, uint *cellStart, uint *cellEnd,
    uint *mapOriginalToSorted, Real3 *bceAcc, int2 updatePortion,
    volatile bool *isErrorD) {
  uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
  uint sphIndex =
      bceIndex +
      updatePortion
          .x; // updatePortion = [start, end] index of the update portion
  if (sphIndex >= updatePortion.y) {
    return;
  }

  uint idA = mapOriginalToSorted[sphIndex];
  Real4 rhoPreMuA = sortedRhoPreMu[idA];
  Real3 posRadA = sortedPosRad[idA];
  Real3 velMasA = sortedVelMas[idA];
  int isAffectedV = 0;
  int isAffectedP = 0;

  Real3 sumVW = mR3(0);
  Real sumWAll = 0;
  Real3 sumRhoRW = mR3(0);
  Real sumPW = 0;
  Real sumWFluid = 0;

  // get address in grid
  int3 gridPos = calcGridPos(posRadA);

  /// if (gridPos.x == paramsD.gridSize.x-1) printf("****aha %d %d\n",
  /// gridPos.x, paramsD.gridSize.x);

  // examine neighbouring cells
  for (int z = -1; z <= 1; z++) {
    for (int y = -1; y <= 1; y++) {
      for (int x = -1; x <= 1; x++) {
        int3 neighbourPos = gridPos + mI3(x, y, z);
        BCE_modification_Share(sumVW, sumWAll, sumRhoRW, sumPW, sumWFluid,
                               isAffectedV, isAffectedP, neighbourPos, posRadA,
                               sortedPosRad, sortedVelMas, sortedRhoPreMu,
                               cellStart, cellEnd);
      }
    }
  }

  if (isAffectedV) {
    Real3 modifiedBCE_v = 2 * velMasA - sumVW / sumWAll;
    velMas_ModifiedBCE[bceIndex] = modifiedBCE_v;
  }
  if (isAffectedP) {
    // pressure
    Real3 a3 = mR3(0);
    if (fabs(rhoPreMuA.w) > 0) { // rigid BCE
      int rigidBceIndex = sphIndex - numObjectsD.startRigidMarkers;
      if (rigidBceIndex < 0 ||
          rigidBceIndex >= numObjectsD.numRigid_SphMarkers) {
        printf("Error! marker index out of bound: thrown from "
               "SDKCollisionSystem.cu, new_BCE_VelocityPressure !\n");
        *isErrorD = true;
        return;
      }
      a3 = bceAcc[rigidBceIndex];
    }
    Real pressure = (sumPW + dot(paramsD.gravity - a3, sumRhoRW)) /
                    sumWFluid; //(in fact:  (paramsD.gravity -
    // aW), but aW for moving rigids
    // is hard to calc. Assume aW is
    // zero for now
    Real density = InvEos(pressure);
    rhoPreMu_ModifiedBCE[bceIndex] =
        mR4(density, pressure, rhoPreMuA.z, rhoPreMuA.w);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------
// calculate marker acceleration, required in ADAMI
__global__ void calcBceAcceleration_kernel(Real3 *bceAcc, Real4 *q_fsiBodies_D,
                                           Real3 *accRigid_fsiBodies_D,
                                           Real3 *omegaVelLRF_fsiBodies_D,
                                           Real3 *omegaAccLRF_fsiBodies_D,
                                           Real3 *rigidSPH_MeshPos_LRF_D,
                                           const uint *rigidIdentifierD) {
  uint bceIndex = blockIdx.x * blockDim.x + threadIdx.x;
  if (bceIndex >= numObjectsD.numRigid_SphMarkers) {
    return;
  }

  int rigidBodyIndex = rigidIdentifierD[bceIndex];
  Real3 acc3 = accRigid_fsiBodies_D[rigidBodyIndex]; // linear acceleration (CM)

  Real4 q4 = q_fsiBodies_D[rigidBodyIndex];
  Real3 a1, a2, a3;
  RotationMatirixFromQuaternion(a1, a2, a3, q4);
  Real3 wVel3 = omegaVelLRF_fsiBodies_D[rigidBodyIndex];
  Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[bceIndex];
  Real3 wVelCrossS = cross(wVel3, rigidSPH_MeshPos_LRF);
  Real3 wVelCrossWVelCrossS = cross(wVel3, wVelCrossS);
  acc3 += dot(a1, wVelCrossWVelCrossS), dot(a2, wVelCrossWVelCrossS),
      dot(a3,
          wVelCrossWVelCrossS); // centrigugal acceleration

  Real3 wAcc3 = omegaAccLRF_fsiBodies_D[rigidBodyIndex];
  Real3 wAccCrossS = cross(wAcc3, rigidSPH_MeshPos_LRF);
  acc3 += dot(a1, wAccCrossS), dot(a2, wAccCrossS),
      dot(a3, wAccCrossS); // tangential acceleration

  //	printf("linear acc %f %f %f point acc %f %f %f \n", accRigid3.x,
  //accRigid3.y, accRigid3.z, acc3.x, acc3.y,
  // acc3.z);
  bceAcc[bceIndex] = acc3;
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the rigid body particles
__global__ void UpdateRigidMarkersPositionVelocityD(
    Real3 *posRadD, Real3 *velMasD, const Real3 *rigidSPH_MeshPos_LRF_D,
    const uint *rigidIdentifierD, Real3 *posRigidD, Real4 *velMassRigidD,
    Real3 *omegaLRF_D, Real4 *qD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= numObjectsD.numRigid_SphMarkers) {
    return;
  }
  uint rigidMarkerIndex =
      index + numObjectsD.startRigidMarkers; // updatePortion = [start, end]
                                             // index of the update portion
  int rigidBodyIndex = rigidIdentifierD[index];

  Real4 q4 = qD[rigidBodyIndex];
  Real3 a1, a2, a3;
  RotationMatirixFromQuaternion(a1, a2, a3, q4);

  Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[index];

  // position
  Real3 p_Rigid = posRigidD[rigidBodyIndex];
  posRadD[rigidMarkerIndex] = p_Rigid + mR3(dot(a1, rigidSPH_MeshPos_LRF),
                                            dot(a2, rigidSPH_MeshPos_LRF),
                                            dot(a3, rigidSPH_MeshPos_LRF));

  // velocity
  Real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
  Real3 omega3 = omegaLRF_D[rigidBodyIndex];
  Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
  velMasD[rigidMarkerIndex] = mR3(vM_Rigid) + dot(a1, omegaCrossS),
  dot(a2, omegaCrossS), dot(a3, omegaCrossS);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Rigid_FSI_ForcesD(Real3 *rigid_FSI_ForcesD,
                                       Real4 *totalSurfaceInteractionRigid4) {
  uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
  if (rigidSphereA >= numObjectsD.numRigidBodies) {
    return;
  }
  Real3 force3 =
      paramsD.markerMass * mR3(totalSurfaceInteractionRigid4[rigidSphereA]);
  rigid_FSI_ForcesD[rigidSphereA] = force3;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Markers_TorquesD(Real3 *torqueMarkersD,
                                      Real4 *derivVelRhoD, Real3 *posRadD,
                                      uint *rigidIdentifierD,
                                      Real3 *posRigidD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= numObjectsD.numRigid_SphMarkers) {
    return;
  }
  uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
  Real3 dist3 =
      Distance(posRadD[rigidMarkerIndex], posRigidD[rigidIdentifierD[index]]);
  torqueMarkersD[index] =
      paramsD.markerMass *
      cross(dist3, mR3(derivVelRhoD[rigidMarkerIndex])); // paramsD.markerMass
                                                         // is multiplied to
                                                         // convert
  // from SPH acceleration to force
}
//--------------------------------------------------------------------------------------------------------------------------------
ChBce::ChBce(SphMarkerDataD *otherSortedSphMarkersD,
             ProximityDataD *otherMarkersProximityD,
             FsiGeneralData *otherFsiGeneralData, SimParams *otherParamsH,
             NumberOfObjects *otherNumObjects)
    : sortedSphMarkersD(otherSortedSphMarkersD),
      markersProximityD(otherMarkersProximityD),
      fsiGeneralData(otherFsiGeneralData), paramsH(otherParamsH),
      numObjectsH(otherNumObjects) {}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::Finalize(SphMarkerDataD *sphMarkersD, FsiBodiesDataD *fsiBodiesD) {
  cudaMemcpyToSymbolAsync(paramsD, paramsH, sizeof(SimParams));
  cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH, sizeof(NumberOfObjects));

  totalSurfaceInteractionRigid4.resize(numObjectsH->numRigidBodies);
  dummyIdentify.resize(numObjectsH->numRigidBodies);
  torqueMarkersD.resize(numObjectsH->numRigid_SphMarkers);

  // Resizing the arrays used to modify the BCE velocity and pressure according
  // to ADAMI
  int numRigidAndBoundaryMarkers =
      fsiGeneralData->referenceArray[2 + numObjectsH->numRigidBodies - 1].y -
      fsiGeneralData->referenceArray[0].y;
  if ((numObjectsH->numBoundaryMarkers + numObjectsH->numRigid_SphMarkers) !=
      numRigidAndBoundaryMarkers) {
    throw std::runtime_error(
        "Error! number of rigid and boundary markers are saved incorrectly!\n");
  }
  velMas_ModifiedBCE.resize(numRigidAndBoundaryMarkers);
  rhoPreMu_ModifiedBCE.resize(numRigidAndBoundaryMarkers);

  // Populate local position of BCE markers
  Populate_RigidSPH_MeshPos_LRF(sphMarkersD, fsiBodiesD);
}
//--------------------------------------------------------------------------------------------------------------------------------
ChBce::~ChBce() {
  // TODO
}

////--------------------------------------------------------------------------------------------------------------------------------
void ChBce::MakeRigidIdentifier() {
  if (numObjectsH->numRigidBodies > 0) {
    for (int rigidSphereA = 0; rigidSphereA < numObjectsH->numRigidBodies;
         rigidSphereA++) {
      int4 referencePart = fsiGeneralData->referenceArray[2 + rigidSphereA];
      if (referencePart.z != 1) {
        printf(" Error! in accessing rigid bodies. Reference array indexing is "
               "wrong\n");
        return;
      }
      int2 updatePortion = mI2(referencePart); // first two component of the
                                               // referenceArray denote to the
                                               // fluid and boundary particles
      thrust::fill(fsiGeneralData->rigidIdentifierD.begin() +
                       (updatePortion.x - numObjectsH->startRigidMarkers),
                   fsiGeneralData->rigidIdentifierD.begin() +
                       (updatePortion.y - numObjectsH->startRigidMarkers),
                   rigidSphereA);
    }
  }
}
////--------------------------------------------------------------------------------------------------------------------------------

void ChBce::Populate_RigidSPH_MeshPos_LRF(SphMarkerDataD *sphMarkersD,
                                          FsiBodiesDataD *fsiBodiesD) {
  if (numObjectsH->numRigidBodies == 0) {
    return;
  }

  MakeRigidIdentifier();

  uint nBlocks_numRigid_SphMarkers;
  uint nThreads_SphMarkers;
  computeGridSize(numObjectsH->numRigid_SphMarkers, 256,
                  nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

  Populate_RigidSPH_MeshPos_LRF_kernel<<<nBlocks_numRigid_SphMarkers,
                                         nThreads_SphMarkers>>>(
      mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D),
      mR3CAST(sphMarkersD->posRadD), U1CAST(fsiGeneralData->rigidIdentifierD),
      mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
      mR4CAST(fsiBodiesD->q_fsiBodies_D));
  cudaThreadSynchronize();
  cudaCheckError();

  UpdateRigidMarkersPositionVelocity(sphMarkersD, fsiBodiesD);
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::RecalcSortedVelocityPressure_BCE(
    thrust::device_vector<Real3> &velMas_ModifiedBCE,
    thrust::device_vector<Real4> &rhoPreMu_ModifiedBCE,
    const thrust::device_vector<Real3> &sortedPosRad,
    const thrust::device_vector<Real3> &sortedVelMas,
    const thrust::device_vector<Real4> &sortedRhoPreMu,
    const thrust::device_vector<uint> &cellStart,
    const thrust::device_vector<uint> &cellEnd,
    const thrust::device_vector<uint> &mapOriginalToSorted,
    const thrust::device_vector<Real3> &bceAcc, int2 updatePortion) {
  bool *isErrorH, *isErrorD;
  isErrorH = (bool *)malloc(sizeof(bool));
  cudaMalloc((void **)&isErrorD, sizeof(bool));
  *isErrorH = false;
  cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
  //------------------------------------------------------------------------

  // thread per particle
  uint numThreads, numBlocks;
  computeGridSize(updatePortion.y - updatePortion.x, 64, numBlocks, numThreads);

  new_BCE_VelocityPressure<<<numBlocks, numThreads>>>(
      mR3CAST(velMas_ModifiedBCE),
      mR4CAST(rhoPreMu_ModifiedBCE), // input: sorted velocities
      mR3CAST(sortedPosRad), mR3CAST(sortedVelMas), mR4CAST(sortedRhoPreMu),
      U1CAST(cellStart), U1CAST(cellEnd), U1CAST(mapOriginalToSorted),
      mR3CAST(bceAcc), updatePortion, isErrorD);

  cudaThreadSynchronize();
  cudaCheckError()

      //------------------------------------------------------------------------
      cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
  if (*isErrorH == true) {
    throw std::runtime_error(
        "Error! program crashed in  new_BCE_VelocityPressure!\n");
  }
  cudaFree(isErrorD);
  free(isErrorH);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::CalcBceAcceleration(
    thrust::device_vector<Real3> &bceAcc,
    const thrust::device_vector<Real4> &q_fsiBodies_D,
    const thrust::device_vector<Real3> &accRigid_fsiBodies_D,
    const thrust::device_vector<Real3> &omegaVelLRF_fsiBodies_D,
    const thrust::device_vector<Real3> &omegaAccLRF_fsiBodies_D,
    const thrust::device_vector<Real3> &rigidSPH_MeshPos_LRF_D,
    const thrust::device_vector<uint> &rigidIdentifierD,
    int numRigid_SphMarkers) {
  // thread per particle
  uint numThreads, numBlocks;
  computeGridSize(numRigid_SphMarkers, 64, numBlocks, numThreads);

  calcBceAcceleration_kernel<<<numBlocks, numThreads>>>(
      mR3CAST(bceAcc), mR4CAST(q_fsiBodies_D), mR3CAST(accRigid_fsiBodies_D),
      mR3CAST(omegaVelLRF_fsiBodies_D), mR3CAST(omegaAccLRF_fsiBodies_D),
      mR3CAST(rigidSPH_MeshPos_LRF_D), U1CAST(rigidIdentifierD));

  cudaThreadSynchronize();
  cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::ModifyBceVelocity(SphMarkerDataD *sphMarkersD,
                              FsiBodiesDataD *fsiBodiesD) {
  // modify BCE velocity and pressure
  int numRigidAndBoundaryMarkers =
      fsiGeneralData->referenceArray[2 + numObjectsH->numRigidBodies - 1].y -
      fsiGeneralData->referenceArray[0].y;
  if ((numObjectsH->numBoundaryMarkers + numObjectsH->numRigid_SphMarkers) !=
      numRigidAndBoundaryMarkers) {
    throw std::runtime_error("Error! number of rigid and boundary markers are "
                             "saved incorrectly. Thrown from "
                             "ModifyBceVelocity!\n");
  }
  if (!(velMas_ModifiedBCE.size() == numRigidAndBoundaryMarkers &&
        rhoPreMu_ModifiedBCE.size() == numRigidAndBoundaryMarkers)) {
    throw std::runtime_error("Error! size error velMas_ModifiedBCE and "
                             "rhoPreMu_ModifiedBCE. Thrown from "
                             "ModifyBceVelocity!\n");
  }
  int2 updatePortion = mI2(
      fsiGeneralData->referenceArray[0].y,
      fsiGeneralData->referenceArray[2 + numObjectsH->numRigidBodies - 1].y);
  if (paramsH->bceType == ADAMI) {
    thrust::device_vector<Real3> bceAcc(numObjectsH->numRigid_SphMarkers);
    if (numObjectsH->numRigid_SphMarkers > 0) {
      CalcBceAcceleration(
          bceAcc, fsiBodiesD->q_fsiBodies_D, fsiBodiesD->accRigid_fsiBodies_D,
          fsiBodiesD->omegaVelLRF_fsiBodies_D,
          fsiBodiesD->omegaAccLRF_fsiBodies_D,
          fsiGeneralData->rigidSPH_MeshPos_LRF_D,
          fsiGeneralData->rigidIdentifierD, numObjectsH->numRigid_SphMarkers);
    }
    RecalcSortedVelocityPressure_BCE(
        velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, sortedSphMarkersD->posRadD,
        sortedSphMarkersD->velMasD, sortedSphMarkersD->rhoPresMuD,
        markersProximityD->cellStartD, markersProximityD->cellEndD,
        markersProximityD->mapOriginalToSorted, bceAcc, updatePortion);
    bceAcc.clear();
  } else {
    thrust::copy(sphMarkersD->velMasD.begin() + updatePortion.x,
                 sphMarkersD->velMasD.begin() + updatePortion.y,
                 velMas_ModifiedBCE.begin());
    thrust::copy(sphMarkersD->rhoPresMuD.begin() + updatePortion.x,
                 sphMarkersD->rhoPresMuD.begin() + updatePortion.y,
                 rhoPreMu_ModifiedBCE.begin());
  }
}
//--------------------------------------------------------------------------------------------------------------------------------
// applies the time step to the current quantities and saves the new values into
// variable with the same name and '2' and
// the end
// precondition: for the first step of RK2, all variables with '2' at the end
// have the values the same as those without
// '2' at the end.
void ChBce::Rigid_Forces_Torques(SphMarkerDataD *sphMarkersD,
                                 FsiBodiesDataD *fsiBodiesD) {
  // Arman: InitSystem has to be called before this point to set the number of
  // objects

  if (numObjectsH->numRigidBodies == 0) {
    return;
  }
  //################################################### make force and torque
  //arrays
  //####### Force (Acceleration)
  if (totalSurfaceInteractionRigid4.size() != numObjectsH->numRigidBodies ||
      dummyIdentify.size() != numObjectsH->numRigidBodies ||
      torqueMarkersD.size() != numObjectsH->numRigid_SphMarkers) {
    throw std::runtime_error("Error! wrong size: totalSurfaceInteractionRigid4 "
                             "or torqueMarkersD or dummyIdentify. Thrown from "
                             "Rigid_Forces_Torques!\n");
  }

  thrust::fill(totalSurfaceInteractionRigid4.begin(),
               totalSurfaceInteractionRigid4.end(), mR4(0));
  thrust::fill(torqueMarkersD.begin(), torqueMarkersD.end(), mR3(0));

  thrust::equal_to<uint> binary_pred;

  //** forces on BCE markers of each rigid body are accumulated at center.
  //"totalSurfaceInteractionRigid4" is got built.
  (void)thrust::reduce_by_key(
      fsiGeneralData->rigidIdentifierD.begin(),
      fsiGeneralData->rigidIdentifierD.end(),
      fsiGeneralData->derivVelRhoD.begin() + numObjectsH->startRigidMarkers,
      dummyIdentify.begin(), totalSurfaceInteractionRigid4.begin(), binary_pred,
      thrust::plus<Real4>());
  thrust::fill(fsiGeneralData->rigid_FSI_ForcesD.begin(),
               fsiGeneralData->rigid_FSI_ForcesD.end(), mR3(0));

  uint nBlock_UpdateRigid;
  uint nThreads_rigidParticles;
  computeGridSize(numObjectsH->numRigidBodies, 128, nBlock_UpdateRigid,
                  nThreads_rigidParticles);

  //** accumulated BCE forces at center are transformed to acceleration of rigid
  //body "rigid_FSI_ForcesD".
  //"rigid_FSI_ForcesD" gets built.
  Calc_Rigid_FSI_ForcesD<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(
      mR3CAST(fsiGeneralData->rigid_FSI_ForcesD),
      mR4CAST(totalSurfaceInteractionRigid4));
  cudaThreadSynchronize();
  cudaCheckError();

  //####### Torque
  uint nBlocks_numRigid_SphMarkers;
  uint nThreads_SphMarkers;
  computeGridSize(numObjectsH->numRigid_SphMarkers, 256,
                  nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

  //** the current position of the rigid, 'posRigidD', is used to calculate the
  //moment of BCE acceleration at the rigid
  //*** body center (i.e. torque/mass). "torqueMarkersD" gets built.
  Calc_Markers_TorquesD<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(
      mR3CAST(torqueMarkersD), mR4CAST(fsiGeneralData->derivVelRhoD),
      mR3CAST(sphMarkersD->posRadD), U1CAST(fsiGeneralData->rigidIdentifierD),
      mR3CAST(fsiBodiesD->posRigid_fsiBodies_D));
  cudaThreadSynchronize();
  cudaCheckError();

  (void)thrust::reduce_by_key(fsiGeneralData->rigidIdentifierD.begin(),
                              fsiGeneralData->rigidIdentifierD.end(),
                              torqueMarkersD.begin(), dummyIdentify.begin(),
                              fsiGeneralData->rigid_FSI_TorquesD.begin(),
                              binary_pred, thrust::plus<Real3>());
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::UpdateRigidMarkersPositionVelocity(SphMarkerDataD *sphMarkersD,
                                               FsiBodiesDataD *fsiBodiesD) {
  if (numObjectsH->numRigidBodies == 0) {
    return;
  }

  uint nBlocks_numRigid_SphMarkers;
  uint nThreads_SphMarkers;
  computeGridSize(numObjectsH->numRigid_SphMarkers, 256,
                  nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

  // Arman: InitSystem has to be called before this lunch to set numObjectsD

  //################################################### update BCE markers
  //position
  //** "posRadD2"/"velMasD2" associated to BCE markers are updated based on new
  //rigid body (position,
  // orientation)/(velocity, angular velocity)
  UpdateRigidMarkersPositionVelocityD<<<nBlocks_numRigid_SphMarkers,
                                        nThreads_SphMarkers>>>(
      mR3CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
      mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D),
      U1CAST(fsiGeneralData->rigidIdentifierD),
      mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
      mR4CAST(fsiBodiesD->velMassRigid_fsiBodies_D),
      mR3CAST(fsiBodiesD->omegaVelLRF_fsiBodies_D),
      mR4CAST(fsiBodiesD->q_fsiBodies_D));
  cudaThreadSynchronize();
  cudaCheckError();
}

} // end namespace fsi
} // end namespace chrono
