/* C/C++ Standard library */
///////////////////////////////////////////////////////////////////////////////
//	collideSphereSphere.cu
//	Implements kernels and functions for fluid force calculation and update, rigids, and bce
//
//	Created by Arman Pazouki

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <fstream>

/* Thrust library*/
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/reduce.h>
#include <thrust/extrema.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

/* Chrono::FSI library*/
#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/SDKCollisionSystem.cuh"
#include "chrono_fsi/collideSphereSphere.cuh"
#include "chrono_fsi/printToFile.cuh"

using namespace std;
//#####################################################################################
#define B_SIZE 128
//#####################################################################################
__constant__ int2 updatePortionD;
__constant__ Real solid_SPH_massD;

//--------------------------------------------------------------------------------------------------------------------------------
// first comp of q is rotation, last 3 components are axis of rot
__device__ __host__ inline void RotationMatirixFromQuaternion(Real3& AD1, Real3& AD2, Real3& AD3, const Real4& q) {
  AD1 = 2 * mR3(0.5f - q.z * q.z - q.w * q.w, q.y * q.z - q.x * q.w, q.y * q.w + q.x * q.z);
  AD2 = 2 * mR3(q.y * q.z + q.x * q.w, 0.5f - q.y * q.y - q.w * q.w, q.z * q.w - q.x * q.y);
  AD3 = 2 * mR3(q.y * q.w - q.x * q.z, q.z * q.w + q.x * q.y, 0.5f - q.y * q.y - q.z * q.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
__device__ __host__ inline Real3 InverseRotate_By_RotationMatrix_DeviceHost(const Real3& A1,
                                                                            const Real3& A2,
                                                                            const Real3& A3,
                                                                            const Real3& r3) {
  return mR3(A1.x * r3.x + A2.x * r3.y + A3.x * r3.z,
             A1.y * r3.x + A2.y * r3.y + A3.y * r3.z,
             A1.z * r3.x + A2.z * r3.y + A3.z * r3.z);
}
//--------------------------------------------------------------------------------------------------------------------------------
void MapSPH_ToGrid(Real resolution,
                   int3& cartesianGridDims,
                   thrust::host_vector<Real4>& rho_Pres_CartH,
                   thrust::host_vector<Real4>& vel_VelMag_CartH,
                   thrust::device_vector<Real3>& posRadD,
                   thrust::device_vector<Real4>& velMasD,
                   thrust::device_vector<Real4>& rhoPresMuD,
                   int numAllMarkers,
                   SimParams paramsH) {
  //	Real3* m_dSortedPosRad;
  //	Real4* m_dSortedVelMas;
  //	Real4* m_dSortedRhoPreMu;
  //	uint* m_dCellStart; // index of start of each cell in sorted list
  //	uint* m_dCellEnd; // index of end of cell

  int3 SIDE = paramsH.gridSize;
  uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z;  // m_gridSize = SIDE
  // TODO here

  // calculate grid hash
  thrust::device_vector<Real3> m_dSortedPosRad(numAllMarkers);
  thrust::device_vector<Real4> m_dSortedVelMas(numAllMarkers);
  thrust::device_vector<Real4> m_dSortedRhoPreMu(numAllMarkers);

  thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
  thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

  thrust::device_vector<uint> mapOriginalToSorted(numAllMarkers);

  thrust::device_vector<uint> m_dCellStart(m_numGridCells);
  thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

  // calculate grid hash
  calcHash(m_dGridMarkerHash, m_dGridMarkerIndex, posRadD, numAllMarkers);

  thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());

  // reorder particle arrays into sorted order and find start and end of each cell
  reorderDataAndFindCellStart(m_dCellStart,
                              m_dCellEnd,
                              m_dSortedPosRad,
                              m_dSortedVelMas,
                              m_dSortedRhoPreMu,
                              m_dGridMarkerHash,
                              m_dGridMarkerIndex,
                              mapOriginalToSorted,
                              posRadD,
                              velMasD,
                              rhoPresMuD,
                              numAllMarkers,
                              m_numGridCells);

  // Real resolution = 8 * paramsH.markerRadius;
  cartesianGridDims = mI3(paramsH.boxDims / resolution) + mI3(1);
  //	printf("^^^ bodDim %f %f %f, GridDim %d %d %d, resolution %f \n", paramsH.boxDims.x, paramsH.boxDims.y,
  // paramsH.boxDims.z, cartesianGridDims.x,
  //			cartesianGridDims.y, cartesianGridDims.z, resolution);
  uint cartesianGridSize = cartesianGridDims.x * cartesianGridDims.y * cartesianGridDims.z;
  thrust::device_vector<Real4> rho_Pres_CartD(cartesianGridSize);
  thrust::device_vector<Real4> vel_VelMag_CartD(cartesianGridSize);

  CalcCartesianData(rho_Pres_CartD,
                    vel_VelMag_CartD,
                    m_dSortedPosRad,
                    m_dSortedVelMas,
                    m_dSortedRhoPreMu,
                    m_dGridMarkerIndex,
                    m_dCellStart,
                    m_dCellEnd,
                    cartesianGridSize,
                    cartesianGridDims,
                    resolution);

  //	freeArray(m_dSortedPosRad);
  //	freeArray(m_dSortedVelMas);
  //	freeArray(m_dSortedRhoPreMu);
  m_dSortedPosRad.clear();
  m_dSortedVelMas.clear();

  m_dSortedRhoPreMu.clear();

  m_dGridMarkerHash.clear();
  m_dGridMarkerIndex.clear();
  mapOriginalToSorted.clear();

  //	freeArray(m_dCellStart);
  //	freeArray(m_dCellEnd);
  m_dCellStart.clear();
  m_dCellEnd.clear();

  rho_Pres_CartH.resize(cartesianGridSize);
  vel_VelMag_CartH.resize(cartesianGridSize);
  thrust::copy(rho_Pres_CartD.begin(), rho_Pres_CartD.end(), rho_Pres_CartH.begin());
  thrust::copy(vel_VelMag_CartD.begin(), vel_VelMag_CartD.end(), vel_VelMag_CartH.begin());

  rho_Pres_CartD.clear();
  vel_VelMag_CartD.clear();
}
/**
 * @brief Calculates the force on each particles. See collideSphereSphere.cuh for more info.
 * @details See collideSphereSphere.cuh for more info
 */
void ForceSPH(thrust::device_vector<Real3>& posRadD,
              thrust::device_vector<Real4>& velMasD,
              thrust::device_vector<Real3>& vel_XSPH_D,
              thrust::device_vector<Real4>& rhoPresMuD,
              thrust::device_vector<uint>& bodyIndexD,
              thrust::device_vector<Real4>& derivVelRhoD,
              const thrust::host_vector<int3>& referenceArray,
              const NumberOfObjects& numObjects,
              SimParams paramsH,
              BceVersion bceType,
              Real dT) {
  // Part1: contact detection
  // #########################################################################################################################
  // grid data for sorting method
  //	Real3* m_dSortedPosRad;
  //	Real4* m_dSortedVelMas;
  //	Real4* m_dSortedRhoPreMu;
  //	uint* m_dCellStart; // index of start of each cell in sorted list
  //	uint* m_dCellEnd; // index of end of cell

  /* Part 1: Sorting - Sort using grid data, this will accelerate contact detection. */

  /* Calculate total number of cells in the domain. */
  uint m_numGridCells = paramsH.gridSize.x * paramsH.gridSize.y * paramsH.gridSize.z;  // m_gridSize = SIDE
  /* Total number of markers (fluid + boundary) */
  int numAllMarkers = numObjects.numAllMarkers;
  /* Allocate space for each vector */
  /* Store positions of each particle in the device memory */
  thrust::device_vector<Real3> m_dSortedPosRad(numAllMarkers);
  /* Store velocities of each particle in the device memory */
  thrust::device_vector<Real4> m_dSortedVelMas(numAllMarkers);
  /* Store Rho, Pressure, Mu of each particle in the device memory */
  thrust::device_vector<Real4> m_dSortedRhoPreMu(numAllMarkers);
  /* Store XSPH velocities of each particle in the device memory */
  thrust::device_vector<Real3> vel_XSPH_Sorted_D(numAllMarkers);
  /* Store Hash for each particle */
  thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
  /* Store index for each particle */
  thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);
  /* Store mapOriginalToSorted[originalIndex] = sortedIndex */
  thrust::device_vector<uint> mapOriginalToSorted(numAllMarkers);
  /* Index of start cell in sorted list */
  thrust::device_vector<uint> m_dCellStart(m_numGridCells);
  /* Index of end cell in sorted list */
  thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

  /* Calculate grid hash */
  calcHash(m_dGridMarkerHash, m_dGridMarkerIndex, posRadD, numAllMarkers);

  //	GpuTimer myT0;
  //	myT0.Start();
  /* Sort by hash key. Hash is associated to location. The following line sorts m_dGridMarkerHash
   * in ascending order and using the same permutations it used to sort m_dGridMarkerHash it    * also sorts
   * m_dGridMarkerIndex.
   */
  thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());
  //	myT0.Stop();
  //	Real t0 = (Real)myT0.Elapsed();
  //	printf("(0) ** Sort by key timer %f, array size %d\n", t0, m_dGridMarkerHash.size());

  /* Reorder particle arrays into sorted order given by m_dGridMarkerIndex and find start and
   * end of each bin in the hash array.
   */
  reorderDataAndFindCellStart(m_dCellStart,
                              m_dCellEnd,
                              m_dSortedPosRad,
                              m_dSortedVelMas,
                              m_dSortedRhoPreMu,
                              m_dGridMarkerHash,
                              m_dGridMarkerIndex,
                              mapOriginalToSorted,
                              posRadD,
                              velMasD,
                              rhoPresMuD,
                              numAllMarkers,
                              m_numGridCells);

  // modify BCE velocity and pressure
  if (bceType == ADAMI) {
    RecalcSortedVelocityPressure_BCE(
        m_dSortedVelMas, m_dSortedRhoPreMu, m_dSortedPosRad, m_dCellStart, m_dCellEnd, numAllMarkers);
  }

  /* Part 2: Collision Detection */
  /* Process collisions */

  //	Real3 totalFluidBodyForce3 = paramsH.bodyForce3 + paramsH.gravity;
  /* Add outside forces. Don't add gravity, it is added in ChSystem */
  Real3 totalFluidBodyForce3 = paramsH.bodyForce3;  // gravity is added in ChSystem
  /* Initialize derivVelRhoD with zero. NECESSARY. */
  thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), mR4(0));
  //	GpuTimer myT1;
  //	myT1.Start();
  /* Add body force to fluid particles. Skip boundary particles that are in the bodies ???????*/
  thrust::fill(derivVelRhoD.begin() + referenceArray[0].x,
               derivVelRhoD.begin() + referenceArray[0].y,
               mR4(totalFluidBodyForce3));  // add body force to fluid particles.
  //	myT1.Stop();
  //	Real t1 = (Real)myT1.Elapsed();
  //	printf("(1) *** fill timer %f, array size %d\n", t1, referenceArray[0].y - referenceArray[0].x);
  /* Calculate vel_XSPH */
  RecalcVelocity_XSPH(vel_XSPH_Sorted_D,
                      m_dSortedPosRad,
                      m_dSortedVelMas,
                      m_dSortedRhoPreMu,
                      m_dGridMarkerIndex,
                      m_dCellStart,
                      m_dCellEnd,
                      numAllMarkers,
                      m_numGridCells);
  /* Collide */
  collide(derivVelRhoD,
          m_dSortedPosRad,
          m_dSortedVelMas,
          vel_XSPH_Sorted_D,
          m_dSortedRhoPreMu,
          m_dGridMarkerIndex,
          m_dCellStart,
          m_dCellEnd,
          numAllMarkers,
          m_numGridCells,
          dT);  // Arman: you can probably safely remove dT from this function.

  Copy_SortedVelXSPH_To_VelXSPH(vel_XSPH_D, vel_XSPH_Sorted_D, m_dGridMarkerIndex, numAllMarkers);

  // set the pressure and density of BC and BCE markers to those of the nearest fluid marker.
  // I put it here to use the already determined proximity computation
  //********************************************************************************************************************************
  //	ProjectDensityPressureToBCandBCE(rhoPresMuD, m_dSortedPosRad, m_dSortedRhoPreMu,
  //				m_dGridMarkerIndex, m_dCellStart, m_dCellEnd, numAllMarkers);
  //********************************************************************************************************************************
  //*********************** Calculate MaxStress on Particles
  //***********************************************************************
  thrust::device_vector<Real3> devStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
  thrust::device_vector<Real3> volStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
  thrust::device_vector<Real4> mainStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
  int numBCE = numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers;
  CalcBCE_Stresses(devStressD,
                   volStressD,
                   mainStressD,
                   m_dSortedPosRad,
                   m_dSortedVelMas,
                   m_dSortedRhoPreMu,
                   mapOriginalToSorted,
                   m_dCellStart,
                   m_dCellEnd,
                   numBCE);

  devStressD.clear();
  volStressD.clear();
  mainStressD.clear();
  //********************************************************************************************************************************
  m_dSortedPosRad.clear();
  m_dSortedVelMas.clear();
  m_dSortedRhoPreMu.clear();
  vel_XSPH_Sorted_D.clear();

  m_dGridMarkerHash.clear();
  m_dGridMarkerIndex.clear();

  mapOriginalToSorted.clear();

  m_dCellStart.clear();
  m_dCellEnd.clear();
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Rigid_FSI_ForcesD(Real3* rigid_FSI_ForcesD, Real4* totalSurfaceInteractionRigid4) {
  uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
  if (rigidSphereA >= numObjectsD.numRigidBodies) {
    return;
  }
  Real3 force3 = solid_SPH_massD * mR3(totalSurfaceInteractionRigid4[rigidSphereA]);
  rigid_FSI_ForcesD[rigidSphereA] = force3;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Markers_TorquesD(Real3* torqueMarkersD,
                                      Real4* derivVelRhoD,
                                      Real3* posRadD,
                                      uint* rigidIdentifierD,
                                      Real3* posRigidD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
  if (index >= numObjectsD.numRigid_SphMarkers) {
    return;
  }
  Real3 dist3 = Distance(posRadD[rigidMarkerIndex], posRigidD[rigidIdentifierD[index]]);
  torqueMarkersD[index] =
      solid_SPH_massD * cross(dist3, mR3(derivVelRhoD[rigidMarkerIndex]));  // solid_SPH_massD is multiplied to convert
                                                                            // from SPH acceleration to force
}
//--------------------------------------------------------------------------------------------------------------------------------
// updates the rigid body particles
__global__ void UpdateRigidMarkersPositionD(Real3* posRadD,
                                            Real4* velMasD,
                                            const Real3* rigidSPH_MeshPos_LRF_D,
                                            const uint* rigidIdentifierD,
                                            Real3* posRigidD,
                                            Real4* velMassRigidD,
                                            Real3* omegaLRF_D,
                                            Real4* qD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= numObjectsD.numRigid_SphMarkers) {
    return;
  }
  uint rigidMarkerIndex =
      index + numObjectsD.startRigidMarkers;  // updatePortionD = [start, end] index of the update portion
  int rigidBodyIndex = rigidIdentifierD[index];

  Real4 q4 = qD[rigidBodyIndex];
  Real3 a1, a2, a3;
  RotationMatirixFromQuaternion(a1, a2, a3, q4);

  Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[index];

  // position
  Real3 p_Rigid = posRigidD[rigidBodyIndex];
  posRadD[rigidMarkerIndex] =
      p_Rigid + mR3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF), dot(a3, rigidSPH_MeshPos_LRF));

  // velociy
  Real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
  Real3 omega3 = omegaLRF_D[rigidBodyIndex];
  Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
  Real4 vM = velMasD[rigidMarkerIndex];
  velMasD[rigidMarkerIndex] =
      mR4(mR3(vM_Rigid) + mR3(dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3, omegaCrossS)), vM.w);
}
//--------------------------------------------------------------------------------------------------------------------------------
// applies periodic BC along x
__global__ void CustomCopyR4ToR3(Real3* velD, Real4* velMasD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= numObjectsD.numAllMarkers) {
    return;
  }
  Real4 velMas = velMasD[index];
  velD[index] = mR3(velMas);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Populate_RigidSPH_MeshPos_LRF_kernel(Real3* rigidSPH_MeshPos_LRF_D,
                                                     Real3* posRadD,
                                                     uint* rigidIdentifierD,
                                                     Real3* posRigidD,
                                                     Real4* qD) {
  uint index = blockIdx.x * blockDim.x + threadIdx.x;
  uint rigidMarkerIndex =
      index + numObjectsD.startRigidMarkers;  // updatePortionD = [start, end] index of the update portion
  if (index >= numObjectsD.numRigid_SphMarkers) {
    return;
  }
  int rigidIndex = rigidIdentifierD[index];
  Real4 q4 = qD[rigidIndex];
  ;
  Real3 a1, a2, a3;
  RotationMatirixFromQuaternion(a1, a2, a3, q4);
  Real3 dist3 = posRadD[rigidMarkerIndex] - posRigidD[rigidIndex];
  Real3 dist3LF = InverseRotate_By_RotationMatrix_DeviceHost(a1, a2, a3, dist3);
  rigidSPH_MeshPos_LRF_D[index] = dist3LF;
}

////--------------------------------------------------------------------------------------------------------------------------------
void MakeRigidIdentifier(thrust::device_vector<uint>& rigidIdentifierD,
                         int numRigidBodies,
                         int startRigidMarkers,
                         const thrust::host_vector<int3>& referenceArray) {
  if (numRigidBodies > 0) {
    for (int rigidSphereA = 0; rigidSphereA < numRigidBodies; rigidSphereA++) {
      int3 referencePart = referenceArray[2 + rigidSphereA];
      if (referencePart.z != 1) {
        printf("error in accessing rigid bodies. Reference array indexing is wrong\n");
        return;
      }
      int2 updatePortion =
          mI2(referencePart);  // first two component of the referenceArray denote to the fluid and boundary particles
      thrust::fill(rigidIdentifierD.begin() + (updatePortion.x - startRigidMarkers),
                   rigidIdentifierD.begin() + (updatePortion.y - startRigidMarkers),
                   rigidSphereA);
    }
  }
}
////--------------------------------------------------------------------------------------------------------------------------------

void Populate_RigidSPH_MeshPos_LRF(thrust::device_vector<uint>& rigidIdentifierD,
                                   thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
                                   const thrust::device_vector<Real3>& posRadD,
                                   const thrust::device_vector<Real3>& posRigidD,
                                   const thrust::device_vector<Real4>& qD,
                                   const thrust::host_vector<int3>& referenceArray,
                                   const NumberOfObjects& numObjects) {
  MakeRigidIdentifier(rigidIdentifierD, numObjects.numRigidBodies, numObjects.startRigidMarkers, referenceArray);

  uint nBlocks_numRigid_SphMarkers;
  uint nThreads_SphMarkers;
  computeGridSize(numObjects.numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

  Populate_RigidSPH_MeshPos_LRF_kernel << <nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>
      (mR3CAST(rigidSPH_MeshPos_LRF_D), mR3CAST(posRadD), U1CAST(rigidIdentifierD), mR3CAST(posRigidD), mR4CAST(qD));
  cudaThreadSynchronize();
  cudaCheckError();
}

//--------------------------------------------------------------------------------------------------------------------------------
// applies the time step to the current quantities and saves the new values into variable with the same name and '2' and
// the end
// precondition: for the first step of RK2, all variables with '2' at the end have the values the same as those without
// '2' at the end.
void Rigid_Forces_Torques(thrust::device_vector<Real3>& rigid_FSI_ForcesD,
                          thrust::device_vector<Real3>& rigid_FSI_TorquesD,

                          const thrust::device_vector<Real3>& posRadD,
                          const thrust::device_vector<Real3>& posRigidD,

                          const thrust::device_vector<Real4>& derivVelRhoD,
                          const thrust::device_vector<uint>& rigidIdentifierD,

                          const NumberOfObjects& numObjects,
                          Real sphMass) {
  // Arman: InitSystem has to be called before this point to set the number of objects

  if (numObjects.numRigidBodies == 0) {
    return;
  }
  cudaMemcpyToSymbolAsync(solid_SPH_massD, &sphMass, sizeof(sphMass));
  //################################################### make force and torque arrays
  //####### Force (Acceleration)
  thrust::device_vector<Real4> totalSurfaceInteractionRigid4(numObjects.numRigidBodies);
  thrust::fill(totalSurfaceInteractionRigid4.begin(), totalSurfaceInteractionRigid4.end(), mR4(0));
  thrust::device_vector<int> dummyIdentify(numObjects.numRigidBodies);
  thrust::equal_to<uint> binary_pred;

  //** forces on BCE markers of each rigid body are accumulated at center. "totalSurfaceInteractionRigid4" is got built.
  (void)thrust::reduce_by_key(rigidIdentifierD.begin(),
                              rigidIdentifierD.end(),
                              derivVelRhoD.begin() + numObjects.startRigidMarkers,
                              dummyIdentify.begin(),
                              totalSurfaceInteractionRigid4.begin(),
                              binary_pred,
                              thrust::plus<Real4>());
  thrust::fill(rigid_FSI_ForcesD.begin(), rigid_FSI_ForcesD.end(), mR3(0));

  uint nBlock_UpdateRigid;
  uint nThreads_rigidParticles;
  computeGridSize(numObjects.numRigidBodies, 128, nBlock_UpdateRigid, nThreads_rigidParticles);

  //** accumulated BCE forces at center are transformed to acceleration of rigid body "rigid_FSI_ForcesD".
  //"rigid_FSI_ForcesD" gets built.
  Calc_Rigid_FSI_ForcesD << <nBlock_UpdateRigid, nThreads_rigidParticles>>>
      (mR3CAST(rigid_FSI_ForcesD), mR4CAST(totalSurfaceInteractionRigid4));
  cudaThreadSynchronize();
  cudaCheckError();

  totalSurfaceInteractionRigid4.clear();

  //####### Torque
  uint nBlocks_numRigid_SphMarkers;
  uint nThreads_SphMarkers;
  computeGridSize(numObjects.numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);
  thrust::device_vector<Real3> torqueMarkersD(numObjects.numRigid_SphMarkers);

  //** the current position of the rigid, 'posRigidD', is used to calculate the moment of BCE acceleration at the rigid
  //*** body center (i.e. torque/mass). "torqueMarkersD" gets built.
  Calc_Markers_TorquesD << <nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>
      (mR3CAST(torqueMarkersD), mR4CAST(derivVelRhoD), mR3CAST(posRadD), U1CAST(rigidIdentifierD), mR3CAST(posRigidD));
  cudaThreadSynchronize();
  cudaCheckError();

  (void)thrust::reduce_by_key(rigidIdentifierD.begin(),
                              rigidIdentifierD.end(),
                              torqueMarkersD.begin(),
                              dummyIdentify.begin(),
                              rigid_FSI_TorquesD.begin(),
                              binary_pred,
                              thrust::plus<Real3>());

  torqueMarkersD.clear();
  dummyIdentify.clear();
}

//--------------------------------------------------------------------------------------------------------------------------------
void UpdateRigidMarkersPosition(thrust::device_vector<Real3>& posRadD,
                                thrust::device_vector<Real4>& velMasD,
                                const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
                                const thrust::device_vector<uint>& rigidIdentifierD,
                                const thrust::device_vector<Real3>& posRigidD,
                                const thrust::device_vector<Real4>& qD,
                                const thrust::device_vector<Real4>& velMassRigidD,
                                const thrust::device_vector<Real3>& omegaLRF_D,
                                NumberOfObjects numObjects) {
  printf(" implement me  ************************* \n\n\n\n\nn\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

  uint nBlocks_numRigid_SphMarkers;
  uint nThreads_SphMarkers;
  computeGridSize(numObjects.numRigid_SphMarkers, 256, nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

  // Arman: InitSystem has to be called before this lunch to set numObjectsD

  //################################################### update BCE markers position
  //** "posRadD2"/"velMasD2" associated to BCE markers are updated based on new rigid body (position,
  // orientation)/(velocity, angular velocity)
  UpdateRigidMarkersPositionD << <nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>> (mR3CAST(posRadD),
                                                                                       mR4CAST(velMasD),
                                                                                       mR3CAST(rigidSPH_MeshPos_LRF_D),
                                                                                       U1CAST(rigidIdentifierD),
                                                                                       mR3CAST(posRigidD),
                                                                                       mR4CAST(velMassRigidD),
                                                                                       mR3CAST(omegaLRF_D),
                                                                                       mR4CAST(qD));
  cudaThreadSynchronize();
  cudaCheckError();
}

//*******************************************************************************************************************************
// builds the neighbors' list of each particle and finds the force on each particle
// calculates the interaction force between 1- fluid-fluid, 2- fluid-solid, 3- solid-fluid particles
// calculates forces from other SPH or solid particles, as wall as boundaries
void ForceSPH_LF(thrust::device_vector<Real3>& posRadD,
                 thrust::device_vector<Real4>& velMasD,
                 thrust::device_vector<Real4>& rhoPresMuD,

                 thrust::device_vector<uint>& bodyIndexD,
                 thrust::device_vector<Real4>& derivVelRhoD,
                 const thrust::host_vector<int3>& referenceArray,
                 const NumberOfObjects& numObjects,
                 SimParams paramsH,
                 BceVersion bceType,
                 Real dT) {
  // Part1: contact detection
  // #########################################################################################################################
  // grid data for sorting method
  //	Real3* m_dSortedPosRad;
  //	Real4* m_dSortedVelMas;
  //	Real4* m_dSortedRhoPreMu;
  //	uint* m_dCellStart; // index of start of each cell in sorted list
  //	uint* m_dCellEnd; // index of end of cell

  uint m_numGridCells = paramsH.gridSize.x * paramsH.gridSize.y * paramsH.gridSize.z;  // m_gridSize = SIDE
  // TODO here

  int numAllMarkers = numObjects.numAllMarkers;
  // calculate grid hash
  thrust::device_vector<Real3> m_dSortedPosRad(numAllMarkers);
  thrust::device_vector<Real4> m_dSortedVelMas(numAllMarkers);
  thrust::device_vector<Real4> m_dSortedRhoPreMu(numAllMarkers);

  thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
  thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

  thrust::device_vector<uint> mapOriginalToSorted(numAllMarkers);

  thrust::device_vector<uint> m_dCellStart(m_numGridCells);
  thrust::device_vector<uint> m_dCellEnd(m_numGridCells);
  // calculate grid hash
  calcHash(m_dGridMarkerHash, m_dGridMarkerIndex, posRadD, numAllMarkers);

  //	GpuTimer myT0;
  //	myT0.Start();
  thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());
  //	myT0.Stop();
  //	Real t0 = (Real)myT0.Elapsed();
  //	printf("(0) ** Sort by key timer %f, array size %d\n", t0, m_dGridMarkerHash.size());

  // reorder particle arrays into sorted order and find start and end of each cell
  reorderDataAndFindCellStart(m_dCellStart,
                              m_dCellEnd,
                              m_dSortedPosRad,
                              m_dSortedVelMas,
                              m_dSortedRhoPreMu,
                              m_dGridMarkerHash,
                              m_dGridMarkerIndex,
                              mapOriginalToSorted,
                              posRadD,
                              velMasD,
                              rhoPresMuD,
                              numAllMarkers,
                              m_numGridCells);

  // modify BCE velocity and pressure
  if (bceType == ADAMI) {
    RecalcSortedVelocityPressure_BCE(
        m_dSortedVelMas, m_dSortedRhoPreMu, m_dSortedPosRad, m_dCellStart, m_dCellEnd, numAllMarkers);
  }

  // process collisions
  Real3 totalFluidBodyForce3 = paramsH.bodyForce3 + paramsH.gravity;
  //	Real3 totalFluidBodyForce3 = paramsH.bodyForce3;  // gravity is added in ChSystem
  thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), mR4(0));  // initialize derivVelRhoD with zero. necessary
  //	GpuTimer myT1;
  //	myT1.Start();
  thrust::fill(derivVelRhoD.begin() + referenceArray[0].x,
               derivVelRhoD.begin() + referenceArray[0].y,
               mR4(totalFluidBodyForce3));  // add body force to fluid particles.
  //	myT1.Stop();
  //	Real t1 = (Real)myT1.Elapsed();
  //	printf("(1) *** fill timer %f, array size %d\n", t1, referenceArray[0].y - referenceArray[0].x);

  thrust::device_vector<Real3> dummy_XSPH(numAllMarkers);
  uint nBlock_NumSpheres, nThreads_SphMarkers;
  computeGridSize(numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
  CustomCopyR4ToR3 << <nBlock_NumSpheres, nThreads_SphMarkers>>> (mR3CAST(dummy_XSPH), mR4CAST(m_dSortedVelMas));
  cudaThreadSynchronize();
  cudaCheckError();

  collide(derivVelRhoD,
          m_dSortedPosRad,
          m_dSortedVelMas,
          dummy_XSPH,
          m_dSortedRhoPreMu,
          m_dGridMarkerIndex,
          m_dCellStart,
          m_dCellEnd,
          numAllMarkers,
          m_numGridCells,
          dT);  // vel XSPH is the same as vel

  dummy_XSPH.clear();

  // set the pressure and density of BC and BCE markers to those of the nearest fluid marker.
  // I put it here to use the already determined proximity computation
  //********************************************************************************************************************************
  //	ProjectDensityPressureToBCandBCE(rhoPresMuD, m_dSortedPosRad, m_dSortedRhoPreMu,
  //				m_dGridMarkerIndex, m_dCellStart, m_dCellEnd, numAllMarkers);
  //********************************************************************************************************************************
  //*********************** Calculate MaxStress on Particles
  //***********************************************************************
  thrust::device_vector<Real3> devStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
  thrust::device_vector<Real3> volStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
  thrust::device_vector<Real4> mainStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
  int numBCE = numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers;
  CalcBCE_Stresses(devStressD,
                   volStressD,
                   mainStressD,
                   m_dSortedPosRad,
                   m_dSortedVelMas,
                   m_dSortedRhoPreMu,
                   mapOriginalToSorted,
                   m_dCellStart,
                   m_dCellEnd,
                   numBCE);

  devStressD.clear();
  volStressD.clear();
  mainStressD.clear();
  //********************************************************************************************************************************
  m_dSortedPosRad.clear();
  m_dSortedVelMas.clear();
  m_dSortedRhoPreMu.clear();

  m_dGridMarkerHash.clear();
  m_dGridMarkerIndex.clear();

  mapOriginalToSorted.clear();

  m_dCellStart.clear();
  m_dCellEnd.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void DensityReinitialization(thrust::device_vector<Real3>& posRadD,
                             thrust::device_vector<Real4>& velMasD,
                             thrust::device_vector<Real4>& rhoPresMuD,
                             int numAllMarkers,
                             int3 SIDE) {
  //	Real3* m_dSortedPosRad;
  //	Real4* m_dSortedVelMas;
  //	Real4* m_dSortedRhoPreMu;
  //	uint* m_dCellStart; // index of start of each cell in sorted list
  //	uint* m_dCellEnd; // index of end of cell

  uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z;  // m_gridSize = SIDE
  // TODO here

  // calculate grid hash
  thrust::device_vector<Real3> m_dSortedPosRad(numAllMarkers);
  thrust::device_vector<Real4> m_dSortedVelMas(numAllMarkers);
  thrust::device_vector<Real4> m_dSortedRhoPreMu(numAllMarkers);

  thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
  thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

  thrust::device_vector<uint> mapOriginalToSorted(numAllMarkers);

  thrust::device_vector<uint> m_dCellStart(m_numGridCells);
  thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

  // calculate grid hash
  calcHash(m_dGridMarkerHash, m_dGridMarkerIndex, posRadD, numAllMarkers);

  thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());

  // reorder particle arrays into sorted order and find start and end of each cell
  reorderDataAndFindCellStart(m_dCellStart,
                              m_dCellEnd,
                              m_dSortedPosRad,
                              m_dSortedVelMas,
                              m_dSortedRhoPreMu,
                              m_dGridMarkerHash,
                              m_dGridMarkerIndex,
                              mapOriginalToSorted,
                              posRadD,
                              velMasD,
                              rhoPresMuD,
                              numAllMarkers,
                              m_numGridCells);

  ReCalcDensity(posRadD,
                velMasD,
                rhoPresMuD,
                m_dSortedPosRad,
                m_dSortedVelMas,
                m_dSortedRhoPreMu,
                m_dGridMarkerIndex,
                m_dCellStart,
                m_dCellEnd,
                numAllMarkers);

  m_dSortedPosRad.clear();
  m_dSortedVelMas.clear();
  m_dSortedRhoPreMu.clear();

  m_dGridMarkerHash.clear();
  m_dGridMarkerIndex.clear();

  mapOriginalToSorted.clear();

  m_dCellStart.clear();
  m_dCellEnd.clear();
}
/**
 * @brief See collideSphereSphere.cuh for documentation.
 */
void InitSystem(SimParams paramsH, NumberOfObjects numObjects) {
  setParameters(&paramsH, &numObjects);                                          // sets paramsD in SDKCollisionSystem
  cudaMemcpyToSymbolAsync(paramsD, &paramsH, sizeof(SimParams));  // sets paramsD for this file
  cudaMemcpyToSymbolAsync(numObjectsD, &numObjects, sizeof(NumberOfObjects));
}
/**
 * @brief See collideSphereSphere.cuh for documentation.
 */
void ResizeMyThrust3(thrust::device_vector<Real3>& mThrustVec, int mSize) {
  mThrustVec.resize(mSize);
}
void ResizeMyThrust4(thrust::device_vector<Real4>& mThrustVec, int mSize) {
  mThrustVec.resize(mSize);
}

/**
 * @brief See collideSphereSphere.cuh for documentation.
 */
void FillMyThrust4(thrust::device_vector<Real4>& mThrustVec, Real4 v) {
  thrust::fill(mThrustVec.begin(), mThrustVec.end(), v);
}

/**
 * @brief See collideSphereSphere.cuh for documentation.
 */
void ClearMyThrustR3(thrust::device_vector<Real3>& mThrustVec) {
  mThrustVec.clear();
}
void ClearMyThrustR4(thrust::device_vector<Real4>& mThrustVec) {
  mThrustVec.clear();
}
void ClearMyThrustU1(thrust::device_vector<uint>& mThrustVec) {
  mThrustVec.clear();
}
void PushBackR3(thrust::device_vector<Real3>& mThrustVec, Real3 a3) {
  mThrustVec.push_back(a3);
}
void PushBackR4(thrust::device_vector<Real4>& mThrustVec, Real4 a4) {
  mThrustVec.push_back(a4);
}
void ResizeR3(thrust::device_vector<Real3>& mThrustVec, int size) {
  mThrustVec.resize(size);
}
void ResizeR4(thrust::device_vector<Real4>& mThrustVec, int size) {
  mThrustVec.resize(size);
}
void ResizeU1(thrust::device_vector<uint>& mThrustVec, int size) {
  mThrustVec.resize(size);
}

/**
 * @brief See collideSphereSphere.cuh for more documentation.
 */
void IntegrateSPH(thrust::device_vector<Real4>& derivVelRhoD,

                  thrust::device_vector<Real3>& posRadD2,
                  thrust::device_vector<Real4>& velMasD2,
                  thrust::device_vector<Real4>& rhoPresMuD2,

                  thrust::device_vector<Real3>& posRadD,
                  thrust::device_vector<Real4>& velMasD,
                  thrust::device_vector<Real3>& vel_XSPH_D,
                  thrust::device_vector<Real4>& rhoPresMuD,

                  thrust::device_vector<uint>& bodyIndexD,
                  const thrust::host_vector<int3>& referenceArray,
                  const NumberOfObjects& numObjects,
                  SimParams currentParamsH,
                  Real dT) {
  ForceSPH(posRadD,
           velMasD,
           vel_XSPH_D,
           rhoPresMuD,
           bodyIndexD,
           derivVelRhoD,
           referenceArray,
           numObjects,
           currentParamsH,
           mORIGINAL,
           dT);  //?$ right now, it does not consider paramsH.gravity or other stuff on rigid bodies. they should be
  // applied at rigid body solver
  UpdateFluid(posRadD2,
              velMasD2,
              vel_XSPH_D,
              rhoPresMuD2,
              derivVelRhoD,
              referenceArray,
              dT);  // assumes ...D2 is a copy of ...D
  // UpdateBoundary(posRadD2, velMasD2, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * currentParamsH.dT);
  // //assumes ...D2 is a copy of ...D
  ApplyBoundarySPH_Markers(posRadD2, rhoPresMuD2, numObjects.numAllMarkers);
}
////##############################################################################################################################################
//// the main function, which updates the particles and implements BC
// void cudaCollisions(
//		thrust::host_vector<Real3> & mPosRad,
//		thrust::host_vector<Real4> & mVelMas,
//		thrust::host_vector<Real4> & mRhoPresMu,
//		const thrust::host_vector<uint> & bodyIndex,
//		const thrust::host_vector<int3> & referenceArray,
//
//		SimParams paramsH,
//		NumberOfObjects numObjects) {
//
//	//--------- initialization ---------------
//	//cudaError_t dumDevErr = cudaSetDevice(2);
//	GpuTimer myTotalTime;
//	myTotalTime.Start();
//	//printf("cMin.x, y, z, CMAx.x, y, z, binSize %f %f %f , %f %f %f, %f\n", paramsH.cMin.x, paramsH.cMin.y,
// paramsH.cMin.z, paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, paramsH.binSize0);
//	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);
//
//	thrust::device_vector<Real3> posRadD=mPosRad;
//	thrust::device_vector<Real4> velMasD=mVelMas;
//	thrust::device_vector<Real4> rhoPresMuD=mRhoPresMu;
//
//	thrust::device_vector<uint> bodyIndexD=bodyIndex;
//	thrust::device_vector<Real4> derivVelRhoD(numObjects.numAllMarkers);
//
//
//
//	//******************************************************************************
//
//	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
//		//************************************************
//		PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray, currentParamsH, realTime, tStep);
//		//************
//
//		GpuTimer myGpuTimer;
//		myGpuTimer.Start();
//
//		// Arman timer Added
//		CpuTimer mCpuTimer;
//		mCpuTimer.Start();
//
//		//***********
//		if (realTime <= paramsH.timePause) 	{
//			currentParamsH = paramsH_B;
//		} else {
//			currentParamsH = paramsH;
//		}
//		//***********
//
//		setParameters(&currentParamsH, &numObjects);// sets paramsD in SDKCollisionSystem
//		cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &currentParamsH, sizeof(SimParams))); 	//sets paramsD
// for this file
//
//		//computations
//				//markers
//		thrust::device_vector<Real3> posRadD2 = posRadD;
//		thrust::device_vector<Real4> velMasD2 = velMasD;
//		thrust::device_vector<Real4> rhoPresMuD2 = rhoPresMuD;
//		thrust::device_vector<Real3> vel_XSPH_D(numObjects.numAllMarkers);
//
//
//		//******** RK2
//		IntegrateSPH(posRadD2, velMasD2, rhoPresMuD2, posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD,
// derivVelRhoD, referenceArray, numObjects, currentParamsH, 0.5 * currentParamsH.dT);
//		IntegrateSPH(posRadD, velMasD, rhoPresMuD, posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD,
// derivVelRhoD, referenceArray, numObjects, currentParamsH, currentParamsH.dT);
//
//
//		//************
//		posRadD2.clear();
//		velMasD2.clear();
//		rhoPresMuD2.clear();
//		vel_XSPH_D.clear();
//
//		//density re-initialization
////		if ((tStep % 10 == 0) && (paramsH.densityReinit != 0)) {
////			DensityReinitialization(posRadD, velMasD, rhoPresMuD, numObjects.numAllMarkers,
/// paramsH.gridSize);
/////does not work for analytical boundaries (non-meshed) and free surfaces
////		}
//
//		myGpuTimer.Stop();
//		Real time2 = (Real)myGpuTimer.Elapsed();
//
//		//cudaDeviceSynchronize();
//
//		//Arman timer Added
//		mCpuTimer.Stop();
//
//		if (tStep % 50 == 0) {
//			printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ", tStep, realTime,
// time2, 1000 * mCpuTimer.Elapsed());
//
////			// ************ calc and print cartesian data ************************************
////			int3 cartesianGridDims;
////			thrust::host_vector<Real4> rho_Pres_CartH(1);
////			thrust::host_vector<Real4> vel_VelMag_CartH(1);
////			MapSPH_ToGrid(2 * paramsH.HSML, cartesianGridDims, rho_Pres_CartH, vel_VelMag_CartH,
////					posRadD, velMasD, rhoPresMuD, numObjects.numAllMarkers, paramsH);
////			PrintCartesianData_MidLine(rho_Pres_CartH, vel_VelMag_CartH, cartesianGridDims, paramsH);
////			// *******************************************************************************
//		}
//		fflush(stdout);
//
//		realTime += currentParamsH.dT;
//
//		//_CrtDumpMemoryLeaks(); //for memory leak detection (msdn suggestion for VS) apparently does not work
// in
// conjunction with cuda
//
//	}
//
//	//you may copy back to host
//	posRadD.clear();
//	velMasD.clear();
//	rhoPresMuD.clear();
//
//	bodyIndexD.clear();
//	derivVelRhoD.clear();
//
//	myTotalTime.Stop();
//	Real time = (Real)myTotalTime.Elapsed();
//	printf("total Time: %f\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n ", time);
//}
