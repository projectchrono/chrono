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
// Base class for managing data in chrono_fsi, aka fluid system.//
// =============================================================================

#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include <thrust/sort.h>

namespace chrono {
namespace fsi {

struct sphTypeCompLess {
  __host__ __device__ bool operator()(const Real4 &o1, const Real4 &o2) {
    return o1.w <= o2.w;
  }
};

struct sphTypeCompEqual {
  __host__ __device__ bool operator()(const Real4 &o1, const Real4 &o2) {
    return o1.w == o2.w;
  }
};

//---------------------------------------------------------------------------------------
zipIterSphD SphMarkerDataD::iterator() {
  return thrust::make_zip_iterator(
      thrust::make_tuple(posRadD.begin(), velMasD.begin(), rhoPresMuD.begin()));
}

// resize
void SphMarkerDataD::resize(int s) {
  posRadD.resize(s);
  velMasD.resize(s);
  rhoPresMuD.resize(s);
}

//---------------------------------------------------------------------------------------

zipIterSphH SphMarkerDataH::iterator() {
  return thrust::make_zip_iterator(
      thrust::make_tuple(posRadH.begin(), velMasH.begin(), rhoPresMuH.begin()));
}

// resize
void SphMarkerDataH::resize(int s) {
  posRadH.resize(s);
  velMasH.resize(s);
  rhoPresMuH.resize(s);
}

//---------------------------------------------------------------------------------------

zipIterRigidD FsiBodiesDataD::iterator() {
  return thrust::make_zip_iterator(thrust::make_tuple(
      posRigid_fsiBodies_D.begin(), velMassRigid_fsiBodies_D.begin(),
      accRigid_fsiBodies_D.begin(), q_fsiBodies_D.begin(),
      omegaVelLRF_fsiBodies_D.begin(), omegaAccLRF_fsiBodies_D.begin()));
}

// resize
void FsiBodiesDataD::resize(int s) {
  posRigid_fsiBodies_D.resize(s);
  velMassRigid_fsiBodies_D.resize(s);
  accRigid_fsiBodies_D.resize(s);
  q_fsiBodies_D.resize(s);
  omegaVelLRF_fsiBodies_D.resize(s);
  omegaAccLRF_fsiBodies_D.resize(s);
}

void FsiBodiesDataD::CopyFromH(const FsiBodiesDataH &other) {
  thrust::copy(other.posRigid_fsiBodies_H.begin(),
               other.posRigid_fsiBodies_H.end(), posRigid_fsiBodies_D.begin());
  thrust::copy(other.velMassRigid_fsiBodies_H.begin(),
               other.velMassRigid_fsiBodies_H.end(),
               velMassRigid_fsiBodies_D.begin());
  thrust::copy(other.accRigid_fsiBodies_H.begin(),
               other.accRigid_fsiBodies_H.end(), accRigid_fsiBodies_D.begin());
  thrust::copy(other.q_fsiBodies_H.begin(), other.q_fsiBodies_H.end(),
               q_fsiBodies_D.begin());
  thrust::copy(other.omegaVelLRF_fsiBodies_H.begin(),
               other.omegaVelLRF_fsiBodies_H.end(),
               omegaVelLRF_fsiBodies_D.begin());
  thrust::copy(other.omegaAccLRF_fsiBodies_H.begin(),
               other.omegaAccLRF_fsiBodies_H.end(),
               omegaAccLRF_fsiBodies_D.begin());
}

FsiBodiesDataD &FsiBodiesDataD::operator=(const FsiBodiesDataD &other) {
  if (this == &other) {
    return *this;
  }
  thrust::copy(other.posRigid_fsiBodies_D.begin(),
               other.posRigid_fsiBodies_D.end(), posRigid_fsiBodies_D.begin());
  thrust::copy(other.velMassRigid_fsiBodies_D.begin(),
               other.velMassRigid_fsiBodies_D.end(),
               velMassRigid_fsiBodies_D.begin());
  thrust::copy(other.accRigid_fsiBodies_D.begin(),
               other.accRigid_fsiBodies_D.end(), accRigid_fsiBodies_D.begin());
  thrust::copy(other.q_fsiBodies_D.begin(), other.q_fsiBodies_D.end(),
               q_fsiBodies_D.begin());
  thrust::copy(other.omegaVelLRF_fsiBodies_D.begin(),
               other.omegaVelLRF_fsiBodies_D.end(),
               omegaVelLRF_fsiBodies_D.begin());
  thrust::copy(other.omegaAccLRF_fsiBodies_D.begin(),
               other.omegaAccLRF_fsiBodies_D.end(),
               omegaAccLRF_fsiBodies_D.begin());
}

//---------------------------------------------------------------------------------------

zipIterRigidH FsiBodiesDataH::iterator() {
  return thrust::make_zip_iterator(thrust::make_tuple(
      posRigid_fsiBodies_H.begin(), velMassRigid_fsiBodies_H.begin(),
      accRigid_fsiBodies_H.begin(), q_fsiBodies_H.begin(),
      omegaVelLRF_fsiBodies_H.begin(), omegaAccLRF_fsiBodies_H.begin()));
}

// resize
void FsiBodiesDataH::resize(int s) {
  posRigid_fsiBodies_H.resize(s);
  velMassRigid_fsiBodies_H.resize(s);
  accRigid_fsiBodies_H.resize(s);
  q_fsiBodies_H.resize(s);
  omegaVelLRF_fsiBodies_H.resize(s);
  omegaAccLRF_fsiBodies_H.resize(s);
}

//---------------------------------------------------------------------------------------
void ProximityDataD::resize(int numAllMarkers) {
  gridMarkerHashD.resize(numAllMarkers);
  gridMarkerIndexD.resize(numAllMarkers);
  mapOriginalToSorted.resize(numAllMarkers);
}

//---------------------------------------------------------------------------------------

ChronoBodiesDataH::ChronoBodiesDataH(int s) { resize(s); }

zipIterChronoBodiesH ChronoBodiesDataH::iterator() {
  return thrust::make_zip_iterator(thrust::make_tuple(
      pos_ChSystemH.begin(), vel_ChSystemH.begin(), acc_ChSystemH.begin(),
      quat_ChSystemH.begin(), omegaVelGRF_ChSystemH.begin(),
      omegaAccGRF_ChSystemH.begin()));
}

// resize
void ChronoBodiesDataH::resize(int s) {
  pos_ChSystemH.resize(s);
  vel_ChSystemH.resize(s);
  acc_ChSystemH.resize(s);
  quat_ChSystemH.resize(s);
  omegaVelGRF_ChSystemH.resize(s);
  omegaAccGRF_ChSystemH.resize(s);
}

//---------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------

ChFsiDataManager::ChFsiDataManager() { InitNumObjects(); }

ChFsiDataManager::~ChFsiDataManager() {}

void ChFsiDataManager::AddSphMarker(Real3 pos, Real3 vel, Real4 rhoPresMu) {
  sphMarkersH.posRadH.push_back(pos);
  sphMarkersH.velMasH.push_back(vel);
  sphMarkersH.rhoPresMuH.push_back(rhoPresMu);
}

void ChFsiDataManager::ArrangeDataManager() {
  thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH.rhoPresMuH;

  // arrange data based on type: fluid, boundary, bce1, bce2, ....
  thrust::sort_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(),
                      sphMarkersH.iterator(), sphTypeCompLess());
  dummyRhoPresMuH.clear();
}

void ChFsiDataManager::InitNumObjects() {
  numObjects.numRigidBodies = 0;      /* Number of rigid bodies */
  numObjects.numFlexBodies = 0;       /* Number of Flexible bodies*/
  numObjects.numFluidMarkers = 0;     /* Number of fluid SPH markers*/
  numObjects.numBoundaryMarkers = 0;  /* Number of boundary SPH markers */
  numObjects.startRigidMarkers = 0;   /* */
  numObjects.startFlexMarkers = 0;    /* */
  numObjects.numRigid_SphMarkers = 0; /* */
  numObjects.numFlex_SphMarkers = 0;  /* */
  numObjects.numAllMarkers = 0;       /* Total number of SPH markers */
}

void ChFsiDataManager::CalcNumObjects() {
  InitNumObjects();
  int rSize = fsiGeneralData.referenceArray.size();
  bool flagRigid = false;
  bool flagFlex = false;
  for (int i = 0; i < rSize; i++) {
    ::int4 rComp4 = fsiGeneralData.referenceArray[i];
    int numMerkers = rComp4.y - rComp4.x;
    switch (rComp4.z) {
    case -1:
      numObjects.numFluidMarkers += numMerkers;
      break;
    case 0:
      numObjects.numBoundaryMarkers += numMerkers;
      break;
    case 1:
      numObjects.numRigid_SphMarkers += numMerkers;
      numObjects.numRigidBodies++;
      flagRigid = true;
      break;
    case 2:
      std::cout << "Error! phase not implemented. Thrown from SetNumObjects\n";
      numObjects.numFlex_SphMarkers += numMerkers;
      numObjects.numFlexBodies++;
      flagFlex = true;
      break;
    default:
      std::cout
          << "Error! particle type not defined! Thrown from CalcNumObjects\n";
      break;
    }
  }

  numObjects.numAllMarkers =
      numObjects.numFluidMarkers + numObjects.numBoundaryMarkers +
      numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers;

  numObjects.startRigidMarkers =
      (flagRigid) ? (numObjects.numFluidMarkers + numObjects.numBoundaryMarkers)
                  : numObjects.numAllMarkers;

  numObjects.startFlexMarkers =
      (flagFlex) ? (numObjects.numFluidMarkers + numObjects.numBoundaryMarkers +
                    numObjects.numRigid_SphMarkers)
                 : numObjects.numAllMarkers;

  printf("numFluid %d boundary %d ridigSph %d flexSph %d all %d start rigid %d "
         "startFlex %d \n",
         numObjects.numFluidMarkers, numObjects.numBoundaryMarkers,
         numObjects.numRigid_SphMarkers, numObjects.numFlex_SphMarkers,
         numObjects.numAllMarkers, numObjects.startRigidMarkers,
         numObjects.startFlexMarkers);
}

void ChFsiDataManager::ConstructReferenceArray() {
  ArrangeDataManager();
  CalcNumObjects();

  // determine the number of each component
  if (numObjects.numAllMarkers != sphMarkersH.rhoPresMuH.size()) {
    throw std::runtime_error(
        "Error! numObjects wrong! thrown from ConstructReferenceArray !\n");
  }
  thrust::host_vector<int> numComponentMarkers(numObjects.numAllMarkers);
  thrust::fill(numComponentMarkers.begin(), numComponentMarkers.end(), 1);
  thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH.rhoPresMuH;
  thrust::copy(sphMarkersH.rhoPresMuH.begin(), sphMarkersH.rhoPresMuH.end(),
               dummyRhoPresMuH.begin());
  int numberOfComponents =
      (thrust::reduce_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(),
                             numComponentMarkers.begin(),
                             dummyRhoPresMuH.begin(),
                             numComponentMarkers.begin(), sphTypeCompEqual()))
          .first -
      dummyRhoPresMuH.begin();
  // if (numberOfComponents == 0) {
  // 	std::cout << "Error! no marker found! Thrown from
  // ConstructReferenceArray\n";
  // 	return;
  // }
  fsiGeneralData.referenceArray.resize(numberOfComponents);
  dummyRhoPresMuH.resize(numberOfComponents);
  numComponentMarkers.resize(numberOfComponents);
  int savedNumber = 0;
  for (int i = 0; i < numberOfComponents; i++) {
    int compType = std::floor(dummyRhoPresMuH[i].w + .1);
    int phaseType = -1;
    if (compType < 0) {
      phaseType = -1;
    } else if (compType == 0) {
      phaseType = 0;
    } else { // Arman : TODO for flex
      phaseType = 1;
    }
    fsiGeneralData.referenceArray[i] = mI4(
        savedNumber, savedNumber + numComponentMarkers[i], phaseType, compType);
    savedNumber += numComponentMarkers[i];
  }
  dummyRhoPresMuH.clear();
  numComponentMarkers.clear();

  printf("reference array \n");
  for (int i = 0; i < fsiGeneralData.referenceArray.size(); i++) {
    int4 num = fsiGeneralData.referenceArray[i];
    printf(" %d %d %d %d \n", num.x, num.y, num.z, num.w);
  }
}

////--------------------------------------------------------------------------------------------------------------------------------
void ChFsiDataManager::ResizeDataManager() {
  ConstructReferenceArray();
  if (numObjects.numAllMarkers != sphMarkersH.rhoPresMuH.size()) {
    throw std::runtime_error(
        "Error! numObjects wrong! thrown from FinalizeDataManager !\n");
  }
  sphMarkersD1.resize(numObjects.numAllMarkers);
  sphMarkersD2.resize(numObjects.numAllMarkers);
  sortedSphMarkersD.resize(numObjects.numAllMarkers);
  sphMarkersH.resize(numObjects.numAllMarkers);
  markersProximityD.resize(numObjects.numAllMarkers);

  fsiGeneralData.derivVelRhoD.resize(numObjects.numAllMarkers);
  fsiGeneralData.vel_XSPH_D.resize(numObjects.numAllMarkers);

  // Arman: implement this in one shot function in class
  thrust::copy(sphMarkersH.posRadH.begin(), sphMarkersH.posRadH.end(),
               sphMarkersD1.posRadD.begin());
  thrust::copy(sphMarkersH.velMasH.begin(), sphMarkersH.velMasH.end(),
               sphMarkersD1.velMasD.begin());
  thrust::copy(sphMarkersH.rhoPresMuH.begin(), sphMarkersH.rhoPresMuH.end(),
               sphMarkersD1.rhoPresMuD.begin());

  thrust::copy(sphMarkersD1.posRadD.begin(), sphMarkersD1.posRadD.end(),
               sphMarkersD2.posRadD.begin());
  thrust::copy(sphMarkersD1.velMasD.begin(), sphMarkersD1.velMasD.end(),
               sphMarkersD2.velMasD.begin());
  thrust::copy(sphMarkersD1.rhoPresMuD.begin(), sphMarkersD1.rhoPresMuD.end(),
               sphMarkersD2.rhoPresMuD.begin());

  // copy rigids
  fsiBodiesD1.resize(numObjects.numRigidBodies);
  fsiBodiesD2.resize(numObjects.numRigidBodies);
  fsiBodiesH.resize(numObjects.numRigidBodies);
  fsiGeneralData.rigid_FSI_ForcesD.resize(numObjects.numRigidBodies);
  fsiGeneralData.rigid_FSI_TorquesD.resize(numObjects.numRigidBodies);
  fsiGeneralData.rigidIdentifierD.resize(numObjects.numRigid_SphMarkers);
  fsiGeneralData.rigidSPH_MeshPos_LRF_D.resize(numObjects.numRigid_SphMarkers);
}

} // end namespace fsi
} // end namespace chrono
