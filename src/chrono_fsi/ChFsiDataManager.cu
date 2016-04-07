// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include <thrust/sort.h>
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChDeviceUtils.cuh" 

namespace chrono {
namespace fsi {

struct sphTypeComp {
 	__host__ __device__ bool operator()(const Real4& o1, const Real4& o2) {
    	return o1.w < o2.w;
  	}
};

//---------------------------------------------------------------------------------------
zipIterSphD SphMarkerDataD::iterator() {
			return thrust::make_zip_iterator(thrust::make_tuple(posRadD.begin(), velMasD.begin(), rhoPresMuD.begin()));
}

// resize
void SphMarkerDataD::resize(int s) {
	posRadD.resize(s);
	velMasD.resize(s);
	rhoPresMuD.resize(s);
}


//---------------------------------------------------------------------------------------

zipIterSphH SphMarkerDataH::iterator() {
	return thrust::make_zip_iterator(thrust::make_tuple(posRadH.begin(), velMasH.begin(), rhoPresMuH.begin()));
}

// resize
void SphMarkerDataH::resize(int s) {
	posRadH.resize(s);
	velMasH.resize(s);
	rhoPresMuH.resize(s);
}

//---------------------------------------------------------------------------------------

zipIterRigidD FsiBodiesDataD::iterator() {
	return thrust::make_zip_iterator(thrust::make_tuple(posRigid_fsiBodies_D.begin(), velMassRigid_fsiBodies_D.begin(), accRigid_fsiBodies_D.begin(),
		q_fsiBodies_D.begin(), omegaVelLRF_fsiBodies_D.begin(), omegaAccLRF_fsiBodies_D.begin()));
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


//---------------------------------------------------------------------------------------

zipIterRigidH FsiBodiesDataH::iterator() {
	return thrust::make_zip_iterator(thrust::make_tuple(posRigid_fsiBodies_H.begin(), velMassRigid_fsiBodies_H.begin(), accRigid_fsiBodies_H.begin(),
		q_fsiBodies_H.begin(), omegaVelLRF_fsiBodies_H.begin(), omegaAccLRF_fsiBodies_H.begin()));
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



//---------------------------------------------------------------------------------------


ChFsiDataManager::ChFsiDataManager() {
	InitNumObjects();
}

ChFsiDataManager::~ChFsiDataManager() {}

void ChFsiDataManager::AddSphMarker(Real3 pos, Real3 vel, Real4 rhoPresMu) {
	sphMarkersH.posRadH.push_back(pos);
	sphMarkersH.velMasH.push_back(vel);
	sphMarkersH.rhoPresMuH.push_back(rhoPresMu);
}

void ChFsiDataManager::ArrangeDataManager() {
	thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH.rhoPresMuH;

	// arrange data based on type: fluid, boundary, bce1, bce2, ....
	thrust::sort_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(), sphMarkersH.iterator(), sphTypeComp());
	dummyRhoPresMuH.clear();
}

void ChFsiDataManager::InitNumObjects() {
	numObjects.numRigidBodies = 0; /* Number of rigid bodies */
	numObjects.numFlexBodies = 0; /* Number of Flexible bodies*/
	numObjects.numFluidMarkers = 0; /* Number of fluid SPH markers*/
	numObjects.numBoundaryMarkers = 0; /* Number of boundary SPH markers */
	numObjects.startRigidMarkers = 0; /* */
	numObjects.startFlexMarkers = 0; /* */
	numObjects.numRigid_SphMarkers = 0; /* */
	numObjects.numFlex_SphMarkers = 0; /* */
	numObjects.numAllMarkers = 0; /* Total number of SPH markers */
}

void ChFsiDataManager::CalcNumObjects() {
	printf("*** calcnumobjects\n");
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
			std::cout << "Error! particle type not defined! Thrown from CalcNumObjects\n";
			break;
		}
	}

	numObjects.numAllMarkers = 
		numObjects.numFluidMarkers + 
		numObjects.numBoundaryMarkers +
		numObjects.numRigid_SphMarkers + 
		numObjects.numFlex_SphMarkers;

	numObjects.startRigidMarkers = (flagRigid) ? 
		(numObjects.numFluidMarkers + numObjects.numBoundaryMarkers) :
		numObjects.numAllMarkers;

	numObjects.startFlexMarkers = (flagFlex) ? 
		(numObjects.numFluidMarkers + numObjects.numBoundaryMarkers + numObjects.numRigid_SphMarkers) :
		numObjects.numAllMarkers;



	printf("numFluid %d boundary %d ridigSph %d flexSph %d all %d start rigid %d startFlex %d \n",
			numObjects.numFluidMarkers,
			numObjects.numBoundaryMarkers,
			numObjects.numRigid_SphMarkers,
			numObjects.numFlex_SphMarkers,
			numObjects.numAllMarkers,
			numObjects.startRigidMarkers,
			numObjects.startFlexMarkers);
}

void ChFsiDataManager::ConstructReferenceArray() {
	ArrangeDataManager();
	CalcNumObjects();

	// determine the number of each component
	if (numObjects.numAllMarkers != sphMarkersH.rhoPresMuH.size()) {
		printf("Error! numObjects wrong! thrown from ConstructReferenceArray");
	}
	thrust::host_vector<int> numComponentMarkers(numObjects.numAllMarkers);
	thrust::fill(numComponentMarkers.begin(), numComponentMarkers.end(), 1);
	thrust::host_vector<Real4> dummyRhoPresMuH = sphMarkersH.rhoPresMuH;
	thrust::copy(sphMarkersH.rhoPresMuH.begin(), sphMarkersH.rhoPresMuH.end(), dummyRhoPresMuH.begin());
	int numberOfComponents = (thrust::reduce_by_key(dummyRhoPresMuH.begin(), dummyRhoPresMuH.end(), numComponentMarkers.begin(), 
			dummyRhoPresMuH.begin(), numComponentMarkers.begin(), sphTypeComp())).second 
			- numComponentMarkers.begin();

	// if (numberOfComponents == 0) {
	// 	std::cout << "Error! no marker found! Thrown from ConstructReferenceArray\n";
	// 	return;
	// }
	fsiGeneralData.referenceArray.resize(numberOfComponents);
	dummyRhoPresMuH.resize(numberOfComponents);
	numComponentMarkers.resize(numberOfComponents);
	int savedNumber = 0;
	for (int i = 0; i < numberOfComponents; numberOfComponents++) {
		int compType = std::floor(dummyRhoPresMuH[i].w + .1);
		int phaseType = -1;
		if (compType < 0) {
			phaseType = -1;
		} else if (compType == 0) {
			phaseType = 0;
		} else { // Arman : TODO for flex
			phaseType = 1;
		}
		fsiGeneralData.referenceArray[i] = mI4(savedNumber, savedNumber + numComponentMarkers[i], phaseType, compType);
	}
	dummyRhoPresMuH.clear();	
	numComponentMarkers.clear();
}

////--------------------------------------------------------------------------------------------------------------------------------
void ChFsiDataManager::ResizeDataManager() {
	ConstructReferenceArray();
		if (numObjects.numAllMarkers != sphMarkersH.rhoPresMuH.size()) {
			printf("Error! numObjects wrong! thrown from FinalizeDataManager");
		}
		sphMarkersD1.resize(numObjects.numAllMarkers);
		sphMarkersD2.resize(numObjects.numAllMarkers);
		fsiGeneralData.derivVelRhoD.resize(numObjects.numAllMarkers);
		fsiGeneralData.vel_XSPH_D.resize(numObjects.numAllMarkers);

		thrust::copy(sphMarkersH.posRadH.begin(), sphMarkersH.posRadH.end(), sphMarkersD1.posRadD.begin());
		thrust::copy(sphMarkersH.velMasH.begin(), sphMarkersH.velMasH.end(), sphMarkersD1.velMasD.begin());
		thrust::copy(sphMarkersH.rhoPresMuH.begin(), sphMarkersH.rhoPresMuH.end(), sphMarkersD1.rhoPresMuD.begin());

		thrust::copy(sphMarkersD1.posRadD.begin(), sphMarkersD1.posRadD.end(), sphMarkersD2.posRadD.begin());
		thrust::copy(sphMarkersD1.velMasD.begin(), sphMarkersD1.velMasD.end(), sphMarkersD2.velMasD.begin());
		thrust::copy(sphMarkersD1.rhoPresMuD.begin(), sphMarkersD1.rhoPresMuD.end(), sphMarkersD2.rhoPresMuD.begin());

		// copy rigids
		fsiBodiesD1.resize(numObjects.numRigidBodies);
		fsiBodiesD2.resize(numObjects.numRigidBodies);
		fsiGeneralData.rigid_FSI_ForcesD.resize(numObjects.numRigidBodies);
		fsiGeneralData.rigid_FSI_TorquesD.resize(numObjects.numRigidBodies);
		fsiGeneralData.rigidIdentifierD.resize(numObjects.numRigid_SphMarkers);
		fsiGeneralData.rigidSPH_MeshPos_LRF_D.resize(numObjects.numRigid_SphMarkers);
}

////--------------------------------------------------------------------------------------------------------------------------------

void ChFsiDataManager::CopyFsiBodiesDataH2D() {

	// Arman: do it with zip iterator
	thrust::copy(fsiBodiesH.posRigid_fsiBodies_H.begin(), fsiBodiesH.posRigid_fsiBodies_H.end(), fsiBodiesD1.posRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.velMassRigid_fsiBodies_H.begin(), fsiBodiesH.velMassRigid_fsiBodies_H.end(), fsiBodiesD1.velMassRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.accRigid_fsiBodies_H.begin(), fsiBodiesH.accRigid_fsiBodies_H.end(), fsiBodiesD1.accRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.q_fsiBodies_H.begin(), fsiBodiesH.q_fsiBodies_H.end(), fsiBodiesD1.q_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.omegaVelLRF_fsiBodies_H.begin(), fsiBodiesH.omegaVelLRF_fsiBodies_H.end(), fsiBodiesD1.omegaVelLRF_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.omegaAccLRF_fsiBodies_H.begin(), fsiBodiesH.omegaAccLRF_fsiBodies_H.end(), fsiBodiesD1.omegaAccLRF_fsiBodies_D.begin());

	thrust::copy(fsiBodiesD1.posRigid_fsiBodies_D.begin(), fsiBodiesD1.posRigid_fsiBodies_D.end(), fsiBodiesD2.posRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.velMassRigid_fsiBodies_D.begin(), fsiBodiesD1.velMassRigid_fsiBodies_D.end(), fsiBodiesD2.velMassRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.accRigid_fsiBodies_D.begin(), fsiBodiesD1.accRigid_fsiBodies_D.end(), fsiBodiesD2.accRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.q_fsiBodies_D.begin(), fsiBodiesD1.q_fsiBodies_D.end(), fsiBodiesD1.q_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.omegaVelLRF_fsiBodies_D.begin(), fsiBodiesD1.omegaVelLRF_fsiBodies_D.end(), fsiBodiesD2.omegaVelLRF_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.omegaAccLRF_fsiBodies_D.begin(), fsiBodiesD1.omegaAccLRF_fsiBodies_D.end(), fsiBodiesD2.omegaAccLRF_fsiBodies_D.begin());
}

} // end namespace fsi
} // end namespace chrono

