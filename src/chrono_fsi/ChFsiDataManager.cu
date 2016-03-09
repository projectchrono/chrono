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

ChFsiDataManager::ChFsiDataManager() {

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

	ConstructReferenceArray();
}

void ChFsiDataManager::ConstructReferenceArray() {
	ArrangeDataManager();

	// determine the number of each component
	int numMarkers = sphMarkersH.rhoPresMuH.size();
	thrust::host_vector<int> numComponentMarkers(numMarkers);
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

void ChFsiDataManager::FinalizeDataManager() {
	ConstructReferenceArray();
	int numMarkers = sphMarkersH.rhoPresMuH.size(); // Arman : do it with numObjects and such
	sphMarkersD1.resize(numMarkers);
	sphMarkersD2.resize(numMarkers);
	fsiGeneralData.derivVelRhoD.resize(numMarkers);
	fsiGeneralData.vel_XSPH_D.resize(numMarkers);

	thrust::copy(sphMarkersH.posRadH.begin(), sphMarkersH.posRadH.end(), sphMarkersD1.posRadD.begin());
	thrust::copy(sphMarkersH.velMasH.begin(), sphMarkersH.velMasH.end(), sphMarkersD1.velMasD.begin());
	thrust::copy(sphMarkersH.rhoPresMuH.begin(), sphMarkersH.rhoPresMuH.end(), sphMarkersD1.rhoPresMuD.begin());

	thrust::copy(sphMarkersD1.posRadD.begin(), sphMarkersD1.posRadD.end(), sphMarkersD2.posRadD.begin());
	thrust::copy(sphMarkersD1.velMasD.begin(), sphMarkersD1.velMasD.end(), sphMarkersD2.velMasD.begin());
	thrust::copy(sphMarkersD1.rhoPresMuD.begin(), sphMarkersD1.rhoPresMuD.end(), sphMarkersD2.rhoPresMuD.begin());

	// copy rigids
	int numFsiBodies = fsiBodiesH.posRigid_fsiBodies_H.size(); // Arman : do it with numObjects and such or do it externally
	fsiBodiesD1.resize(numFsiBodies);
	fsiBodiesD2.resize(numFsiBodies);
	fsiGeneralData.rigid_FSI_ForcesD.resize(numFsiBodies);
	fsiGeneralData.rigid_FSI_TorquesD.resize(numFsiBodies);

	// Arman: do it with zip iterator
	thrust::copy(fsiBodiesH.posRigid_fsiBodies_H.begin(), fsiBodiesH.posRigid_fsiBodies_H.end(), fsiBodiesD1.posRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.velMassRigid_fsiBodies_H.begin(), fsiBodiesH.velMassRigid_fsiBodies_H.end(), fsiBodiesD1.velMassRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.accRigid_fsiBodies_H.begin(), fsiBodiesH.accRigid_fsiBodies_H.end(), fsiBodiesD1.accRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.q_fsiBodies_H.begin(), fsiBodiesH.q_fsiBodies_H.end(), fsiBodiesD1.q_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.omegaVelLRF_fsiBodies_H.begin(), fsiBodiesH.omegaVelLRF_fsiBodies_H.end(), fsiBodiesD1.omegaVelLRF_fsiBodies_D.begin());
	thrust::copy(fsiBodiesH.omegaAccLRF_fsiBodies_H.begin(), fsiBodiesH.omegaAccLRF_fsiBodies_H.end(), fsiBodiesD1.omegaAccLRF_fsiBodies_D.begin());

	thrust::copy(fsiBodiesD1.posRigid_fsiBodies_H.begin(), fsiBodiesD1.posRigid_fsiBodies_H.end(), fsiBodiesD2.posRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.velMassRigid_fsiBodies_H.begin(), fsiBodiesD1.velMassRigid_fsiBodies_H.end(), fsiBodiesD2.velMassRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.accRigid_fsiBodies_H.begin(), fsiBodiesD1.accRigid_fsiBodies_H.end(), fsiBodiesD2.accRigid_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.q_fsiBodies_H.begin(), fsiBodiesD1.q_fsiBodies_H.end(), fsiBodiesD1.fsiBodiesD2.begin());
	thrust::copy(fsiBodiesD1.omegaVelLRF_fsiBodies_H.begin(), fsiBodiesD1.omegaVelLRF_fsiBodies_H.end(), fsiBodiesD2.omegaVelLRF_fsiBodies_D.begin());
	thrust::copy(fsiBodiesD1.omegaAccLRF_fsiBodies_H.begin(), fsiBodiesD1.omegaAccLRF_fsiBodies_H.end(), fsiBodiesD2.omegaAccLRF_fsiBodies_D.begin());
}

} // end namespace fsi
} // end namespace chrono

