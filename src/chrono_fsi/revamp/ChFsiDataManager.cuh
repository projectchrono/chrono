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

#ifndef CH_FSI_DATAMANAGER_H_
#define CH_FSI_DATAMANAGER_H_

namespace fsi {
	struct SphMarkerDataD {
		thrust::device_vector<Real3> posRadD;
		thrust::device_vector<Real3> velMasD;
		thrust::device_vector<Real4> rhoPresMuD;
	}

	struct SphMarkerDataH {
		thrust::host_vector<Real3> posRadH; // do not set the size here since you are using push back later
		thrust::host_vector<Real3> velMasH;
		thrust::host_vector<Real4> rhoPresMuH;
	}

	struct FsiBodiesDataD {
		thrust::device_vector<Real3> posRigid_fsiBodies_D;
		thrust::device_vector<Real4> velMassRigid_fsiBodies_D;
		thrust::device_vector<Real3> accRigid_fsiBodies_D;
		thrust::device_vector<Real4> q_fsiBodies_D;
		thrust::device_vector<Real3> omegaVelLRF_fsiBodies_D;
		thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D;
	}

	// dummy fsi bodies
	struct FsiBodiesDataH {
		thrust::host_vector<Real3> posRigid_fsiBodies_H;
		thrust::host_vector<Real4> velMassRigid_fsiBodies_H;
		thrust::host_vector<Real3> accRigid_fsiBodies_H;
		thrust::host_vector<Real4> q_fsiBodies_H;
		thrust::host_vector<Real3> omegaVelLRF_fsiBodies_H;
		thrust::host_vector<Real3> omegaAccLRF_fsiBodies_H;
	}

	struct ProximityDataD {
		thrust::device_vector<uint> gridMarkerHashD;//(numAllMarkers);
		thrust::device_vector<uint> gridMarkerIndexD;//(numAllMarkers);
		thrust::device_vector<uint> cellStartD;//(m_numGridCells); // Index of start cell in sorted list
		thrust::device_vector<uint> cellEndD;//(m_numGridCells); // Index of end cell in sorted list
		thrust::device_vector<uint> mapOriginalToSorted;
	}

	struct ChronoBodiesDataH {
		thrust::host_vector<Real3> pos_ChSystemH;
		thrust::host_vector<Real3> vel_ChSystemH;
		thrust::host_vector<Real3> acc_ChSystemH;
		thrust::host_vector<Real4> quat_ChSystemH;
		thrust::host_vector<Real3> omegaVelGRF_ChSystemH;
		thrust::host_vector<Real3> omegaAccGRF_ChSystemH;
	}

// make them classes
	struct FsiGeneralData {

		// ----------------
		//  host
		// ----------------
		// fluidfsiBodeisIndex
		thrust::host_vector<::int4> referenceArray;
		thrust::host_vector<uint> bodyIndexH;

		// map fsi to chrono bodies
		std::vector<ChSharedPtr<ChBody> > fsiBodeisPtr;

		// ----------------
		//  device
		// ----------------
		// fluid
		thrust::device_vector<uint> bodyIndexD;
		thrust::device_vector<Real4> derivVelRhoD;
		thrust::device_vector<Real3> vel_XSPH_D;

		// BCE
		thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D;
		thrust::device_vector<uint> rigidIdentifierD;

		// fsi bodies
		thrust::device_vector<Real3> rigid_FSI_ForcesD;
		thrust::device_vector<Real3> rigid_FSI_TorquesD;
	};

class CH_FSI_API ChFsiDataManager {
public:
	ChFsiDataManager();
	~ChFsiDataManager();

	SphMarkerDataD sphMarkersD1;
	SphMarkerDataD sphMarkersD2;
	SphMarkerDataD sortedSphMarkersD;
	SphMarkerDataH sphMarkersH;

	FsiBodiesDataD fsiBodiesD1;
	FsiBodiesDataD fsiBodiesD2;
	FsiBodiesDataH fsiBodiesH;
	ChronoBodiesDataH chronoRigidBackup;

	FsiGeneralData fsiGeneralData;

	ProximityDataD markersProximityD;isSolid
private:
}

}





#endif /* CH_FSI_DATAMANAGER_H_ */
