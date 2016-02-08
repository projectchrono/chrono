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
// Base class for processing sph force in fsi system.//
// =============================================================================

#ifndef CH_FSI_FORCEPARALLEL_H_
#define CH_FSI_FORCEPARALLEL_H_

#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChCollisionSystemFsi.cuh"

namespace chrono {
namespace fsi {

class CH_FSI_API ChFsiForceParallel : public ChFsiGeneral{
	public:
		ChFsiForceParallel(
			SphMarkerDataD * otherSortedSphMarkersD,
			ProximityDataD * otherMarkersProximityD,
			FsiGeneralData * otherFsiGeneralData,
			SimParams* otherParamsH, 
			NumberOfObjects* otherNumObjects);
		~ChFsiForceParallel();



		ChFsiForceParallel(FsiDataContainer* otherFsiData) : fsiData(otherFsiData) {
			fsiCollisionSystem = new ChCollisionSystemFsi(fsiData);
		};
		~ChFsiForceParallel();

		/**
 * @brief Calculates the force on each particles. See collideSphereSphere.cuh for more info.
 * @details See collideSphereSphere.cuh for more info
 */
		void ForceSPH(
		SphMarkerDataD * otherSphMarkersD,
		FsiBodiesDataD * otherFsiBodiesD);
	private:

	void ModifyBceVelocity();
	void CalculateXSPH_velocity();
	void CollideWrapper();
	void DensityReinitialization(); // Arman : TODO
// TODO : make these four dudes static in ChCollisionSystemFsi
	void CopySortedToOriginal_Invasive_R3(thrust::device_vector<Real3>& original,
		thrust::device_vector<Real3>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex);
	void CopySortedToOriginal_NonInvasive_R3(thrust::device_vector<Real3>& original,
		thrust::device_vector<Real3>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex);
	void CopySortedToOriginal_Invasive_R4(thrust::device_vector<Real4>& original,
			thrust::device_vector<Real4>& sorted,
			const thrust::device_vector<uint>& gridMarkerIndex);
	void CopySortedToOriginal_NonInvasive_R4(thrust::device_vector<Real4>& original,
			thrust::device_vector<Real4>& sorted,
			const thrust::device_vector<uint>& gridMarkerIndex);
///////////////////////////////////////////////////////////////////



		ChCollisionSystemFsi* fsiCollisionSystem;

		SphMarkerDataD * sphMarkersD;
		SphMarkerDataD * sortedSphMarkersD;
		ProximityDataD * markersProximityD;
		FsiBodiesDataD * fsiBodiesD;
		FsiGeneralData * fsiGeneralData;



		thrust::device_vector<Real3> velMas_ModifiedBCE;//(numRigidAndBoundaryMarkers);
		thrust::device_vector<Real4> rhoPreMu_ModifiedBCE;//(numRigidAndBoundaryMarkers);
		thrust::device_vector<Real3> vel_XSPH_Sorted_D;


		void collide(thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<Real3>& velMas_ModifiedBCE,
		thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,

		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers, uint numCells,
		Real dT);

		void RecalcVelocity_XSPH(thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers,
		uint numCells);



};
} // end namespace fsi
} // end namespace chrono
#endif /* CH_COLLISIONSYSTEM_FSI_H_ */
