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

namespace fsi {

class CH_FSI_API ChFsiForceParallel : public ChFsiGeneral{
	public:
		// ChFsiForceParallel();
		ChFsiForceParallel(FsiDataContainer* otherFsiData) : fsiData(otherFsiData) {
			fsiCollisionSystem = new ChCollisionSystemFsi(fsiData);
		};
		~ChFsiForceParallel();

		/**
 * @brief Calculates the force on each particles. See collideSphereSphere.cuh for more info.
 * @details See collideSphereSphere.cuh for more info
 */
		void ForceSPH();

		/**
 * @brief Modify BCE velocities for boundary implementation
 * @details 
 */
		void ModifyBceVelocity();

		void DensityReinitialization();

	private:
		ChCollisionSystemFsi* fsiCollisionSystem;
		FsiDataContainer* fsiData;



};
}

#endif /* CH_COLLISIONSYSTEM_FSI_H_ */
