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
	struct FsiDataContainer {

		// host
		thrust::host_vector<::int4> referenceArray;
		thrust::host_vector<Real3> posRadH; // do not set the size here since you are using push back later
		thrust::host_vector<Real3> velMasH;
		thrust::host_vector<Real4> rhoPresMuH;
		thrust::host_vector<uint> bodyIndex;

		thrust::host_vector<Real3> pos_ChSystemBackupH;
		thrust::host_vector<Real3> vel_ChSystemBackupH;
		thrust::host_vector<Real3> acc_ChSystemBackupH;
		thrust::host_vector<Real4> quat_ChSystemBackupH;
		thrust::host_vector<Real3> omegaVelGRF_ChSystemBackupH;
		thrust::host_vector<Real3> omegaAccGRF_ChSystemBackupH;

		std::vector<ChSharedPtr<ChBody> > FSI_Bodies;

		thrust::host_vector<Real3> posRigid_fsiBodies_dummyH;
		thrust::host_vector<Real4> velMassRigid_fsiBodies_dummyH;
		thrust::host_vector<Real3> accRigid_fsiBodies_dummyH;
		thrust::host_vector<Real4> q_fsiBodies_dummyH;
		thrust::host_vector<Real3> omegaVelLRF_fsiBodies_dummyH;
		thrust::host_vector<Real3> omegaAccLRF_fsiBodies_dummyH;

		// device
		thrust::device_vector<Real3> posRadD;
		thrust::device_vector<Real3> velMasD;
		thrust::device_vector<Real4> rhoPresMuD;
		thrust::device_vector<uint> bodyIndexD;
		thrust::device_vector<Real4> derivVelRhoD;

		thrust::device_vector<Real3> posRigid_fsiBodies_D;
		thrust::device_vector<Real4> velMassRigid_fsiBodies_D;
		thrust::device_vector<Real3> accRigid_fsiBodies_D;
		thrust::device_vector<Real4> q_fsiBodies_D;
		thrust::device_vector<Real3> omegaVelLRF_fsiBodies_D;
		thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D;

		thrust::device_vector<Real3> posRigid_fsiBodies_D2;
		thrust::device_vector<Real4> velMassRigid_fsiBodies_D2;
		thrust::device_vector<Real3> accRigid_fsiBodies_D2;

		thrust::device_vector<Real4> q_fsiBodies_D2;
		thrust::device_vector<Real3> omegaVelLRF_fsiBodies_D2;
		thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D2;

		thrust::device_vector<Real3> rigid_FSI_ForcesD;
		thrust::device_vector<Real3> rigid_FSI_TorquesD;
	};
}





#endif /* CH_FSI_DATAMANAGER_H_ */
