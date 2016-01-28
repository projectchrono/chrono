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
// Base class for processing the interface between chrono and fsi modules
// =============================================================================
#ifndef CH_FSIINTERFACE_H_
#define CH_FSIINTERFACE_H_

namespace fsi {

class CH_FSI_API ChFsiInterface : public ChFsiGeneral {
	public:
		ChFsiInterface();
		~ChFsiInterface(); // TODO

		void Add_Rigid_ForceTorques_To_ChSystem();
		void Copy_External_To_ChSystem();
		void Copy_ChSystem_to_External();
		void Copy_fsiBodies_ChSystem_to_FluidSystem();
	private:
		FsiBodiesDataH * fsiBodiesH;
		ChronoBodiesDataH * chronoRigidBackup;
		chrono::ChSystemParallelDVI * mphysicalSystem;
		std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisIndex;
		thrust::device_vector<Real3> * rigid_FSI_ForcesD;
		thrust::device_vector<Real3> * rigid_FSI_TorquesD;
};
}

#endif