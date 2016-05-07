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

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiGeneral.cuh"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_parallel/physics/ChSystemParallel.h"

namespace chrono {
namespace fsi {

class CH_FSI_API ChFsiInterface : public ChFsiGeneral {
	public:
		ChFsiInterface(
			FsiBodiesDataH * other_fsiBodiesH,
			chrono::ChSystemParallelDVI * other_mphysicalSystem,
			std::vector<chrono::ChSharedPtr<chrono::ChBody> > * other_fsiBodeisPtr,
			thrust::device_vector<Real3> * other_rigid_FSI_ForcesD,
			thrust::device_vector<Real3> * other_rigid_FSI_TorquesD);
		~ChFsiInterface(); // TODO

		void Add_Rigid_ForceTorques_To_ChSystem();
		void Copy_External_To_ChSystem();
		void Copy_ChSystem_to_External();
		void Copy_fsiBodies_ChSystem_to_FluidSystem(FsiBodiesDataD * fsiBodiesD);
	private:
		FsiBodiesDataH * fsiBodiesH;
		ChronoBodiesDataH * chronoRigidBackup;
		chrono::ChSystemParallelDVI * mphysicalSystem;
		std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisPtr;
		thrust::device_vector<Real3> * rigid_FSI_ForcesD;
		thrust::device_vector<Real3> * rigid_FSI_TorquesD;
};
} // end namespace fsi
} // end namespace chrono
#endif
