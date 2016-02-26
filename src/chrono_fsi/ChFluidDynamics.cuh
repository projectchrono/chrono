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
// Class for performing time integration in fluid system.//
// =============================================================================

#ifndef CH_FLUIDDYNAMICS_H_
#define CH_FLUIDDYNAMICS_H_

#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChFsiForceParallel.cuh"

namespace chrono {
namespace fsi {

class CH_FSI_API ChFluidDynamics : public ChFsiGeneral{
public:
	//TODO: Default constructor
	ChFluidDynamics(
		ChBce* otherBceWorker,
		ChFsiDataManager* otherFsiData,
		SimParams* otherParamsH, 
		NumberOfObjects* otherNumObjects);

	~ChFluidDynamics(); //TODO
	
	void IntegrateSPH(
	SphMarkerDataD * sphMarkersD2,
	SphMarkerDataD * sphMarkersD1,
	FsiBodiesDataD * fsiBodiesD1,
	Real dT);

	void DensityReinitialization();

protected:
	ChFsiDataManager* fsiData;
	ChFsiForceParallel* forceSystem;
	SimParams* paramsH;
	NumberOfObjects* numObjectsH;


	void UpdateFluid(SphMarkerDataD * sphMarkersD, Real dT);
	void ApplyBoundarySPH_Markers(SphMarkerDataD * sphMarkersD);

};

} // end namespace fsi
} // end namespace chrono

#endif
