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
// Class for performing time integration in fsi system.//
// =============================================================================

#ifndef CH_TIMEINTEGRATE_FSI_H_
#define CH_TIMEINTEGRATE_FSI_H_

namespace fsi {

class CH_FSI_API ChTimeIntegrateFsi : public ChFsiGeneral{
public:
	//TODO: Default constructor
	ChTimeIntegrateFsi(
		ChFsiDataManager* otherFsiData,
		SimParams* otherParamsH, 
		NumberOfObjects* otherNumObjects);

	~ChTimeIntegrateFsi(); //TODO

protected:
	ChFsiDataManager* fsiData;
	ChFsiForceParallel* forceSystem;


	void IntegrateSPH(
		SphMarkerDataD * sphMarkersD2,
		SphMarkerDataD * sphMarkersD1,
		FsiBodiesDataD * fsiBodiesD1,
		Real dT);
	void UpdateFluid(SphMarkerDataD * sphMarkersD, Real dT);
	void ApplyBoundarySPH_Markers(SphMarkerDataD * sphMarkersD);

}

#endif
