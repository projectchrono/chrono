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

void ChFsiDynamics::CopyDeviceDataToHalfStep(){
	
	thrust::copy(fsiData->sphMarkersD1.posRadD.begin(), fsiData->sphMarkersD1.posRadD.end(), fsiData->sphMarkersD2.posRadD.begin());
	thrust::copy(fsiData->sphMarkersD1.velMasD.begin(), fsiData->sphMarkersD1.velMasD.end(), fsiData->sphMarkersD2.velMasD.begin());
	thrust::copy(fsiData->sphMarkersD1.rhoPresMuD.begin(), fsiData->sphMarkersD1.rhoPresMuD.end(), fsiData->sphMarkersD2.rhoPresMuD.begin());
}

void ChFsiDynamics::DoStepDynamics_FSI(){
	ChFsiInterface->Copy_ChSystem_to_External();
	this->CopyDeviceDataToHalfStep();
	ChDeviceUtils::FillMyThrust4(fsiData->FsiGeneralData.derivVelRhoD);
	ChFluidDynamics->IntegrateSPH(
		fsiData->sphMarkersD2,
		fsiData->sphMarkersD1,
		fsiData->fsiBodiesD1,
		0.5 * paramsH.dT);

	// left off here
}