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
//--------------------------------------------------------------------------------------------------------------------------------
// Arman : split this later. move vehicle stuff out of this class.
int DoStepChronoSystem(Real dT,
		double mTime, double time_hold_vehicle, bool haveVehicle) {
	if (haveVehicle) {
		// Release the vehicle chassis at the end of the hold time.

		if (mVehicle->GetVehicle()->GetChassis()->GetBodyFixed()
				&& mTime > time_hold_vehicle) {
			mVehicle->GetVehicle()->GetChassis()->SetBodyFixed(false);
			for (int i = 0; i < 2 * mVehicle->GetVehicle()->GetNumberAxles();
					i++) {
				mVehicle->GetVehicle()->GetWheelBody(i)->SetBodyFixed(false);
			}
		}

		// Update vehicle
		mVehicle->Update(mTime);
	}

#ifdef CHRONO_OPENGL
	if (gl_window.Active()) {
		gl_window.DoStepDynamics(dT);
		gl_window.Render();
	}
#else
	mphysicalSystem->DoStepDynamics(dT);
#endif
	return 1;
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiDynamics::DoStepDynamics_FSI(){

	ChFsiInterface->Copy_ChSystem_to_External();
	this->CopyDeviceDataToHalfStep();
	ChDeviceUtils::FillMyThrust4(fsiData->FsiGeneralData.derivVelRhoD);
	ChFluidDynamics->IntegrateSPH(
		fsiData->sphMarkersD2,
		fsiData->sphMarkersD1,
		fsiData->fsiBodiesD1,
		0.5 * paramsH.dT);

	bceWorker->Rigid_Forces_Torques(fsiData->sphMarkersD1, fsiData->fsiBodiesD1);
	ChFsiInterface->Add_Rigid_ForceTorques_To_ChSystem();
	mTime += 0.5 * paramsH.dT;


	// TODO
		DoStepChronoSystem(mphysicalSystem, mVehicle, 0.5 * paramsH.dT, mTime,
				time_hold_vehicle, haveVehicle); // Keep only this if you are just interested in the rigid sys
	//
	ChFsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiData->fsiBodiesD2);
	bceWorker->UpdateRigidMarkersPositionVelocity(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);



	ChFluidDynamics->IntegrateSPH(
		fsiData->sphMarkersD1,
		fsiData->sphMarkersD2,
		fsiData->fsiBodiesD2,
		paramsH.dT);
	bceWorker->Rigid_Forces_Torques(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);
	ChFsiInterface->Add_Rigid_ForceTorques_To_ChSystem();
	mTime -= 0.5 * paramsH.dT;
	ChFsiInterface->Copy_External_To_ChSystem();
	mTime += paramsH.dT;

	// TODO
		DoStepChronoSystem(mphysicalSystem, mVehicle, 1.0 * paramsH.dT, mTime,
			time_hold_vehicle, haveVehicle);
	//
	ChFsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiData->fsiBodiesD1);
	bceWorker->UpdateRigidMarkersPositionVelocity(fsiData->sphMarkersD1, fsiData->fsiBodiesD1);


	// TODO
	if ((tStep % 10 == 0) && (paramsH.densityReinit != 0)) {
		DensityReinitialization(posRadD, velMasD, rhoPresMuD,
				numObjects.numAllMarkers, paramsH.gridSize);
	}

}