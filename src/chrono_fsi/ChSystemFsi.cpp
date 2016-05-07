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
// Class for performing handling fsi system.
// =============================================================================

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChDeviceUtils.cuh" 


#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
chrono::opengl::ChOpenGLWindow& gl_window =
		chrono::opengl::ChOpenGLWindow::getInstance();
#endif

namespace chrono {
namespace fsi {

// Arman: have a default constructor where you create mphysical system.
// Arman: have a function to set mphysical system
//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::ChSystemFsi(ChSystemParallelDVI * other_physicalSystem) : mphysicalSystem(other_physicalSystem), mTime(0), haveVehicle(false), mVehicle(NULL) {
	fsiData = new ChFsiDataManager();
	fsiBodeisPtr.resize(0);
	paramsH = new SimParams; // Arman: define a function to set paramsH default values
	numObjectsH = &(fsiData->numObjects);

	bceWorker = new ChBce(&(fsiData->fsiGeneralData), paramsH, numObjectsH);
	fluidDynamics = new ChFluidDynamics(bceWorker, fsiData, paramsH, numObjectsH);
	fsiInterface = new ChFsiInterface(&(fsiData->fsiBodiesH),
		mphysicalSystem, &fsiBodeisPtr,
		&(fsiData->fsiGeneralData.rigid_FSI_ForcesD),
		&(fsiData->fsiGeneralData.rigid_FSI_TorquesD));

	InitializeChronoGraphics();
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Finalize() {
	printf("** 21\n");
	FinalizeData();
	printf("** 22\n");
	bceWorker->Finalize();
	printf("** 23\n");
	fluidDynamics->Finalize();
	printf("** 24\n");
}
//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::~ChSystemFsi() {
	delete fsiData;
	delete paramsH;
	delete bceWorker;
	delete fluidDynamics;
	delete fsiInterface;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::CopyDeviceDataToHalfStep() {	
	thrust::copy(fsiData->sphMarkersD1.posRadD.begin(), fsiData->sphMarkersD1.posRadD.end(), fsiData->sphMarkersD2.posRadD.begin());
	thrust::copy(fsiData->sphMarkersD1.velMasD.begin(), fsiData->sphMarkersD1.velMasD.end(), fsiData->sphMarkersD2.velMasD.begin());
	thrust::copy(fsiData->sphMarkersD1.rhoPresMuD.begin(), fsiData->sphMarkersD1.rhoPresMuD.end(), fsiData->sphMarkersD2.rhoPresMuD.begin());
}
//--------------------------------------------------------------------------------------------------------------------------------
// Arman : split this later. move vehicle stuff out of this class.
int ChSystemFsi::DoStepChronoSystem(Real dT,
		double mTime) {
	if (haveVehicle) {
		// Release the vehicle chassis at the end of the hold time.

		if (mVehicle->GetVehicle()->GetChassis()->GetBodyFixed()) {
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

void ChSystemFsi::DoStepDynamics_FSI(){

	fsiInterface->Copy_ChSystem_to_External();
	this->CopyDeviceDataToHalfStep();
	ChDeviceUtils::FillMyThrust4(fsiData->fsiGeneralData.derivVelRhoD, mR4(0));
	fluidDynamics->IntegrateSPH(
		&(fsiData->sphMarkersD2),
		&(fsiData->sphMarkersD1),
		&(fsiData->fsiBodiesD1),
		0.5 * paramsH->dT);
	int tStep = mTime / paramsH->dT;

	bceWorker->Rigid_Forces_Torques(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1));
	fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();
	mTime += 0.5 * paramsH->dT;


	// TODO
		DoStepChronoSystem(0.5 * paramsH->dT, mTime); // Keep only this if you are just interested in the rigid sys
	//
		
	fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD2));
	bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));

	fluidDynamics->IntegrateSPH(
		&(fsiData->sphMarkersD1),
		&(fsiData->sphMarkersD2),
		&(fsiData->fsiBodiesD2),
		paramsH->dT);
	bceWorker->Rigid_Forces_Torques(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));
	fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();
	mTime -= 0.5 * paramsH->dT;
	fsiInterface->Copy_External_To_ChSystem();
	mTime += paramsH->dT;

	// TODO
		DoStepChronoSystem(1.0 * paramsH->dT, mTime);
	//
	fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD1));
	bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1));


	// TODO
	if ((tStep % 10 == 0) && (paramsH->densityReinit != 0)) {
		fluidDynamics->DensityReinitialization();
	}

}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::DoStepDynamics_ChronoRK2() {
	fsiInterface->Copy_ChSystem_to_External();
	mTime += 0.5 * paramsH->dT;

	DoStepChronoSystem(0.5 * paramsH->dT, mTime); // Keep only this if you are just interested in the rigid sys
	mTime -= 0.5 * paramsH->dT;
	fsiInterface->Copy_External_To_ChSystem();
	mTime += paramsH->dT;

	DoStepChronoSystem(1.0 * paramsH->dT, mTime);
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::SetVehicle(chrono::vehicle::ChWheeledVehicleAssembly* other_mVehicle) {
	mVehicle = other_mVehicle;
	haveVehicle = true;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::FinalizeData() {
	printf("** 210\n");
	fsiData->ResizeDataManager();
	printf("** 211\n");
	// Arman: very important: you cannot change the order of (1-3). Fix the issue later
	fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD1)); //(1)
	printf("** 212\n");
	fsiData->CopyFsiBodiesDataH2D();	// (2)
	printf("** 213\n");
	bceWorker->Populate_RigidSPH_MeshPos_LRF(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1)); // (3)
	printf("** 214\n");
	bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1)); //(4)
	printf("** 215\n");
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::InitializeChronoGraphics(chrono::ChVector<> CameraLocation, chrono::ChVector<> CameraLookAt) {
#ifdef CHRONO_OPENGL
	gl_window.Initialize(1280, 720, "FSI_Problem", mphysicalSystem);
	gl_window.SetCamera(CameraLocation, CameraLookAt,
			chrono::ChVector<>(0, 0, 1));
	gl_window.SetRenderMode(chrono::opengl::WIREFRAME);
#endif
}

} // end namespace fsi
} // end namespace chrono
