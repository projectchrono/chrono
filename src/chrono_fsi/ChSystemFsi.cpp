// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Implementation of fsi system that includes all subclasses for proximity and
// force calculation, and time integration
// =============================================================================

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChDeviceUtils.cuh"

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::ChSystemFsi(ChSystem* other_physicalSystem, bool other_haveFluid)
    : mphysicalSystem(other_physicalSystem), haveFluid(other_haveFluid), mTime(0) {
    fsiData = new ChFsiDataManager();
    paramsH = new SimParams;
    fsiBodeisPtr.resize(0);
    numObjectsH = &(fsiData->numObjects);

    bceWorker = new ChBce(&(fsiData->sortedSphMarkersD), &(fsiData->markersProximityD), &(fsiData->fsiGeneralData),
                          paramsH, numObjectsH);
    fluidDynamics = new ChFluidDynamics(bceWorker, fsiData, paramsH, numObjectsH);
    fsiInterface =
        new ChFsiInterface(&(fsiData->fsiBodiesH), mphysicalSystem, &fsiBodeisPtr,
                           &(fsiData->fsiGeneralData.rigid_FSI_ForcesD), &(fsiData->fsiGeneralData.rigid_FSI_TorquesD));
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Finalize() {
    FinalizeData();
    if (haveFluid) {
        bceWorker->Finalize(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1));
        fluidDynamics->Finalize();
    }
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
    thrust::copy(fsiData->sphMarkersD1.posRadD.begin(), fsiData->sphMarkersD1.posRadD.end(),
                 fsiData->sphMarkersD2.posRadD.begin());
    thrust::copy(fsiData->sphMarkersD1.velMasD.begin(), fsiData->sphMarkersD1.velMasD.end(),
                 fsiData->sphMarkersD2.velMasD.begin());
    thrust::copy(fsiData->sphMarkersD1.rhoPresMuD.begin(), fsiData->sphMarkersD1.rhoPresMuD.end(),
                 fsiData->sphMarkersD2.rhoPresMuD.begin());
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::DoStepDynamics_FSI() {
    fsiInterface->Copy_ChSystem_to_External();
    this->CopyDeviceDataToHalfStep();
    ChDeviceUtils::FillMyThrust4(fsiData->fsiGeneralData.derivVelRhoD, mR4(0));
    fluidDynamics->IntegrateSPH(&(fsiData->sphMarkersD2), &(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1),
                                0.5 * paramsH->dT);

    bceWorker->Rigid_Forces_Torques(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1));
    fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();
    mTime += 0.5 * paramsH->dT;

    mphysicalSystem->DoStepDynamics(0.5 * paramsH->dT);

    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD2));
    bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));

    fluidDynamics->IntegrateSPH(&(fsiData->sphMarkersD1), &(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2),
                                paramsH->dT);

    bceWorker->Rigid_Forces_Torques(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));

    fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

    mTime -= 0.5 * paramsH->dT;
    fsiInterface->Copy_External_To_ChSystem();
    mTime += paramsH->dT;

    mphysicalSystem->DoStepDynamics(1.0 * paramsH->dT);
    //
    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD1));
    bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1));

    // Density re-initialization
    int tStep = mTime / paramsH->dT;
    if ((tStep % 10 == 0) && (paramsH->densityReinit != 0)) {
        fluidDynamics->DensityReinitialization();
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::DoStepDynamics_ChronoRK2() {
    fsiInterface->Copy_ChSystem_to_External();
    mTime += 0.5 * paramsH->dT;

    mphysicalSystem->DoStepDynamics(0.5 * paramsH->dT);
    mTime -= 0.5 * paramsH->dT;
    fsiInterface->Copy_External_To_ChSystem();
    mTime += paramsH->dT;
    mphysicalSystem->DoStepDynamics(1.0 * paramsH->dT);
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::FinalizeData() {
    fsiData->ResizeDataManager();
    // Important note: the order of (1-3) cannot be change. Needs to be fixed
    fsiInterface->ResizeChronoBodiesData();
    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD1));  //(1)
    fsiData->fsiBodiesD2 = fsiData->fsiBodiesD1;                                    //(2) construct midpoint rigid data
}
//--------------------------------------------------------------------------------------------------------------------------------


}  // end namespace fsi
}  // end namespace chrono

