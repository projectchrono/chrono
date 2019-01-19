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

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::ChSystemFsi(ChSystem* other_physicalSystem, bool other_haveFluid, ChFluidDynamics::Integrator type)
    : mphysicalSystem(other_physicalSystem), haveFluid(other_haveFluid), mTime(0) {
    fsiData = new ChFsiDataManager();
    paramsH = new SimParams;
    numObjectsH = &(fsiData->numObjects);
    fluidIntegrator = type;
    bceWorker = new ChBce(&(fsiData->sortedSphMarkersD), &(fsiData->markersProximityD), &(fsiData->fsiGeneralData),
                          paramsH, numObjectsH);
    fluidDynamics = new ChFluidDynamics(bceWorker, fsiData, paramsH, numObjectsH, fluidIntegrator);

    fsi_mesh = std::make_shared<fea::ChMesh>();
    fsiBodeisPtr.resize(0);
    fsiShellsPtr.resize(0);
    fsiCablesPtr.resize(0);
    fsiNodesPtr.resize(0);
    fsiInterface = new ChFsiInterface(
        paramsH, &(fsiData->fsiBodiesH), &(fsiData->fsiMeshH), mphysicalSystem, &fsiBodeisPtr, &fsiNodesPtr,
        &fsiCablesPtr, &fsiShellsPtr, fsi_mesh, &(fsiData->fsiGeneralData.CableElementsNodesH),
        &(fsiData->fsiGeneralData.CableElementsNodes), &(fsiData->fsiGeneralData.ShellElementsNodesH),
        &(fsiData->fsiGeneralData.ShellElementsNodes), &(fsiData->fsiGeneralData.rigid_FSI_ForcesD),
        &(fsiData->fsiGeneralData.rigid_FSI_TorquesD), &(fsiData->fsiGeneralData.Flex_FSI_ForcesD));
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Finalize() {
    printf("\n\nChSystemFsi::Finalize 1-FinalizeData\n");
    FinalizeData();

    if (haveFluid) {
        printf("\n\nChSystemFsi::Finalize 2-bceWorker->Finalize\n");
        bceWorker->Finalize(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1), &(fsiData->fsiMeshD));
        printf("\n\nChSystemFsi::Finalize 3-fluidDynamics->Finalize\n");
        fluidDynamics->Finalize();
        std::cout << "referenceArraySize in 3-fluidDynamics->Finalize"
                  << GetDataManager()->fsiGeneralData.referenceArray.size() << "\n";
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

void ChSystemFsi::SetFluidSystemLinearSolver(ChFsiLinearSolver::SolverType other_solverType) {
    fluidDynamics->GetForceSystem()->SetLinearSolver(other_solverType);
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::DoStepDynamics_FSI() {
    /// The following is used to execute the previous Explicit SPH
    if (fluidDynamics->GetIntegratorType() == ChFluidDynamics::Integrator::ExplicitSPH) {
        fsiInterface->Copy_ChSystem_to_External();
        this->CopyDeviceDataToHalfStep();
        ChDeviceUtils::FillMyThrust4(fsiData->fsiGeneralData.derivVelRhoD, mR4(0));
        fluidDynamics->IntegrateSPH(&(fsiData->sphMarkersD2), &(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1),
                                    &(fsiData->fsiMeshD), 0.5 * paramsH->dT);

        bceWorker->Rigid_Forces_Torques(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1));
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();
        mTime += 0.5 * paramsH->dT;

        mphysicalSystem->DoStepDynamics(0.5 * paramsH->dT);

        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD2));
        bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));

        fluidDynamics->IntegrateSPH(&(fsiData->sphMarkersD1), &(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2),
                                    &(fsiData->fsiMeshD), 0.5 * paramsH->dT);

        bceWorker->Rigid_Forces_Torques(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));

        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        mTime -= 0.5 * paramsH->dT;
        fsiInterface->Copy_External_To_ChSystem();
        mTime += paramsH->dT;

        mphysicalSystem->DoStepDynamics(0.5 * paramsH->dT);
        //
        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD1));
        bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD1), &(fsiData->fsiBodiesD1));

        // Density re-initialization
        int tStep = mTime / paramsH->dT;
        if ((tStep % (paramsH->densityReinit + 1) == 0)) {
            fluidDynamics->DensityReinitialization();
        }
    } else {
        /// A different coupling scheme is used for implicit SPH formulations
        printf("Copy_ChSystem_to_External\n");
        fsiInterface->Copy_ChSystem_to_External();
        printf("IntegrateIISPH\n");
        fluidDynamics->IntegrateSPH(&(fsiData->sphMarkersD2), &(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2),
                                    &(fsiData->fsiMeshD), 0.0);
        printf("Fluid-structure forces\n");
        bceWorker->Rigid_Forces_Torques(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(&(fsiData->sphMarkersD2), &(fsiData->fsiMeshD));
        // Note that because of applying forces to the nodal coordinates using SetForce() no other external forces can
        // be applied, or if any thing has been applied will be rewritten by Add_Flex_Forces_To_ChSystem();
        fsiInterface->Add_Flex_Forces_To_ChSystem();

        // paramsH->dT_Flex = paramsH->dT;
        mTime += 1 * paramsH->dT;
        if (paramsH->dT_Flex == 0)
            paramsH->dT_Flex = paramsH->dT;
        int sync = (paramsH->dT / paramsH->dT_Flex);
        if (sync < 1)
            sync = 1;
        printf("%d * DoStepChronoSystem with dt= %f\n", sync, paramsH->dT / sync);
        for (int t = 0; t < sync; t++) {
            mphysicalSystem->DoStepDynamics(paramsH->dT / sync);
        }

        printf("Update Rigid Marker\n");
        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD2));
        bceWorker->UpdateRigidMarkersPositionVelocity(&(fsiData->sphMarkersD2), &(fsiData->fsiBodiesD2));

        printf("Update Flexible Marker\n");
        fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(&(fsiData->fsiMeshD));
        bceWorker->UpdateFlexMarkersPositionVelocity(&(fsiData->sphMarkersD2), &(fsiData->fsiMeshD));
        printf("=================================================================================================\n");
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

void SetIntegratorType(ChFluidDynamics::Integrator type) {}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::FinalizeData() {
    printf("\n\nfsiInterface->ResizeChronoBodiesData()\n");
    fsiInterface->ResizeChronoBodiesData();
    int fea_node = 0;
    fsiInterface->ResizeChronoCablesData(CableElementsNodes, &(fsiData->fsiGeneralData.CableElementsNodesH));
    fsiInterface->ResizeChronoShellsData(ShellElementsNodes, &(fsiData->fsiGeneralData.ShellElementsNodesH));
    fsiInterface->ResizeChronoFEANodesData();
    printf("\nfsiData->ResizeDataManager...\n");
    fea_node = fsi_mesh->GetNnodes();

    fsiData->ResizeDataManager(fea_node);

    printf("\n\nfsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem()\n");
    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(&(fsiData->fsiBodiesD1));

    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(&(fsiData->fsiMeshD));

    std::cout << "referenceArraySize in FinalizeData " << GetDataManager()->fsiGeneralData.referenceArray.size()
              << "\n";
    fsiData->fsiBodiesD2 = fsiData->fsiBodiesD1;  //(2) construct midpoint rigid data
}
//--------------------------------------------------------------------------------------------------------------------------------

}  // end namespace fsi
}  // end namespace chrono
