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
// Author: Milad Rakhsha, Arman Pazouki
// =============================================================================
//
// Implementation of fsi system that includes all subclasses for proximity and
// force calculation, and time integration
// =============================================================================
#include "chrono/core/ChTypes.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::ChSystemFsi(ChSystem& other_physicalSystem, ChFluidDynamics::Integrator type)
    : mphysicalSystem(other_physicalSystem), mTime(0), fluidIntegrator(type) {
    fsiData = chrono_types::make_shared<ChFsiDataManager>();
    paramsH = chrono_types::make_shared<SimParams>();
    numObjectsH = fsiData->numObjects;
    bceWorker = chrono_types::make_shared<ChBce>(fsiData->sortedSphMarkersD, fsiData->markersProximityD,
                                                 fsiData->fsiGeneralData, paramsH, numObjectsH);
    fluidDynamics = chrono_types::make_shared<ChFluidDynamics>(bceWorker, fsiData, paramsH, numObjectsH, type);

    fsi_mesh = chrono_types::make_shared<fea::ChMesh>();
    fsiBodeis.resize(0);
    fsiShells.resize(0);
    fsiCables.resize(0);
    fsiNodes.resize(0);
    fsiInterface = chrono_types::make_shared<ChFsiInterface>(
        mphysicalSystem, fsi_mesh, paramsH, fsiData->fsiBodiesH, fsiData->fsiMeshH, fsiBodeis, fsiNodes, fsiCables,
        fsiShells, fsiData->fsiGeneralData->CableElementsNodesH, fsiData->fsiGeneralData->CableElementsNodes,
        fsiData->fsiGeneralData->ShellElementsNodesH, fsiData->fsiGeneralData->ShellElementsNodes,
        fsiData->fsiGeneralData->rigid_FSI_ForcesD, fsiData->fsiGeneralData->rigid_FSI_TorquesD,
        fsiData->fsiGeneralData->Flex_FSI_ForcesD);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFluidIntegratorType(fluid_dynamics params_type) {
    if (params_type == fluid_dynamics::IISPH) {
        fluidIntegrator = ChFluidDynamics::Integrator::IISPH;
        std::cout << "fluid dynamics is reset to IISPH" << std::endl;
    } else if (params_type == fluid_dynamics::WCSPH) {
        fluidIntegrator = ChFluidDynamics::Integrator::ExplicitSPH;
        std::cout << "fluid dynamics is reset to Explicit WCSPH" << std::endl;
    } else {
        fluidIntegrator = ChFluidDynamics::Integrator::I2SPH;
        std::cout << "fluid dynamics is reset to I2SPH" << std::endl;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFluidDynamics(fluid_dynamics params_type) {
    SetFluidIntegratorType(params_type);
    fluidDynamics =
        chrono_types::make_shared<ChFluidDynamics>(bceWorker, fsiData, paramsH, numObjectsH, fluidIntegrator);
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Finalize() {
    printf("\n\nChSystemFsi::Finalize 1-FinalizeData\n");
    FinalizeData();
    printf("\n\nChSystemFsi::Finalize 2-bceWorker->Finalize\n");
    bceWorker->Finalize(fsiData->sphMarkersD1, fsiData->fsiBodiesD1, fsiData->fsiMeshD);
    printf("\n\nChSystemFsi::Finalize 3-fluidDynamics->Finalize\n");
    fluidDynamics->Finalize();
    std::cout << "referenceArraySize in 3-fluidDynamics->Finalize"
              << GetDataManager()->fsiGeneralData->referenceArray.size() << "\n";
}
//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::~ChSystemFsi() {}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::CopyDeviceDataToHalfStep() {
    thrust::copy(fsiData->sphMarkersD2->posRadD.begin(), fsiData->sphMarkersD2->posRadD.end(),
                 fsiData->sphMarkersD1->posRadD.begin());
    thrust::copy(fsiData->sphMarkersD2->velMasD.begin(), fsiData->sphMarkersD2->velMasD.end(),
                 fsiData->sphMarkersD1->velMasD.begin());
    thrust::copy(fsiData->sphMarkersD2->rhoPresMuD.begin(), fsiData->sphMarkersD2->rhoPresMuD.end(),
                 fsiData->sphMarkersD1->rhoPresMuD.begin());
    thrust::copy(fsiData->sphMarkersD2->tauXxYyZzD.begin(), fsiData->sphMarkersD2->tauXxYyZzD.end(),
                 fsiData->sphMarkersD1->tauXxYyZzD.begin());
    thrust::copy(fsiData->sphMarkersD2->tauXyXzYzD.begin(), fsiData->sphMarkersD2->tauXyXzYzD.end(),
                 fsiData->sphMarkersD1->tauXyXzYzD.begin());
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::DoStepDynamics_FSI() {
    /// The following is used to execute the Explicit WCSPH
    if (fluidDynamics->GetIntegratorType() == ChFluidDynamics::Integrator::ExplicitSPH) {
        fsiInterface->Copy_ChSystem_to_External();
        CopyDeviceDataToHalfStep();
        ChUtilsDevice::FillMyThrust3(fsiData->fsiGeneralData->derivTauXxYyZzD, mR3(0));
        ChUtilsDevice::FillMyThrust3(fsiData->fsiGeneralData->derivTauXyXzYzD, mR3(0));
        ChUtilsDevice::FillMyThrust4(fsiData->fsiGeneralData->derivVelRhoD, mR4(0));
        fluidDynamics->IntegrateSPH(fsiData->sphMarkersD2, fsiData->sphMarkersD1, fsiData->fsiBodiesD2,
                                    fsiData->fsiMeshD, 0.5 * paramsH->dT);
        fluidDynamics->IntegrateSPH(fsiData->sphMarkersD1, fsiData->sphMarkersD2, fsiData->fsiBodiesD2,
                                    fsiData->fsiMeshD, 1.0 * paramsH->dT);
        bceWorker->Rigid_Forces_Torques(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(fsiData->sphMarkersD2, fsiData->fsiMeshD);
        // Note that because of applying forces to the nodal coordinates using SetForce() no other external forces can
        // be applied, or if any thing has been applied will be rewritten by Add_Flex_Forces_To_ChSystem();
        fsiInterface->Add_Flex_Forces_To_ChSystem();

        fsiInterface->Copy_External_To_ChSystem();

        // paramsH->dT_Flex = paramsH->dT; 
        // dT_Flex is the time step of solid body system
        mTime += 1 * paramsH->dT;
        if (paramsH->dT_Flex == 0)
            paramsH->dT_Flex = paramsH->dT;
        int sync = int(paramsH->dT / paramsH->dT_Flex);
        if (sync < 1)
            sync = 1;
        // printf("%d * DoStepChronoSystem with dt= %f\n", sync, paramsH->dT / sync);
        for (int t = 0; t < sync; t++) {
            mphysicalSystem.DoStepDynamics(paramsH->dT / sync);
        }

        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiData->fsiBodiesD2);
        bceWorker->UpdateRigidMarkersPositionVelocity(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);
        // fsiData->sphMarkersD1 = fsiData->sphMarkersD2;
        // Density re-initialization
        int tStep = int(mTime / paramsH->dT);
        //        if ((tStep % (paramsH->densityReinit + 1) == 0)) {
        //            fluidDynamics->DensityReinitialization();
        //        }
    } else {
        // A different coupling scheme is used for implicit SPH formulations
        printf("Copy_ChSystem_to_External\n");
        fsiInterface->Copy_ChSystem_to_External();
        printf("IntegrateIISPH\n");
        fluidDynamics->IntegrateSPH(fsiData->sphMarkersD2, fsiData->sphMarkersD2, fsiData->fsiBodiesD2,
                                    fsiData->fsiMeshD, 0.0);
        printf("Fluid-structure forces\n");
        bceWorker->Rigid_Forces_Torques(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(fsiData->sphMarkersD2, fsiData->fsiMeshD);
        // Note that because of applying forces to the nodal coordinates using SetForce() no other external forces can
        // be applied, or if any thing has been applied will be rewritten by Add_Flex_Forces_To_ChSystem();
        fsiInterface->Add_Flex_Forces_To_ChSystem();

        // paramsH->dT_Flex = paramsH->dT;
        mTime += 1 * paramsH->dT;
        if (paramsH->dT_Flex == 0)
            paramsH->dT_Flex = paramsH->dT;
        int sync = int(paramsH->dT / paramsH->dT_Flex);
        if (sync < 1)
            sync = 1;
        printf("%d * DoStepChronoSystem with dt= %f\n", sync, paramsH->dT / sync);
        for (int t = 0; t < sync; t++) {
            mphysicalSystem.DoStepDynamics(paramsH->dT / sync);
        }

        printf("Update Rigid Marker\n");
        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiData->fsiBodiesD2);
        bceWorker->UpdateRigidMarkersPositionVelocity(fsiData->sphMarkersD2, fsiData->fsiBodiesD2);

        printf("Update Flexible Marker\n");
        fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiData->fsiMeshD);
        bceWorker->UpdateFlexMarkersPositionVelocity(fsiData->sphMarkersD2, fsiData->fsiMeshD);

        printf("Update Flexible Marker\n");
        fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiData->fsiMeshD);
        bceWorker->UpdateFlexMarkersPositionVelocity(fsiData->sphMarkersD2, fsiData->fsiMeshD);
        printf("=================================================================================================\n");
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::DoStepDynamics_ChronoRK2() {
    fsiInterface->Copy_ChSystem_to_External();
    mTime += 0.5 * paramsH->dT;

    mphysicalSystem.DoStepDynamics(0.5 * paramsH->dT);
    mTime -= 0.5 * paramsH->dT;
    fsiInterface->Copy_External_To_ChSystem();
    mTime += paramsH->dT;
    mphysicalSystem.DoStepDynamics(1.0 * paramsH->dT);
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::FinalizeData() {
    printf("\n\nfsiInterface->ResizeChronoBodiesData()\n");
    fsiInterface->ResizeChronoBodiesData();
    int fea_node = 0;
    fsiInterface->ResizeChronoCablesData(CableElementsNodes, fsiData->fsiGeneralData->CableElementsNodesH);
    fsiInterface->ResizeChronoShellsData(ShellElementsNodes, fsiData->fsiGeneralData->ShellElementsNodesH);
    fsiInterface->ResizeChronoFEANodesData();
    printf("\nfsiData->ResizeDataManager...\n");
    fea_node = fsi_mesh->GetNnodes();

    fsiData->ResizeDataManager(fea_node);

    printf("\n\nfsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem()\n");
    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiData->fsiBodiesD1);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiData->fsiMeshD);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiData->fsiMeshD);

    std::cout << "referenceArraySize in FinalizeData " << GetDataManager()->fsiGeneralData->referenceArray.size()
              << "\n";
    fsiData->fsiBodiesD2 = fsiData->fsiBodiesD1;  //(2) construct midpoint rigid data
}
//--------------------------------------------------------------------------------------------------------------------------------

}  // end namespace fsi
}  // end namespace chrono
