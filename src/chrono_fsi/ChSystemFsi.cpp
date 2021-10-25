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
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::ChSystemFsi(ChSystem& other_physicalSystem, CHFSI_TIME_INTEGRATOR other_integrator)
    : mphysicalSystem(other_physicalSystem), mTime(0), fluidIntegrator(other_integrator), file_write_mode(CHFSI_OUTPUT_MODE::NONE) {
    fsiSystem = chrono_types::make_shared<ChSystemFsi_impl>();
    // fsiSystem = new ChSystemFsi_impl();
    paramsH = chrono_types::make_shared<SimParams>();
    numObjectsH = fsiSystem->numObjects;
    bceWorker = chrono_types::make_shared<ChBce>(fsiSystem->sortedSphMarkersD, fsiSystem->markersProximityD,
                                                 fsiSystem->fsiGeneralData, paramsH, numObjectsH);
    fluidDynamics = chrono_types::make_shared<ChFluidDynamics>(bceWorker, fsiSystem, paramsH, numObjectsH, other_integrator);

    fsi_mesh = chrono_types::make_shared<fea::ChMesh>();
    fsiBodies.resize(0);
    fsiShells.resize(0);
    fsiCables.resize(0);
    fsiNodes.resize(0);
    fsiInterface = chrono_types::make_shared<ChFsiInterface>(
        mphysicalSystem, fsi_mesh, paramsH, fsiSystem->fsiBodiesH, fsiSystem->fsiMeshH, fsiBodies, fsiNodes, fsiCables,
        fsiShells, fsiSystem->fsiGeneralData->CableElementsNodesH, fsiSystem->fsiGeneralData->CableElementsNodes,
        fsiSystem->fsiGeneralData->ShellElementsNodesH, fsiSystem->fsiGeneralData->ShellElementsNodes,
        fsiSystem->fsiGeneralData->rigid_FSI_ForcesD, fsiSystem->fsiGeneralData->rigid_FSI_TorquesD,
        fsiSystem->fsiGeneralData->Flex_FSI_ForcesD);
    // m_sys = new ChSystemFsi_impl(other_physicalSystem, other_integrator);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFluidIntegratorType(fluid_dynamics params_type) {
    if (params_type == fluid_dynamics::IISPH) {
        fluidIntegrator = CHFSI_TIME_INTEGRATOR::IISPH;
        std::cout << "fluid dynamics is reset to IISPH" << std::endl;
    } else if (params_type == fluid_dynamics::WCSPH) {
        fluidIntegrator = CHFSI_TIME_INTEGRATOR::ExplicitSPH;
        std::cout << "fluid dynamics is reset to Explicit WCSPH" << std::endl;
    } else {
        fluidIntegrator = CHFSI_TIME_INTEGRATOR::I2SPH;
        std::cout << "fluid dynamics is reset to I2SPH" << std::endl;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFluidDynamics(fluid_dynamics params_type) {
    SetFluidIntegratorType(params_type);
    fluidDynamics =
        chrono_types::make_shared<ChFluidDynamics>(bceWorker, fsiSystem, paramsH, numObjectsH, fluidIntegrator);
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Finalize() {
    printf("\n\nChSystemFsi::Finalize 1-FinalizeData\n");
    FinalizeData();
    printf("\n\nChSystemFsi::Finalize 2-bceWorker->Finalize\n");
    bceWorker->Finalize(fsiSystem->sphMarkersD1, fsiSystem->fsiBodiesD1, fsiSystem->fsiMeshD);
    printf("\n\nChSystemFsi::Finalize 3-fluidDynamics->Finalize\n");
    fluidDynamics->Finalize();
    std::cout << "referenceArraySize in 3-fluidDynamics->Finalize"
              << fsiSystem->fsiGeneralData->referenceArray.size() << "\n";
}
//--------------------------------------------------------------------------------------------------------------------------------

ChSystemFsi::~ChSystemFsi() {}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::CopyDeviceDataToHalfStep() {
    thrust::copy(fsiSystem->sphMarkersD2->posRadD.begin(), fsiSystem->sphMarkersD2->posRadD.end(),
                 fsiSystem->sphMarkersD1->posRadD.begin());
    thrust::copy(fsiSystem->sphMarkersD2->velMasD.begin(), fsiSystem->sphMarkersD2->velMasD.end(),
                 fsiSystem->sphMarkersD1->velMasD.begin());
    thrust::copy(fsiSystem->sphMarkersD2->rhoPresMuD.begin(), fsiSystem->sphMarkersD2->rhoPresMuD.end(),
                 fsiSystem->sphMarkersD1->rhoPresMuD.begin());
    thrust::copy(fsiSystem->sphMarkersD2->tauXxYyZzD.begin(), fsiSystem->sphMarkersD2->tauXxYyZzD.end(),
                 fsiSystem->sphMarkersD1->tauXxYyZzD.begin());
    thrust::copy(fsiSystem->sphMarkersD2->tauXyXzYzD.begin(), fsiSystem->sphMarkersD2->tauXyXzYzD.end(),
                 fsiSystem->sphMarkersD1->tauXyXzYzD.begin());
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::DoStepDynamics_FSI() {
    /// The following is used to execute the Explicit WCSPH
    if (fluidDynamics->GetIntegratorType() == CHFSI_TIME_INTEGRATOR::ExplicitSPH) {
        fsiInterface->Copy_ChSystem_to_External();
        CopyDeviceDataToHalfStep();
        ChUtilsDevice::FillMyThrust3(fsiSystem->fsiGeneralData->derivTauXxYyZzD, mR3(0));
        ChUtilsDevice::FillMyThrust3(fsiSystem->fsiGeneralData->derivTauXyXzYzD, mR3(0));
        ChUtilsDevice::FillMyThrust4(fsiSystem->fsiGeneralData->derivVelRhoD, mR4(0));
        fluidDynamics->IntegrateSPH(fsiSystem->sphMarkersD2, fsiSystem->sphMarkersD1, fsiSystem->fsiBodiesD2,
                                    fsiSystem->fsiMeshD, 0.5 * paramsH->dT);
        fluidDynamics->IntegrateSPH(fsiSystem->sphMarkersD1, fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2,
                                    fsiSystem->fsiMeshD, 1.0 * paramsH->dT);
        bceWorker->Rigid_Forces_Torques(fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(fsiSystem->sphMarkersD2, fsiSystem->fsiMeshD);
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

        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiSystem->fsiBodiesD2);
        bceWorker->UpdateRigidMarkersPositionVelocity(fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2);
        // fsiSystem->sphMarkersD1 = fsiSystem->sphMarkersD2;
        // Density re-initialization
        //        int tStep = int(mTime / paramsH->dT);
        //        if ((tStep % (paramsH->densityReinit + 1) == 0)) {
        //            fluidDynamics->DensityReinitialization();
        //        }
    } else {
        // A different coupling scheme is used for implicit SPH formulations
        printf("Copy_ChSystem_to_External\n");
        fsiInterface->Copy_ChSystem_to_External();
        printf("IntegrateIISPH\n");
        fluidDynamics->IntegrateSPH(fsiSystem->sphMarkersD2, fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2,
                                    fsiSystem->fsiMeshD, 0.0);
        printf("Fluid-structure forces\n");
        bceWorker->Rigid_Forces_Torques(fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(fsiSystem->sphMarkersD2, fsiSystem->fsiMeshD);
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
        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiSystem->fsiBodiesD2);
        bceWorker->UpdateRigidMarkersPositionVelocity(fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2);

        printf("Update Flexible Marker\n");
        fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiSystem->fsiMeshD);
        bceWorker->UpdateFlexMarkersPositionVelocity(fsiSystem->sphMarkersD2, fsiSystem->fsiMeshD);

        printf("Update Flexible Marker\n");
        fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiSystem->fsiMeshD);
        bceWorker->UpdateFlexMarkersPositionVelocity(fsiSystem->sphMarkersD2, fsiSystem->fsiMeshD);
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
    fsiInterface->ResizeChronoCablesData(CableElementsNodes, fsiSystem->fsiGeneralData->CableElementsNodesH);
    fsiInterface->ResizeChronoShellsData(ShellElementsNodes, fsiSystem->fsiGeneralData->ShellElementsNodesH);
    fsiInterface->ResizeChronoFEANodesData();
    printf("\nfsiSystem->ResizeDataManager...\n");
    fea_node = fsi_mesh->GetNnodes();

    fsiSystem->ResizeDataManager(fea_node);

    printf("\n\nfsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem()\n");
    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiSystem->fsiBodiesD1);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiSystem->fsiMeshD);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiSystem->fsiMeshD);

    std::cout << "referenceArraySize in FinalizeData " << fsiSystem->fsiGeneralData->referenceArray.size()
              << "\n";
    fsiSystem->fsiBodiesD2 = fsiSystem->fsiBodiesD1;  //(2) construct midpoint rigid data
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::WriteParticleFile(const std::string& outfilename) const {
    if (file_write_mode == CHFSI_OUTPUT_MODE::CSV) {
        utils::WriteCsvParticlesToFile(fsiSystem->sphMarkersD2->posRadD, fsiSystem->sphMarkersD2->velMasD,
                                       fsiSystem->sphMarkersD2->rhoPresMuD, fsiSystem->fsiGeneralData->referenceArray,
                                       outfilename);
    } else if (file_write_mode == CHFSI_OUTPUT_MODE::CHPF) {
        utils::WriteChPFParticlesToFile(fsiSystem->sphMarkersD2->posRadD, fsiSystem->fsiGeneralData->referenceArray,
                                        outfilename);
    }
}

void ChSystemFsi::AddSphMarker(
    const ChVector<>& points,
    const ChVector<>& properties,
    const double h,
    const double particle_type,
    const ChVector<>& velocity,
    const ChVector<>& tauXxYyZz,
    const ChVector<>& tauXyXzYz) {

    fsiSystem->AddSphMarker(ChUtilsTypeConvert::ChVectorRToReal4(points, h),
                            ChUtilsTypeConvert::ChVectorRToReal4(properties, particle_type),
                            ChUtilsTypeConvert::ChVectorToReal3(velocity),
                            ChUtilsTypeConvert::ChVectorToReal3(tauXxYyZz),
                            ChUtilsTypeConvert::ChVectorToReal3(tauXyXzYz));
}

//--------------------------------------------------------------------------------------------------------------------------------

}  // end namespace fsi
}  // end namespace chrono
