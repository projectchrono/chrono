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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration
// =============================================================================

#include "chrono/core/ChTypes.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------
ChSystemFsi::ChSystemFsi(ChSystem& other_physicalSystem, CHFSI_TIME_INTEGRATOR other_integrator)
    : mphysicalSystem(other_physicalSystem), mTime(0), fluidIntegrator(other_integrator),
      file_write_mode(CHFSI_OUTPUT_MODE::NONE) {
    fsiSystem = chrono_types::make_shared<ChSystemFsi_impl>();
    paramsH = chrono_types::make_shared<SimParams>();
    numObjectsH = fsiSystem->numObjects;
    bceWorker = chrono_types::make_shared<ChBce>(
        fsiSystem->sortedSphMarkersD, fsiSystem->markersProximityD,
        fsiSystem->fsiGeneralData, paramsH, numObjectsH);
    fluidDynamics = chrono_types::make_shared<ChFluidDynamics>(
        bceWorker, fsiSystem, paramsH, numObjectsH, other_integrator);

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
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFluidIntegratorType(fluid_dynamics params_type) {
    if (params_type == fluid_dynamics::IISPH) {
        fluidIntegrator = CHFSI_TIME_INTEGRATOR::IISPH;
        std::cout << "=====================   Fluid dynamics is reset to IISPH   ====================" << std::endl;
        std::cout << "===============================================================================" << std::endl;
        std::cout << "===============================================================================" << std::endl;
    } else if (params_type == fluid_dynamics::WCSPH) {
        fluidIntegrator = CHFSI_TIME_INTEGRATOR::ExplicitSPH;
        std::cout << "===============================================================================" << std::endl;
        std::cout << "=====================   Fluid dynamics is reset to WCSPH   ====================" << std::endl;
        std::cout << "===============================================================================" << std::endl;
    } else {
        fluidIntegrator = CHFSI_TIME_INTEGRATOR::I2SPH;
        std::cout << "===============================================================================" << std::endl;
        std::cout << "=====================   Fluid dynamics is reset to I2SPH   ====================" << std::endl;
        std::cout << "===============================================================================" << std::endl;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFluidDynamics(fluid_dynamics params_type) {
    SetFluidIntegratorType(params_type);
    fluidDynamics =
        chrono_types::make_shared<ChFluidDynamics>(bceWorker, fsiSystem, paramsH, numObjectsH, fluidIntegrator);
    fluidDynamics->GetForceSystem()->SetLinearSolver(paramsH->LinearSolver);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::Finalize() {
    printf("===============================================================================\n");
    printf("ChSystemFsi::Finalize 1-FinalizeData\n");
    FinalizeData();
    
    printf("===============================================================================\n");
    printf("ChSystemFsi::Finalize 2-bceWorker->Finalize\n");
    bceWorker->Finalize(fsiSystem->sphMarkersD1, fsiSystem->fsiBodiesD1, fsiSystem->fsiMeshD);
    
    printf("===============================================================================\n");
    printf("ChSystemFsi::Finalize 3-fluidDynamics->Finalize\n");
    fluidDynamics->Finalize();
    std::cout << "referenceArraySize in fluid dynamics is " 
              << fsiSystem->fsiGeneralData->referenceArray.size()
              << std::endl;
    printf("===============================================================================\n");
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
    if (fluidDynamics->GetIntegratorType() == CHFSI_TIME_INTEGRATOR::ExplicitSPH) {
        // The following is used to execute the Explicit WCSPH
        CopyDeviceDataToHalfStep();
        ChUtilsDevice::FillMyThrust4(fsiSystem->fsiGeneralData->derivVelRhoD, mR4(0));
        fluidDynamics->IntegrateSPH(fsiSystem->sphMarkersD2, fsiSystem->sphMarkersD1, fsiSystem->fsiBodiesD2,
                                    fsiSystem->fsiMeshD, 0.5 * paramsH->dT, mTime);
        fluidDynamics->IntegrateSPH(fsiSystem->sphMarkersD1, fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2,
                                    fsiSystem->fsiMeshD, 1.0 * paramsH->dT, mTime);
        bceWorker->Rigid_Forces_Torques(fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(fsiSystem->sphMarkersD2, fsiSystem->fsiMeshD);
        // Note that because of applying forces to the nodal coordinates using SetForce() no other external forces can
        // be applied, or if any thing has been applied will be rewritten by Add_Flex_Forces_To_ChSystem();
        fsiInterface->Add_Flex_Forces_To_ChSystem();

        // dT_Flex is the time step of solid body system
        mTime += 1 * paramsH->dT;
        if (paramsH->dT_Flex == 0)
            paramsH->dT_Flex = paramsH->dT;
        int sync = int(paramsH->dT / paramsH->dT_Flex);
        if (sync < 1)
            sync = 1;
        for (int t = 0; t < sync; t++) {
            mphysicalSystem.DoStepDynamics(paramsH->dT / sync);
        }

        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiSystem->fsiBodiesD2);
        bceWorker->UpdateRigidMarkersPositionVelocity(fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2);
    } else {
        // A different coupling scheme is used for implicit SPH formulations
        printf("Copy_ChSystem_to_External\n");
        fsiInterface->Copy_ChSystem_to_External();
        printf("IntegrateIISPH\n");
        fluidDynamics->IntegrateSPH(fsiSystem->sphMarkersD2, fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2,
                                    fsiSystem->fsiMeshD, 0.0, mTime);
        printf("Fluid-structure forces\n");
        bceWorker->Rigid_Forces_Torques(fsiSystem->sphMarkersD2, fsiSystem->fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(fsiSystem->sphMarkersD2, fsiSystem->fsiMeshD);
        // Note that because of applying forces to the nodal coordinates using SetForce() no other external forces can
        // be applied, or if any thing has been applied will be rewritten by Add_Flex_Forces_To_ChSystem();
        fsiInterface->Add_Flex_Forces_To_ChSystem();

        mTime += 1 * paramsH->dT;
        if (paramsH->dT_Flex == 0)
            paramsH->dT_Flex = paramsH->dT;
        int sync = int(paramsH->dT / paramsH->dT_Flex);
        if (sync < 1)
            sync = 1;
        printf("%d * Chrono StepDynamics with dt= %f\n", sync, paramsH->dT / sync);
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
    printf("fsiInterface->ResizeChronoBodiesData()\n");
    fsiInterface->ResizeChronoBodiesData();
    int fea_node = 0;
    fsiInterface->ResizeChronoCablesData(CableElementsNodes, fsiSystem->fsiGeneralData->CableElementsNodesH);
    fsiInterface->ResizeChronoShellsData(ShellElementsNodes, fsiSystem->fsiGeneralData->ShellElementsNodesH);
    fsiInterface->ResizeChronoFEANodesData();
    printf("fsiSystem->ResizeDataManager()\n");
    fea_node = fsi_mesh->GetNnodes();

    fsiSystem->ResizeDataManager(fea_node);

    printf("fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem()\n");
    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(fsiSystem->fsiBodiesD1);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiSystem->fsiMeshD);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(fsiSystem->fsiMeshD);

    std::cout << "referenceArraySize in FinalizeData is " 
              << fsiSystem->fsiGeneralData->referenceArray.size() 
              << std::endl;
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
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::PrintParticleToFile(const std::string& out_dir) const {
    utils::PrintToFile(fsiSystem->sphMarkersD2->posRadD, fsiSystem->sphMarkersD2->velMasD,
                       fsiSystem->sphMarkersD2->rhoPresMuD, fsiSystem->fsiGeneralData->sr_tau_I_mu_i,
                       fsiSystem->fsiGeneralData->referenceArray, thrust::host_vector<int4>(), out_dir, 
                       paramsH, true);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddSphMarker(const ChVector<>& point,
                               double rho0,
                               double pres0,
                               double mu0,
                               double h,
                               double particle_type,
                               const ChVector<>& velocity,
                               const ChVector<>& tauXxYyZz,
                               const ChVector<>& tauXyXzYz) {
    fsiSystem->AddSphMarker(ChUtilsTypeConvert::ChVectorToReal4(point, h), mR4(rho0, pres0, mu0, particle_type),
                            ChUtilsTypeConvert::ChVectorToReal3(velocity),
                            ChUtilsTypeConvert::ChVectorToReal3(tauXxYyZz),
        ChUtilsTypeConvert::ChVectorToReal3(tauXyXzYz));
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddRefArray(const int start, const int numPart, const int compType, const int phaseType) {
    fsiSystem->fsiGeneralData->referenceArray.push_back(mI4(start, numPart, compType, phaseType));
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddBceBox(std::shared_ptr<SimParams> paramsH,
                            std::shared_ptr<ChBody> body,
                            const ChVector<>& relPos,
                            const ChQuaternion<>& relRot,
                            const ChVector<>& size,
                            int plane,
                            bool isSolid) {
    utils::AddBoxBce(fsiSystem, paramsH, body, relPos, relRot, size, plane, isSolid);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddBceSphere(std::shared_ptr<SimParams> paramsH,
                               std::shared_ptr<ChBody> body,
                               const ChVector<>& relPos,
                               const ChQuaternion<>& relRot,
                               double radius) {
    utils::AddSphereBce(fsiSystem, paramsH, body, relPos, relRot, radius);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddBceCylinder(std::shared_ptr<SimParams> paramsH,
                                 std::shared_ptr<ChBody> body,
                                 const ChVector<>& relPos,
                                 const ChQuaternion<>& relRot,
                                 double radius,
                                 double height,
                                 double kernel_h,
                                 bool cartesian) {
    utils::AddCylinderBce(fsiSystem, paramsH, body, relPos, relRot, radius, height, kernel_h, cartesian);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddBceFromPoints(std::shared_ptr<SimParams> paramsH,
                                   std::shared_ptr<ChBody> body,
                                   const std::vector<chrono::ChVector<>>& points,
                                   const ChVector<>& collisionShapeRelativePos,
                                   const ChQuaternion<>& collisionShapeRelativeRot) {
    utils::AddBCE_FromPoints(fsiSystem, paramsH, body, points, collisionShapeRelativePos, collisionShapeRelativeRot);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddBceFile(std::shared_ptr<SimParams> paramsH,
                             std::shared_ptr<ChBody> body,
                             const std::string& dataPath,
                             const ChVector<>& collisionShapeRelativePos,
                             const ChQuaternion<>& collisionShapeRelativeRot,
                             double scale,
                             bool isSolid) {  // true means moving body, false means fixed boundary
    utils::AddBCE_FromFile(fsiSystem, paramsH, body, dataPath, collisionShapeRelativePos, collisionShapeRelativeRot,
                           scale, isSolid);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::AddBceFromMesh(std::shared_ptr<SimParams> paramsH,
                                 std::shared_ptr<fea::ChMesh> my_mesh,
                                 const std::vector<std::vector<int>>& NodeNeighborElement,
                                 const std::vector<std::vector<int>>& _1D_elementsNodes,
                                 const std::vector<std::vector<int>>& _2D_elementsNodes,
                                 bool add1DElem,
                                 bool add2DElem,
                                 bool multiLayer,
                                 bool removeMiddleLayer,
                                 int SIDE,
                                 int SIDE2D) {
    utils::AddBCE_FromMesh(fsiSystem, paramsH, my_mesh, this->GetFsiNodes(), this->GetFsiCables(), this->GetFsiShells(),
                           NodeNeighborElement, _1D_elementsNodes, _2D_elementsNodes, add1DElem, add2DElem, multiLayer,
                           removeMiddleLayer, SIDE, SIDE2D);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetSimParameter(const std::string& inputJson,
                                  std::shared_ptr<SimParams> paramsH,
                                  const ChVector<>& box_size) {
    utils::ParseJSON(inputJson, paramsH, ChUtilsTypeConvert::ChVectorToReal3(box_size));
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetBoundaries(const ChVector<>& cMin, const ChVector<>& cMax, std::shared_ptr<SimParams> paramsH) {
    paramsH->cMin = ChUtilsTypeConvert::ChVectorToReal3(cMin);
    paramsH->cMax = ChUtilsTypeConvert::ChVectorToReal3(cMax);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetInitPressure(std::shared_ptr<SimParams> paramsH, const double fzDim) {
    size_t numParticles = fsiSystem->sphMarkersH->rhoPresMuH.size();
    for (int i = 0; i < numParticles; i++) {
        double z = fsiSystem->sphMarkersH->posRadH[i].z;
        fsiSystem->sphMarkersH->rhoPresMuH[i].y =
            -paramsH->rho0 * paramsH->gravity.z * paramsH->gravity.z * (z - fzDim);
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
float ChSystemFsi::GetKernelLength() const {
    return paramsH->HSML;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetSubDomain(std::shared_ptr<SimParams> paramsH) {
    utils::FinalizeDomain(paramsH);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFsiOutputDir(std::shared_ptr<SimParams> paramsH,
                                  std::string& demo_dir,
                                  std::string out_dir,
                                  std::string inputJson) {
    utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetDiscreType(bool useGmatrix, bool useLmatrix){
    paramsH->USE_Consistent_G = useGmatrix;
    paramsH->USE_Consistent_L = useLmatrix;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetFsiInfoOutput(bool outputFsiInfo){
    paramsH->output_fsi =  outputFsiInfo;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetOutputLength(int OutputLength){
    paramsH->output_length = OutputLength;
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChSystemFsi::SetWallBC(BceVersion wallBC){
    paramsH->bceTypeWall = wallBC;
}
//--------------------------------------------------------------------------------------------------------------------------------
std::vector<ChVector<>> ChSystemFsi::GetParticlePosOrProperties() {
    thrust::host_vector<Real4> posRadH = fsiSystem->sphMarkersD2->posRadD;
    std::vector<ChVector<>> pos;
    for (size_t i = 0; i < posRadH.size(); i++) {
        pos.push_back(ChUtilsTypeConvert::Real4ToChVector(posRadH[i]));
    }
    return pos;
}
//--------------------------------------------------------------------------------------------------------------------------------
std::vector<ChVector<>> ChSystemFsi::GetParticleVel() {
    thrust::host_vector<Real3> velH = fsiSystem->sphMarkersD2->velMasD;
    std::vector<ChVector<>> vel;
    for (size_t i = 0; i < velH.size(); i++) {
        vel.push_back(ChUtilsTypeConvert::Real3ToChVector(velH[i]));
    }
    return vel;
}
//--------------------------------------------------------------------------------------------------------------------------------

}  // end namespace fsi
}  // end namespace chrono
