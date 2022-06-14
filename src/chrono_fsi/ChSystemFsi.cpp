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

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/utils/ChUtilsGeneratorBce.h"

namespace chrono {
namespace fsi {

ChSystemFsi::ChSystemFsi(ChSystem& other_physicalSystem)
    : sysMBS(other_physicalSystem),
      mTime(0),
      file_write_mode(CHFSI_OUTPUT_MODE::NONE) {
    paramsH = chrono_types::make_shared<SimParams>();
    InitParams();
    numObjectsH = sysFSI.numObjects;

    fsi_mesh = chrono_types::make_shared<fea::ChMesh>();
    fsiBodies.resize(0);
    fsiShells.resize(0);
    fsiCables.resize(0);
    fsiNodes.resize(0);
    fsiInterface = chrono_types::make_shared<ChFsiInterface>(sysMBS, sysFSI,     //
                                                             paramsH, fsi_mesh,  //
                                                             fsiBodies, fsiNodes, fsiCables, fsiShells);
}

ChSystemFsi::~ChSystemFsi() {}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::InitParams() {
    //// RADU TODO
    //// Provide default values for *all* parameters!

    strcpy(paramsH->out_name, "Undefined");
    paramsH->output_length = 1;
    paramsH->output_fsi = true;

    // Fluid properties
    paramsH->rho0 = Real(1.0);
    paramsH->invrho0 = 1 / paramsH->rho0;
    paramsH->rho_solid = paramsH->rho0;
    paramsH->mu0 = Real(0.001);
    paramsH->bodyForce3 = mR3(0, 0, 0);
    paramsH->gravity = mR3(0, 0, 0);
    paramsH->kappa = Real(0.0);
    paramsH->L_Characteristic = Real(1.0);

    // SPH parameters
    paramsH->fluid_dynamic_type = fluid_dynamics::I2SPH;
    paramsH->HSML = Real(0.02);
    paramsH->INVHSML = 1 / paramsH->HSML;
    paramsH->INITSPACE = paramsH->HSML;
    paramsH->volume0 = cube(paramsH->INITSPACE);
    paramsH->INV_INIT = 1 / paramsH->INITSPACE;
    paramsH->MULT_INITSPACE_Shells = Real(1.0);
    paramsH->v_Max = Real(1.0);
    paramsH->EPS_XSPH = Real(0.11);
    paramsH->beta_shifting = Real(0.0);
    paramsH->densityReinit = 2147483647;
    paramsH->Conservative_Form = true;
    paramsH->gradient_type = 0;
    paramsH->laplacian_type = 0;
    paramsH->USE_Consistent_L = true;
    paramsH->USE_Consistent_G = true;

    paramsH->markerMass = paramsH->volume0 * paramsH->rho0;

    paramsH->NUM_BOUNDARY_LAYERS = 3;

    // Time stepping
    paramsH->Adaptive_time_stepping = false;
    paramsH->Co_number = Real(0.1);
    paramsH->Beta = Real(0.5);
    paramsH->dT = Real(0.01);
    paramsH->INV_dT = 1 / paramsH->dT;
    paramsH->dT_Flex = paramsH->dT;
    paramsH->dT_Max = Real(1.0);
    paramsH->out_fps = Real(20);
    paramsH->tFinal = Real(2000);

    // Pressure equation
    paramsH->PPE_Solution_type = PPE_SolutionType::MATRIX_FREE;
    paramsH->Alpha = paramsH->HSML;
    paramsH->PPE_relaxation = Real(1.0);
    paramsH->LinearSolver_Abs_Tol = Real(0.0);
    paramsH->LinearSolver_Rel_Tol = Real(0.0);
    paramsH->LinearSolver_Max_Iter = 1000;
    paramsH->Verbose_monitoring = false;
    paramsH->Pressure_Constraint = false;
    paramsH->BASEPRES = Real(0.0);
    paramsH->ClampPressure = false;

    paramsH->bceType = BceVersion::ADAMI;
    paramsH->bceTypeWall = BceVersion::ADAMI;

    // Elastic SPH
    paramsH->C_Wi = Real(0.8);

    //
    paramsH->bodyActiveDomain = mR3(1e10, 1e10, 1e10);
    paramsH->settlingTime = Real(1e10);

    //
    paramsH->Max_Pressure = Real(1e20);
    paramsH->Domain = mR3(1, 1, 1);

    //// RADU TODO
    //// material model

    paramsH->use_default_limits = true;
    paramsH->use_init_pressure = false;
    paramsH->use_subdomains = false;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::SetFluidDynamics(fluid_dynamics SPH_method, ChFsiLinearSolver::SolverType lin_solver) {
    paramsH->fluid_dynamic_type = SPH_method;
    paramsH->LinearSolver = lin_solver;
}

void ChSystemFsi::SetSimParameter(const std::string& inputJson, const ChVector<>& box_size) {
    utils::ParseJSON(inputJson, paramsH, ChUtilsTypeConvert::ChVectorToReal3(box_size));
}

void ChSystemFsi::SetSimDim(const ChVector<>& fluidDim) {
    paramsH->fluidDimX = fluidDim.x();
    paramsH->fluidDimY = fluidDim.y();
    paramsH->fluidDimZ = fluidDim.z();
}

void ChSystemFsi::SetContainerDim(const ChVector<>& boxDim) {
    paramsH->boxDimX = boxDim.x();
    paramsH->boxDimY = boxDim.y();
    paramsH->boxDimZ = boxDim.z();
}

void ChSystemFsi::SetBoundaries(const ChVector<>& cMin, const ChVector<>& cMax) {
    paramsH->cMin = ChUtilsTypeConvert::ChVectorToReal3(cMin);
    paramsH->cMax = ChUtilsTypeConvert::ChVectorToReal3(cMax);
    paramsH->use_default_limits = false;
}

void ChSystemFsi::SetNumBoundaryLayers(int num_layers) {
    paramsH->NUM_BOUNDARY_LAYERS = num_layers;
}

void ChSystemFsi::SetInitPressure(const double height) {
    paramsH->pressure_height = height;
    paramsH->use_init_pressure = true;
}

void ChSystemFsi::Set_G_acc(const ChVector<>& gravity) {
    paramsH->gravity.x = gravity.x();
    paramsH->gravity.y = gravity.y();
    paramsH->gravity.z = gravity.z();
}

void ChSystemFsi::SetInitialSpacing(double spacing) {
    paramsH->INITSPACE = (Real)spacing;
    paramsH->INV_INIT = 1 / paramsH->INITSPACE;
    paramsH->volume0 = cube(paramsH->INITSPACE);
    paramsH->MULT_INITSPACE = paramsH->INITSPACE / paramsH->HSML;
    paramsH->markerMass = paramsH->volume0 * paramsH->rho0;
}

void ChSystemFsi::SetKernelLength(double length) {
    paramsH->HSML = (Real)length;
    paramsH->MULT_INITSPACE = paramsH->INITSPACE / paramsH->HSML;
    paramsH->INVHSML = 1 / paramsH->HSML;
}

void ChSystemFsi::SetStepSize(double dT, double dT_Flex) {
    paramsH->dT = dT;
    paramsH->INV_dT = 1 / paramsH->dT;
    paramsH->dT_Flex = (dT_Flex == 0) ? paramsH->dT : dT_Flex;
}

void ChSystemFsi::SetDensity(double rho0) {
    paramsH->rho0 = rho0;
    paramsH->invrho0 = 1 / paramsH->rho0;
    paramsH->markerMass = paramsH->volume0 * paramsH->rho0;
}

void ChSystemFsi::SetSubdomains(bool val) {
    paramsH->use_subdomains = val;
}

void ChSystemFsi::SetFsiOutputDir(std::string& demo_dir, std::string out_dir, std::string inputJson) {
    utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson);
}

void ChSystemFsi::SetDiscreType(bool useGmatrix, bool useLmatrix) {
    paramsH->USE_Consistent_G = useGmatrix;
    paramsH->USE_Consistent_L = useLmatrix;
}

void ChSystemFsi::SetFsiInfoOutput(bool outputFsiInfo) {
    paramsH->output_fsi = outputFsiInfo;
}

void ChSystemFsi::SetOutputLength(int OutputLength) {
    paramsH->output_length = OutputLength;
}

void ChSystemFsi::SetWallBC(BceVersion wallBC) {
    paramsH->bceTypeWall = wallBC;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::Finalize() {
    printf("===============================================================================\n");
    printf("ChSystemFsi::Finalize 1-FinalizeData\n");

    // Calculate dependent quantities in the params structure

    paramsH->Cs = 10 * paramsH->v_Max;

    int NN = 0;
    ////paramsH->markerMass = massCalculator(NN, paramsH->HSML, paramsH->MULT_INITSPACE * paramsH->HSML, paramsH->rho0);
    paramsH->num_neighbors = NN;

    if (paramsH->use_default_limits) {
        paramsH->cMin =
            mR3(-2 * paramsH->Domain.x, -2 * paramsH->Domain.y, -2 * paramsH->Domain.z) - 10 * mR3(paramsH->HSML);
        paramsH->cMax =
            mR3(+2 * paramsH->Domain.x, +2 * paramsH->Domain.y, +2 * paramsH->Domain.z) + 10 * mR3(paramsH->HSML);
    }

    if (paramsH->use_init_pressure) {
        size_t numParticles = sysFSI.sphMarkersH->rhoPresMuH.size();
        for (int i = 0; i < numParticles; i++) {
            double z = sysFSI.sphMarkersH->posRadH[i].z;
            sysFSI.sphMarkersH->rhoPresMuH[i].y =
                -paramsH->rho0 * paramsH->gravity.z * paramsH->gravity.z * (z - paramsH->pressure_height);
        }
    }

    if (paramsH->use_subdomains) {
        paramsH->NUM_BOUNDARY_LAYERS = 3;
        paramsH->Apply_BC_U = false;  // You should go to custom_math.h all the way to end of file and set your function
        int3 side0 = mI3((int)floor((paramsH->cMax.x - paramsH->cMin.x) / (RESOLUTION_LENGTH_MULT * paramsH->HSML)),
                         (int)floor((paramsH->cMax.y - paramsH->cMin.y) / (RESOLUTION_LENGTH_MULT * paramsH->HSML)),
                         (int)floor((paramsH->cMax.z - paramsH->cMin.z) / (RESOLUTION_LENGTH_MULT * paramsH->HSML)));
        Real3 binSize3 =
            mR3((paramsH->cMax.x - paramsH->cMin.x) / side0.x, (paramsH->cMax.y - paramsH->cMin.y) / side0.y,
                (paramsH->cMax.z - paramsH->cMin.z) / side0.z);
        paramsH->binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
        paramsH->binSize0 = binSize3.x;
        paramsH->boxDims = paramsH->cMax - paramsH->cMin;
        paramsH->straightChannelBoundaryMin = paramsH->cMin;  // mR3(0, 0, 0);  // 3D channel
        paramsH->straightChannelBoundaryMax = paramsH->cMax;  // SmR3(3, 2, 3) * paramsH->sizeScale;
        paramsH->deltaPress = mR3(0);
        int3 SIDE = mI3(int((paramsH->cMax.x - paramsH->cMin.x) / paramsH->binSize0 + .1),
                        int((paramsH->cMax.y - paramsH->cMin.y) / paramsH->binSize0 + .1),
                        int((paramsH->cMax.z - paramsH->cMin.z) / paramsH->binSize0 + .1));
        Real mBinSize = paramsH->binSize0;
        paramsH->gridSize = SIDE;
        paramsH->worldOrigin = paramsH->cMin;
        paramsH->cellSize = mR3(mBinSize, mBinSize, mBinSize);    
    }

    printf("fsiInterface->ResizeChronoBodiesData()\n");
    fsiInterface->ResizeChronoBodiesData();
    int fea_node = 0;
    fsiInterface->ResizeChronoCablesData(CableElementsNodes);
    fsiInterface->ResizeChronoShellsData(ShellElementsNodes);
    fsiInterface->ResizeChronoFEANodesData();
    printf("sysFSI.ResizeDataManager()\n");
    fea_node = fsi_mesh->GetNnodes();

    sysFSI.ResizeDataManager(fea_node);

    printf("fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem()\n");
    fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(sysFSI.fsiBodiesD1);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(sysFSI.fsiMeshD);
    fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(sysFSI.fsiMeshD);

    std::cout << "referenceArraySize in FinalizeData is " << sysFSI.fsiGeneralData->referenceArray.size() << std::endl;
    sysFSI.fsiBodiesD2 = sysFSI.fsiBodiesD1;  //(2) construct midpoint rigid data

    // Create BCE and SPH worker objects

    bceWorker = chrono_types::make_shared<ChBce>(sysFSI.sortedSphMarkersD, sysFSI.markersProximityD,
                                                 sysFSI.fsiGeneralData, paramsH, numObjectsH);

    switch (paramsH->fluid_dynamic_type) {
        case fluid_dynamics::IISPH:
            fluidIntegrator = CHFSI_TIME_INTEGRATOR::IISPH;
            break;
        case fluid_dynamics::WCSPH:
            fluidIntegrator = CHFSI_TIME_INTEGRATOR::ExplicitSPH;
            break;
        default:
            fluidIntegrator = CHFSI_TIME_INTEGRATOR::I2SPH;
            break;
    }
    fluidDynamics =
        chrono_types::make_shared<ChFluidDynamics>(bceWorker, sysFSI, paramsH, numObjectsH, fluidIntegrator);
    fluidDynamics->GetForceSystem()->SetLinearSolver(paramsH->LinearSolver);

    // Initialize worker objects

    printf("===============================================================================\n");
    printf("ChSystemFsi::Finalize 2-bceWorker->Finalize\n");
    bceWorker->Finalize(sysFSI.sphMarkersD1, sysFSI.fsiBodiesD1, sysFSI.fsiMeshD);

    printf("===============================================================================\n");
    printf("ChSystemFsi::Finalize 3-fluidDynamics->Finalize\n");
    fluidDynamics->Finalize();
    std::cout << "referenceArraySize in fluid dynamics is " << sysFSI.fsiGeneralData->referenceArray.size()
              << std::endl;
    printf("===============================================================================\n");
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::CopyDeviceDataToHalfStep() {
    thrust::copy(sysFSI.sphMarkersD2->posRadD.begin(), sysFSI.sphMarkersD2->posRadD.end(),
                 sysFSI.sphMarkersD1->posRadD.begin());
    thrust::copy(sysFSI.sphMarkersD2->velMasD.begin(), sysFSI.sphMarkersD2->velMasD.end(),
                 sysFSI.sphMarkersD1->velMasD.begin());
    thrust::copy(sysFSI.sphMarkersD2->rhoPresMuD.begin(), sysFSI.sphMarkersD2->rhoPresMuD.end(),
                 sysFSI.sphMarkersD1->rhoPresMuD.begin());
    thrust::copy(sysFSI.sphMarkersD2->tauXxYyZzD.begin(), sysFSI.sphMarkersD2->tauXxYyZzD.end(),
                 sysFSI.sphMarkersD1->tauXxYyZzD.begin());
    thrust::copy(sysFSI.sphMarkersD2->tauXyXzYzD.begin(), sysFSI.sphMarkersD2->tauXyXzYzD.end(),
                 sysFSI.sphMarkersD1->tauXyXzYzD.begin());
}

void ChSystemFsi::DoStepDynamics_FSI() {
    if (fluidDynamics->GetIntegratorType() == CHFSI_TIME_INTEGRATOR::ExplicitSPH) {
        // The following is used to execute the Explicit WCSPH
        CopyDeviceDataToHalfStep();
        ChUtilsDevice::FillMyThrust4(sysFSI.fsiGeneralData->derivVelRhoD, mR4(0));
        fluidDynamics->IntegrateSPH(sysFSI.sphMarkersD2, sysFSI.sphMarkersD1, sysFSI.fsiBodiesD2, sysFSI.fsiMeshD,
                                    0.5 * paramsH->dT, mTime);
        fluidDynamics->IntegrateSPH(sysFSI.sphMarkersD1, sysFSI.sphMarkersD2, sysFSI.fsiBodiesD2, sysFSI.fsiMeshD,
                                    1.0 * paramsH->dT, mTime);
        bceWorker->Rigid_Forces_Torques(sysFSI.sphMarkersD2, sysFSI.fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(sysFSI.sphMarkersD2, sysFSI.fsiMeshD);
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
            sysMBS.DoStepDynamics(paramsH->dT / sync);
        }

        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(sysFSI.fsiBodiesD2);
        bceWorker->UpdateRigidMarkersPositionVelocity(sysFSI.sphMarkersD2, sysFSI.fsiBodiesD2);
    } else {
        // A different coupling scheme is used for implicit SPH formulations
        printf("Copy_ChSystem_to_External\n");
        fsiInterface->Copy_ChSystem_to_External();
        printf("IntegrateIISPH\n");
        fluidDynamics->IntegrateSPH(sysFSI.sphMarkersD2, sysFSI.sphMarkersD2, sysFSI.fsiBodiesD2, sysFSI.fsiMeshD, 0.0,
                                    mTime);
        printf("Fluid-structure forces\n");
        bceWorker->Rigid_Forces_Torques(sysFSI.sphMarkersD2, sysFSI.fsiBodiesD2);
        fsiInterface->Add_Rigid_ForceTorques_To_ChSystem();

        bceWorker->Flex_Forces(sysFSI.sphMarkersD2, sysFSI.fsiMeshD);
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
            sysMBS.DoStepDynamics(paramsH->dT / sync);
        }

        printf("Update Rigid Marker\n");
        fsiInterface->Copy_fsiBodies_ChSystem_to_FluidSystem(sysFSI.fsiBodiesD2);
        bceWorker->UpdateRigidMarkersPositionVelocity(sysFSI.sphMarkersD2, sysFSI.fsiBodiesD2);

        printf("Update Flexible Marker\n");
        fsiInterface->Copy_fsiNodes_ChSystem_to_FluidSystem(sysFSI.fsiMeshD);
        bceWorker->UpdateFlexMarkersPositionVelocity(sysFSI.sphMarkersD2, sysFSI.fsiMeshD);

        printf("=================================================================================================\n");
    }
}

void ChSystemFsi::DoStepDynamics_ChronoRK2() {
    fsiInterface->Copy_ChSystem_to_External();
    mTime += 0.5 * paramsH->dT;

    sysMBS.DoStepDynamics(0.5 * paramsH->dT);
    mTime -= 0.5 * paramsH->dT;
    fsiInterface->Copy_External_To_ChSystem();
    mTime += paramsH->dT;
    sysMBS.DoStepDynamics(1.0 * paramsH->dT);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::WriteParticleFile(const std::string& outfilename) const {
    if (file_write_mode == CHFSI_OUTPUT_MODE::CSV) {
        utils::WriteCsvParticlesToFile(sysFSI.sphMarkersD2->posRadD, sysFSI.sphMarkersD2->velMasD,
                                       sysFSI.sphMarkersD2->rhoPresMuD, sysFSI.fsiGeneralData->referenceArray,
                                       outfilename);
    } else if (file_write_mode == CHFSI_OUTPUT_MODE::CHPF) {
        utils::WriteChPFParticlesToFile(sysFSI.sphMarkersD2->posRadD, sysFSI.fsiGeneralData->referenceArray,
                                        outfilename);
    }
}

void ChSystemFsi::PrintParticleToFile(const std::string& out_dir) const {
    utils::PrintToFile(sysFSI.sphMarkersD2->posRadD, sysFSI.sphMarkersD2->velMasD, sysFSI.sphMarkersD2->rhoPresMuD,
                       sysFSI.fsiGeneralData->sr_tau_I_mu_i, sysFSI.fsiGeneralData->referenceArray,
                       thrust::host_vector<int4>(), out_dir, paramsH, true);
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
    sysFSI.AddSphMarker(ChUtilsTypeConvert::ChVectorToReal4(point, h), mR4(rho0, pres0, mu0, particle_type),
                        ChUtilsTypeConvert::ChVectorToReal3(velocity), ChUtilsTypeConvert::ChVectorToReal3(tauXxYyZz),
                        ChUtilsTypeConvert::ChVectorToReal3(tauXyXzYz));
}

void ChSystemFsi::AddRefArray(const int start, const int numPart, const int compType, const int phaseType) {
    sysFSI.fsiGeneralData->referenceArray.push_back(mI4(start, numPart, compType, phaseType));
}

void ChSystemFsi::AddSphMarkerBox(double initSpace,
                                  double kernelLength,
                                  const ChVector<>& boxCenter,
                                  const ChVector<>& boxHalfDim) {
    // Use a chrono sampler to create a bucket of points
    chrono::utils::GridSampler<> sampler(initSpace);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        AddSphMarker(points[i], paramsH->rho0, 0, paramsH->mu0, kernelLength, -1,
                     ChVector<>(0),   // initial velocity
                     ChVector<>(0),   // tauxxyyzz
                     ChVector<>(0));  // tauxyxzyz
    }
    AddRefArray(0, (int)numPart, -1, -1);
}

void ChSystemFsi::AddBceBox(std::shared_ptr<ChBody> body,
                            const ChVector<>& relPos,
                            const ChQuaternion<>& relRot,
                            const ChVector<>& size,
                            int plane,
                            bool isSolid) {
    thrust::host_vector<Real4> posRadBCE;
    utils::CreateBCE_On_Box(posRadBCE, ChUtilsTypeConvert::ChVectorToReal3(size), plane, paramsH);
    CreateBceGlobalMarkersFromBceLocalPosBoundary(posRadBCE, body, relPos, relRot, isSolid, false);
    posRadBCE.clear();
}

void ChSystemFsi::AddBceSphere(std::shared_ptr<ChBody> body,
                               const ChVector<>& relPos,
                               const ChQuaternion<>& relRot,
                               double radius) {
    thrust::host_vector<Real4> posRadBCE;
    utils::CreateBCE_On_Sphere(posRadBCE, radius, paramsH);
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body);
    posRadBCE.clear();
}

void ChSystemFsi::AddBceSphereSurface(std::shared_ptr<ChBody> body,
                                      const ChVector<>& relPos,
                                      const ChQuaternion<>& relRot,
                                      Real radius,
                                      Real kernel_h) {
    thrust::host_vector<Real4> posRadBCE;
    thrust::host_vector<Real3> normals;
    utils::CreateBCE_On_surface_of_Sphere(posRadBCE, radius, kernel_h);
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body, relPos, relRot, false, true);
    posRadBCE.clear();
    normals.clear();
}

void ChSystemFsi::AddBceCylinder(std::shared_ptr<ChBody> body,
                                 const ChVector<>& relPos,
                                 const ChQuaternion<>& relRot,
                                 double radius,
                                 double height,
                                 double kernel_h,
                                 bool cartesian) {
    thrust::host_vector<Real4> posRadBCE;
    utils::CreateBCE_On_Cylinder(posRadBCE, radius, height, paramsH, kernel_h, cartesian);
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body, relPos, relRot);
    posRadBCE.clear();
}

void ChSystemFsi::AddBceCylinderSurface(std::shared_ptr<ChBody> body,
                                        const ChVector<>& relPos,
                                        const ChQuaternion<>& relRot,
                                        Real radius,
                                        Real height,
                                        Real kernel_h) {
    thrust::host_vector<Real4> posRadBCE;
    thrust::host_vector<Real3> normals;
    utils::CreateBCE_On_surface_of_Cylinder(posRadBCE, normals, radius, height, kernel_h);
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body, relPos, relRot, false, true);
    posRadBCE.clear();
    normals.clear();
}

void ChSystemFsi::AddBceCone(std::shared_ptr<ChBody> body,
                             const ChVector<>& relPos,
                             const ChQuaternion<>& relRot,
                             double radius,
                             double height,
                             double kernel_h,
                             bool cartesian) {
    thrust::host_vector<Real4> posRadBCE;
    utils::CreateBCE_On_Cone(posRadBCE, radius, height, paramsH, kernel_h, cartesian);
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body, relPos, relRot);
    posRadBCE.clear();
}

void ChSystemFsi::AddBceFromPoints(std::shared_ptr<ChBody> body,
                                   const std::vector<chrono::ChVector<>>& points,
                                   const ChVector<>& collisionShapeRelativePos,
                                   const ChQuaternion<>& collisionShapeRelativeRot) {
    thrust::host_vector<Real4> posRadBCE;
    for (auto& p : points)
        posRadBCE.push_back(mR4(p.x(), p.y(), p.z(), paramsH->HSML));
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body, collisionShapeRelativePos, collisionShapeRelativeRot);
}

void ChSystemFsi::AddBceFile(std::shared_ptr<ChBody> body,
                             const std::string& dataPath,
                             const ChVector<>& collisionShapeRelativePos,
                             const ChQuaternion<>& collisionShapeRelativeRot,
                             double scale,
                             bool isSolid)  // true means moving body, false means fixed boundary
{
    thrust::host_vector<Real4> posRadBCE;
    utils::LoadBCE_fromFile(posRadBCE, dataPath, scale, paramsH->HSML);
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body, collisionShapeRelativePos, collisionShapeRelativeRot,
                                          isSolid);
    posRadBCE.clear();
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::AddBceFromMesh(std::shared_ptr<fea::ChMesh> my_mesh,
                                 const std::vector<std::vector<int>>& NodeNeighborElement,
                                 const std::vector<std::vector<int>>& _1D_elementsNodes,
                                 const std::vector<std::vector<int>>& _2D_elementsNodes,
                                 bool add1DElem,
                                 bool add2DElem,
                                 bool multiLayer,
                                 bool removeMiddleLayer,
                                 int SIDE,
                                 int SIDE2D) {
    double kernel_h = 0;

    thrust::host_vector<Real4> posRadBCE;
    int numElems = my_mesh->GetNelements();
    std::vector<int> remove2D;
    std::vector<int> remove1D;

    for (size_t i = 0; i < my_mesh->GetNnodes(); i++) {
        auto thisNode = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(my_mesh->GetNode((unsigned int)i));
        fsiNodes.push_back(thisNode);
    }

    for (size_t i = 0; i < numElems; i++) {
        // Check for Cable Elements
        if (_1D_elementsNodes.size() > 0) {
            if (auto thisCable =
                    std::dynamic_pointer_cast<fea::ChElementCableANCF>(my_mesh->GetElement((unsigned int)i))) {
                remove1D.resize(2);
                std::fill(remove1D.begin(), remove1D.end(), 0);
                fsiCables.push_back(thisCable);

                size_t myNumNodes = (_1D_elementsNodes[i].size() > 2) ? 2 : _1D_elementsNodes[i].size();
                for (size_t j = 0; j < myNumNodes; j++) {
                    int thisNode = _1D_elementsNodes[i][j];

                    // Look into the elements attached to thisNode
                    for (size_t k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                        int neighborElement = NodeNeighborElement[thisNode][k];
                        if (neighborElement >= i)
                            continue;
                        remove1D[j] = 1;
                    }
                }

                if (add1DElem) {
                    utils::CreateBCE_On_ChElementCableANCF(posRadBCE, paramsH, thisCable, remove1D, multiLayer,
                                                           removeMiddleLayer, SIDE);
                    CreateBceGlobalMarkersFromBceLocalPos_CableANCF(posRadBCE, thisCable);
                }
                posRadBCE.clear();
            }
        }
        size_t Curr_size = _1D_elementsNodes.size();

        // Check for Shell Elements
        if (_2D_elementsNodes.size() > 0) {
            if (auto thisShell =
                    std::dynamic_pointer_cast<fea::ChElementShellANCF_3423>(my_mesh->GetElement((unsigned int)i))) {
                remove2D.resize(4);
                std::fill(remove2D.begin(), remove2D.begin() + 4, 0);

                fsiShells.push_back(thisShell);
                // Look into the nodes of this element
                size_t myNumNodes =
                    (_2D_elementsNodes[i - Curr_size].size() > 4) ? 4 : _2D_elementsNodes[i - Curr_size].size();

                for (size_t j = 0; j < myNumNodes; j++) {
                    int thisNode = _2D_elementsNodes[i - Curr_size][j];
                    // Look into the elements attached to thisNode
                    for (size_t k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                        // If this neighbor element has more than one common node with the previous
                        // node this means that we must not add BCEs to this edge anymore. Because
                        // that edge has already been given BCE particles.
                        // The kth element of this node:
                        size_t neighborElement = NodeNeighborElement[thisNode][k] - Curr_size;
                        if (neighborElement >= i - Curr_size)
                            continue;

                        size_t JNumNodes = (_2D_elementsNodes[neighborElement].size() > 4)
                                               ? 4
                                               : _2D_elementsNodes[neighborElement].size();

                        for (size_t inode = 0; inode < myNumNodes; inode++) {
                            for (size_t jnode = 0; jnode < JNumNodes; jnode++) {
                                if (_2D_elementsNodes[i - Curr_size][inode] ==
                                        _2D_elementsNodes[neighborElement][jnode] &&
                                    thisNode != _2D_elementsNodes[i - Curr_size][inode] && i > neighborElement) {
                                    remove2D[inode] = 1;
                                }
                            }
                        }
                    }
                }
                if (add2DElem) {
                    utils::CreateBCE_On_ChElementShellANCF(posRadBCE, paramsH, thisShell, remove2D, multiLayer,
                                                           removeMiddleLayer, SIDE2D, kernel_h);
                    CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(posRadBCE, thisShell, kernel_h);
                }
                posRadBCE.clear();
            }
        }
    }
}

void ChSystemFsi::AddBCE_ShellANCF(std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells,
                                   std::shared_ptr<fea::ChMesh> my_mesh,
                                   bool multiLayer,
                                   bool removeMiddleLayer,
                                   int SIDE) {
    thrust::host_vector<Real4> posRadBCE;
    int numShells = my_mesh->GetNelements();
    printf("number of shells to be meshed is %d\n", numShells);
    for (size_t i = 0; i < numShells; i++) {
        auto thisShell = std::dynamic_pointer_cast<fea::ChElementShellANCF_3423>(my_mesh->GetElement((unsigned int)i));
        fsiShells.push_back(thisShell);
        utils::CreateBCE_On_shell(posRadBCE, paramsH, thisShell, multiLayer, removeMiddleLayer, SIDE);
        CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(posRadBCE, thisShell);

        posRadBCE.clear();
    }
}

void ChSystemFsi::AddBCE_ShellFromMesh(std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells,
                                       std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& fsiNodes,
                                       std::shared_ptr<fea::ChMesh> my_mesh,
                                       const std::vector<std::vector<int>>& elementsNodes,
                                       const std::vector<std::vector<int>>& NodeNeighborElement,
                                       bool multiLayer,
                                       bool removeMiddleLayer,
                                       int SIDE) {
    thrust::host_vector<Real4> posRadBCE;
    int numShells = my_mesh->GetNelements();
    std::vector<int> remove;

    for (size_t i = 0; i < NodeNeighborElement.size(); i++) {
        auto thisNode = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(my_mesh->GetNode((unsigned int)i));
        fsiNodes.push_back(thisNode);
    }

    for (size_t i = 0; i < numShells; i++) {
        remove.resize(4);
        std::fill(remove.begin(), remove.begin() + 4, 0);
        auto thisShell = std::dynamic_pointer_cast<fea::ChElementShellANCF_3423>(my_mesh->GetElement((unsigned int)i));
        fsiShells.push_back(thisShell);
        // Look into the nodes of this element
        size_t myNumNodes = (elementsNodes[i].size() > 4) ? 4 : elementsNodes[i].size();

        for (size_t j = 0; j < myNumNodes; j++) {
            int thisNode = elementsNodes[i][j] - 1;
            // Look into the elements attached to thisNode
            for (size_t k = 0; k < NodeNeighborElement[thisNode].size(); k++) {
                // If this neighbor element has more than one common node with the previous
                // node this means that we must not add BCEs to this edge anymore. Because
                // that edge has already been given BCE particles.
                // The kth element of this node:
                int neighborElement = NodeNeighborElement[thisNode][k];
                if (neighborElement >= i)
                    continue;

                size_t JNumNodes =
                    (elementsNodes[neighborElement].size() > 4) ? 4 : elementsNodes[neighborElement].size();

                for (size_t inode = 0; inode < myNumNodes; inode++) {
                    for (int jnode = 0; jnode < JNumNodes; jnode++) {
                        if (elementsNodes[i][inode] - 1 == elementsNodes[neighborElement][jnode] - 1 &&
                            thisNode != elementsNodes[i][inode] - 1 && i > neighborElement) {
                            remove[inode] = 1;
                        }
                    }
                }
            }
        }
        utils::CreateBCE_On_ChElementShellANCF(posRadBCE, paramsH, thisShell, remove, multiLayer, removeMiddleLayer,
                                               SIDE);
        CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(posRadBCE, thisShell);
        posRadBCE.clear();
    }
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::CreateSphereFSI(std::shared_ptr<ChMaterialSurface> mat_prop,
                                  Real density,
                                  const ChVector<>& pos,
                                  Real radius) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetPos(pos);
    double volume = chrono::utils::CalcSphereVolume(radius);
    ChVector<> gyration = chrono::utils::CalcSphereGyration(radius).diagonal();
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddSphereGeometry(body.get(), mat_prop, radius);
    body->GetCollisionModel()->BuildModel();
    sysMBS.AddBody(body);
    fsiBodies.push_back(body);

    AddBceSphere(body, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), radius);
}

void ChSystemFsi::CreateCylinderFSI(std::shared_ptr<ChMaterialSurface> mat_prop,
                                    Real density,
                                    const ChVector<>& pos,
                                    const ChQuaternion<>& rot,
                                    Real radius,
                                    Real length) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetPos(pos);
    body->SetRot(rot);
    double volume = chrono::utils::CalcCylinderVolume(radius, 0.5 * length);
    ChVector<> gyration = chrono::utils::CalcCylinderGyration(radius, 0.5 * length).diagonal();
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddCylinderGeometry(body.get(), mat_prop, radius, 0.5 * length);
    body->GetCollisionModel()->BuildModel();
    sysMBS.AddBody(body);

    fsiBodies.push_back(body);
    AddBceCylinder(body, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), radius, length,
                   paramsH->HSML * paramsH->MULT_INITSPACE);
}

void ChSystemFsi::CreateBoxFSI(std::shared_ptr<ChMaterialSurface> mat_prop,
                               Real density,
                               const ChVector<>& pos,
                               const ChQuaternion<>& rot,
                               const ChVector<>& hsize) {
    auto body = chrono_types::make_shared<ChBody>();
    body->SetBodyFixed(false);
    body->SetCollide(true);
    body->SetPos(pos);
    body->SetRot(rot);
    double volume = chrono::utils::CalcBoxVolume(hsize);
    ChVector<> gyration = chrono::utils::CalcBoxGyration(hsize).diagonal();
    double mass = density * volume;
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);
    //
    body->GetCollisionModel()->ClearModel();
    chrono::utils::AddBoxGeometry(body.get(), mat_prop, hsize);
    body->GetCollisionModel()->BuildModel();
    sysMBS.AddBody(body);

    fsiBodies.push_back(body);
    AddBceBox(body, ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), hsize);
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChSystemFsi::CreateBceGlobalMarkersFromBceLocalPos(const thrust::host_vector<Real4>& posRadBCE,
                                                        std::shared_ptr<ChBody> body,
                                                        const ChVector<>& collisionShapeRelativePos,
                                                        const ChQuaternion<>& collisionShapeRelativeRot,
                                                        bool isSolid,
                                                        bool add_to_fluid_helpers,
                                                        bool add_to_previous_object) {
    if (sysFSI.fsiGeneralData->referenceArray.size() < 1 && !add_to_fluid_helpers) {
        printf(
            "\n\n\n\n Error! fluid need to be initialized before boundary. "
            "Reference array should have two components. \n\n\n\n");
        std::cin.get();
    }

    if (sysFSI.fsiGeneralData->referenceArray.size() == 0)
        sysFSI.fsiGeneralData->referenceArray.push_back(mI4(0, (int)posRadBCE.size(), -3, -1));

    int4 refSize4 = sysFSI.fsiGeneralData->referenceArray.back();
    int type = 0;
    int object = 0;
    if (isSolid) {
        object = refSize4.w + !add_to_previous_object;
        type = 1;
        printf("Adding BCE to solid object %d, type is %d, ref size = %zd\n", object, type,
               sysFSI.fsiGeneralData->referenceArray.size() + 1);
    }

    if (type < 0) {
        printf(
            "\n\n\n\n Error! reference array type is not correct."
            "It does not denote boundary or rigid. \n\n\n\n");
        std::cin.get();
    }

    if (add_to_fluid_helpers)
        type = -3;

    for (size_t i = 0; i < posRadBCE.size(); i++) {
        ChVector<> posLoc_collisionShape = ChUtilsTypeConvert::Real3ToChVector(mR3(posRadBCE[i]));
        ChVector<> posLoc_body = ChTransform<>::TransformLocalToParent(posLoc_collisionShape, collisionShapeRelativePos,
                                                                       collisionShapeRelativeRot);
        ChVector<> posLoc_COG = utils::TransformBCEToCOG(body, posLoc_body);
        ChVector<> posGlob = ChTransform<>::TransformLocalToParent(posLoc_COG, body->GetPos(), body->GetRot());
        sysFSI.sphMarkersH->posRadH.push_back(mR4(ChUtilsTypeConvert::ChVectorToReal3(posGlob), posRadBCE[i].w));

        ChVector<> vAbs = body->PointSpeedLocalToParent(posLoc_COG);
        Real3 v3 = ChUtilsTypeConvert::ChVectorToReal3(vAbs);
        sysFSI.sphMarkersH->velMasH.push_back(v3);
        sysFSI.sphMarkersH->rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, (double)type));
        sysFSI.sphMarkersH->tauXxYyZzH.push_back(mR3(0.0));
        sysFSI.sphMarkersH->tauXyXzYzH.push_back(mR3(0.0));
    }

    // ------------------------
    // Modify number of objects
    // ------------------------
    size_t numBce = posRadBCE.size();
    printf("Particle Type = %d ", type);
    printf("Pushing back to reference array\n");

    sysFSI.numObjects->numAllMarkers += numBce;
    // For helper particles, type = -3
    if (type == -3 && sysFSI.fsiGeneralData->referenceArray.size() != 1) {
        sysFSI.fsiGeneralData->referenceArray.push_back(mI4(refSize4.y, refSize4.y + (int)posRadBCE.size(), -3, -1));
    }
    // For boundary particles, type = 0
    else if ((type == 0 || (add_to_previous_object && type == 1)) && !add_to_fluid_helpers) {
        sysFSI.numObjects->numBoundaryMarkers += numBce;
        if (refSize4.w == -1) {
            sysFSI.fsiGeneralData->referenceArray.push_back(mI4(refSize4.y, refSize4.y + (int)numBce, 0, 0));
        } else if (refSize4.w == 0 || (refSize4.w && add_to_previous_object)) {
            refSize4.y = refSize4.y + (int)numBce;
            sysFSI.fsiGeneralData->referenceArray.back() = refSize4;
        }
    }
    // For rigid body particles, type = 1
    else if (!add_to_fluid_helpers) {
        if (sysFSI.fsiGeneralData->referenceArray.size() < 2) {
            printf(
                "Error! Boundary particles are not initialized while trying to "
                "initialize rigid particle!\n\n");
            std::cin.get();
        }
        sysFSI.numObjects->numRigid_SphMarkers += numBce;
        sysFSI.numObjects->numRigidBodies += 1;
        sysFSI.numObjects->startRigidMarkers = sysFSI.fsiGeneralData->referenceArray[1].y;
        sysFSI.fsiGeneralData->referenceArray.push_back(mI4(refSize4.y, refSize4.y + (int)numBce, 1, object));
        printf("refSize4.y = %d, refSize4.y + numBce = %d, particle type = %d, object number = %d\n", refSize4.y,
               refSize4.y + (int)numBce, 1, object);
    }
}

void ChSystemFsi::CreateBceGlobalMarkersFromBceLocalPos_CableANCF(const thrust::host_vector<Real4>& posRadBCE,
                                                                  std::shared_ptr<fea::ChElementCableANCF> cable) {
    int type = 2;

    fea::ChElementCableANCF::ShapeVector N;
    double dx = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).Length();

    ChVector<> Element_Axis = (cable->GetNodeB()->GetX0() - cable->GetNodeA()->GetX0()).GetNormalized();
    printf(" Element_Axis= %f, %f, %f\n", Element_Axis.x(), Element_Axis.y(), Element_Axis.z());

    ChVector<> Old_axis = ChVector<>(1, 0, 0);
    ChQuaternion<double> Rotation = (Q_from_Vect_to_Vect(Old_axis, Element_Axis));
    Rotation.Normalize();
    ChVector<> new_y_axis = Rotation.Rotate(ChVector<>(0, 1, 0));
    ChVector<> new_z_axis = Rotation.Rotate(ChVector<>(0, 0, 1));

    ChVector<> physic_to_natural(1 / dx, 1, 1);

    ChVector<> nAp = cable->GetNodeA()->GetPos();
    ChVector<> nBp = cable->GetNodeB()->GetPos();

    ChVector<> nAv = cable->GetNodeA()->GetPos_dt();
    ChVector<> nBv = cable->GetNodeB()->GetPos_dt();

    int posRadSizeModified = 0;

    printf(" posRadBCE.size()= :%zd\n", posRadBCE.size());
    for (size_t i = 0; i < posRadBCE.size(); i++) {
        ChVector<> pos_physical = ChUtilsTypeConvert::Real3ToChVector(mR3(posRadBCE[i]));

        ChVector<> pos_natural = pos_physical * physic_to_natural;

        cable->ShapeFunctions(N, pos_natural.x());

        Real2 NFSI = Cables_ShapeFunctions(pos_natural.x());
        ChVector<> NFSI_Chvector = ChUtilsTypeConvert::Real2ToChVector(NFSI);

        ChVector<> Normal;

        ChVector<> Correct_Pos = NFSI_Chvector.x() * nAp + NFSI_Chvector.y() * nBp + new_y_axis * pos_physical.y() +
                                 new_z_axis * pos_physical.z();

        printf(" physic_to_natural is = (%f,%f,%f)\n", physic_to_natural.x(), physic_to_natural.y(),
               physic_to_natural.z());
        printf(" pos_physical is = (%f,%f,%f)\n", pos_physical.x(), pos_physical.y(), pos_physical.z());
        printf(" pos_natural is = (%f,%f,%f)\n ", pos_natural.x(), pos_natural.y(), pos_natural.z());
        printf(" Correct_Pos is = (%f,%f,%f)\n\n\n ", Correct_Pos.x(), Correct_Pos.y(), Correct_Pos.z());

        if ((Correct_Pos.x() < paramsH->cMin.x || Correct_Pos.x() > paramsH->cMax.x) ||
            (Correct_Pos.y() < paramsH->cMin.y || Correct_Pos.y() > paramsH->cMax.y) ||
            (Correct_Pos.z() < paramsH->cMin.z || Correct_Pos.z() > paramsH->cMax.z)) {
            continue;
        }

        bool addthis = true;
        for (int p = 0; p < sysFSI.sphMarkersH->posRadH.size() - 1; p++) {
            if (length(mR3(sysFSI.sphMarkersH->posRadH[p]) - ChUtilsTypeConvert::ChVectorToReal3(Correct_Pos)) < 1e-5 &&
                sysFSI.sphMarkersH->rhoPresMuH[p].w != -1) {
                addthis = false;
                printf("remove this particle %f,%f,%f because of its overlap with a particle at %f,%f,%f\n",
                       sysFSI.sphMarkersH->posRadH[p].x, sysFSI.sphMarkersH->posRadH[p].y,
                       sysFSI.sphMarkersH->posRadH[p].z, Correct_Pos.x(), Correct_Pos.y(), Correct_Pos.z());

                break;
            }
        }

        if (addthis) {
            sysFSI.sphMarkersH->posRadH.push_back(
                mR4(ChUtilsTypeConvert::ChVectorToReal3(Correct_Pos), posRadBCE[i].w));
            sysFSI.fsiGeneralData->FlexSPH_MeshPos_LRF_H.push_back(ChUtilsTypeConvert::ChVectorToReal3(pos_physical));
            ChVector<> Correct_Vel = N(0) * nAv + N(2) * nBv + ChVector<double>(1e-20);
            Real3 v3 = ChUtilsTypeConvert::ChVectorToReal3(Correct_Vel);
            sysFSI.sphMarkersH->velMasH.push_back(v3);
            sysFSI.sphMarkersH->rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, type));
            posRadSizeModified++;
        }
    }

    // ------------------------
    // Modify number of objects
    // ------------------------
    size_t numObjects = sysFSI.fsiGeneralData->referenceArray.size();
    size_t numBce = posRadSizeModified;
    sysFSI.numObjects->numAllMarkers += numBce;

    int numRigid = (int)sysFSI.numObjects->numRigidBodies;
    sysFSI.numObjects->numFlex_SphMarkers += numBce;
    sysFSI.numObjects->numFlexBodies1D += 1;
    sysFSI.numObjects->startFlexMarkers = sysFSI.fsiGeneralData->referenceArray[numRigid + 1].y;

    int4 last = sysFSI.fsiGeneralData->referenceArray[sysFSI.fsiGeneralData->referenceArray.size() - 1];
    sysFSI.fsiGeneralData->referenceArray.push_back(
        mI4(last.y, last.y + (int)numBce, type, (int)sysFSI.numObjects->numFlexBodies1D));  // 2: for cable

    sysFSI.fsiGeneralData->referenceArray_FEA.push_back(
        mI4(last.y, last.y + (int)numBce, type, (int)sysFSI.numObjects->numFlexBodies1D));  // 2: for cable

    printf(" push_back Index %zd. ", sysFSI.fsiGeneralData->referenceArray.size() - 1);
    int4 test = sysFSI.fsiGeneralData->referenceArray[sysFSI.fsiGeneralData->referenceArray.size() - 1];
    printf(" x=%d, y=%d, z=%d, w=%d\n", test.x, test.y, test.z, test.w);

    if (sysFSI.numObjects->numFlexBodies1D !=
        sysFSI.fsiGeneralData->referenceArray.size() - 2 - sysFSI.numObjects->numRigidBodies) {
        printf("Error! num rigid Flexible does not match reference array size!\n\n");
        std::cin.get();
    }
    numObjects = sysFSI.fsiGeneralData->referenceArray.size();
    printf("numObjects : %zd\n ", numObjects);
    printf("numObjects->startFlexMarkers  : %zd\n ", sysFSI.numObjects->startFlexMarkers);
}

void ChSystemFsi::CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(const thrust::host_vector<Real4>& posRadBCE,
                                                                  std::shared_ptr<fea::ChElementShellANCF_3423> shell,
                                                                  double kernel_h) {
    int type = 3;

    fea::ChElementShellANCF_3423::ShapeVector N;
    size_t posRadSizeModified = 0;

    double my_h = (kernel_h == 0) ? paramsH->HSML : kernel_h;

    Real dx = shell->GetLengthX();
    Real dy = shell->GetLengthY();
    ChVector<> physic_to_natural(2 / dx, 2 / dy, 1);
    ChVector<> nAp = shell->GetNodeA()->GetPos();
    ChVector<> nBp = shell->GetNodeB()->GetPos();
    ChVector<> nCp = shell->GetNodeC()->GetPos();
    ChVector<> nDp = shell->GetNodeD()->GetPos();

    ChVector<> nAv = shell->GetNodeA()->GetPos_dt();
    ChVector<> nBv = shell->GetNodeB()->GetPos_dt();
    ChVector<> nCv = shell->GetNodeC()->GetPos_dt();
    ChVector<> nDv = shell->GetNodeD()->GetPos_dt();

    printf(" posRadBCE.size()= :%zd\n", posRadBCE.size());
    for (size_t i = 0; i < posRadBCE.size(); i++) {
        ChVector<> pos_physical = ChUtilsTypeConvert::Real3ToChVector(mR3(posRadBCE[i]));
        ChVector<> pos_natural = pos_physical * physic_to_natural;

        shell->ShapeFunctions(N, pos_natural.x(), pos_natural.y(), pos_natural.z());
        ChVector<> x_dir = (nBp - nAp + nCp - nDp);
        ChVector<> y_dir = (nCp - nBp + nDp - nAp);
        ChVector<> Normal;
        Normal.Cross(x_dir, y_dir);
        Normal.Normalize();

        ChVector<> Correct_Pos = N(0) * nAp + N(2) * nBp + N(4) * nCp + N(6) * nDp +
                                 Normal * pos_physical.z() * my_h * paramsH->MULT_INITSPACE_Shells;

        if ((Correct_Pos.x() < paramsH->cMin.x || Correct_Pos.x() > paramsH->cMax.x) ||
            (Correct_Pos.y() < paramsH->cMin.y || Correct_Pos.y() > paramsH->cMax.y) ||
            (Correct_Pos.z() < paramsH->cMin.z || Correct_Pos.z() > paramsH->cMax.z)) {
            continue;
        }

        // Note that the fluid particles are removed differently
        bool addthis = true;
        for (size_t p = 0; p < sysFSI.sphMarkersH->posRadH.size() - 1; p++) {
            if (length(mR3(sysFSI.sphMarkersH->posRadH[p]) - ChUtilsTypeConvert::ChVectorToReal3(Correct_Pos)) < 1e-8 &&
                sysFSI.sphMarkersH->rhoPresMuH[p].w != -1) {
                addthis = false;
                break;
            }
        }

        if (addthis) {
            sysFSI.sphMarkersH->posRadH.push_back(
                mR4(ChUtilsTypeConvert::ChVectorToReal3(Correct_Pos), posRadBCE[i].w));
            sysFSI.fsiGeneralData->FlexSPH_MeshPos_LRF_H.push_back(ChUtilsTypeConvert::ChVectorToReal3(pos_natural));

            ChVector<> Correct_Vel = N(0) * nAv + N(2) * nBv + N(4) * nCv + N(6) * nDv;
            Real3 v3 = ChUtilsTypeConvert::ChVectorToReal3(Correct_Vel);
            sysFSI.sphMarkersH->velMasH.push_back(v3);
            sysFSI.sphMarkersH->rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, type));
            posRadSizeModified++;
        }
    }
    sysFSI.sphMarkersH->rhoPresMuH.size();

    // ------------------------
    // Modify number of objects
    // ------------------------
    size_t numObjects = sysFSI.fsiGeneralData->referenceArray.size();
    size_t numBce = posRadSizeModified;
    sysFSI.numObjects->numAllMarkers += numBce;

    int numRigid = (int)sysFSI.numObjects->numRigidBodies;
    sysFSI.numObjects->numFlex_SphMarkers += numBce;
    sysFSI.numObjects->numFlexBodies2D += 1;
    sysFSI.numObjects->startFlexMarkers = sysFSI.fsiGeneralData->referenceArray[numRigid + 1].y;

    int4 last = sysFSI.fsiGeneralData->referenceArray[sysFSI.fsiGeneralData->referenceArray.size() - 1];
    sysFSI.fsiGeneralData->referenceArray.push_back(
        mI4(last.y, last.y + (int)numBce, type, (int)sysFSI.numObjects->numFlexBodies2D));  // 3: for Shell

    sysFSI.fsiGeneralData->referenceArray_FEA.push_back(
        mI4(last.y, last.y + (int)numBce, type, (int)sysFSI.numObjects->numFlexBodies2D));  // 3: for Shell

    printf(" referenceArray size %zd. ", sysFSI.fsiGeneralData->referenceArray.size());
    int4 test = sysFSI.fsiGeneralData->referenceArray[sysFSI.fsiGeneralData->referenceArray.size() - 1];
    printf(" x=%d, y=%d, z=%d, w=%d\n", test.x, test.y, test.z, test.w);

    if (sysFSI.numObjects->numFlexBodies2D != sysFSI.fsiGeneralData->referenceArray.size() - 2 -
                                                  sysFSI.numObjects->numRigidBodies -
                                                  sysFSI.numObjects->numFlexBodies1D) {
        printf("Error! num rigid Flexible does not match reference array size!\n\n");
        std::cin.get();
    }
    numObjects = sysFSI.fsiGeneralData->referenceArray.size();
    printf("numObjects : %zd\n ", numObjects);
}

void ChSystemFsi::CreateBceGlobalMarkersFromBceLocalPosBoundary(const thrust::host_vector<Real4>& posRadBCE,
                                                                std::shared_ptr<ChBody> body,
                                                                const ChVector<>& collisionShapeRelativePos,
                                                                const ChQuaternion<>& collisionShapeRelativeRot,
                                                                bool isSolid,
                                                                bool add_to_previous) {
    CreateBceGlobalMarkersFromBceLocalPos(posRadBCE, body, collisionShapeRelativePos, collisionShapeRelativeRot,
                                          isSolid, false, add_to_previous);
}

//--------------------------------------------------------------------------------------------------------------------------------

float ChSystemFsi::GetKernelLength() const {
    return paramsH->HSML;
}

float ChSystemFsi::GetInitialSpacing() const {
    return paramsH->INITSPACE;
}

size_t ChSystemFsi::GetNumFluidMarkers() const {
    return sysFSI.numObjects->numFluidMarkers;
}

size_t ChSystemFsi::GetNumRigidBodyMarkers() const {
    return sysFSI.numObjects->numRigid_SphMarkers;
}

size_t ChSystemFsi::GetNumFlexBodyMarkers() const {
    return sysFSI.numObjects->numFlex_SphMarkers;
}

size_t ChSystemFsi::GetNumBoundaryMarkers() const {
    return sysFSI.numObjects->numBoundaryMarkers;
}

std::vector<ChVector<>> ChSystemFsi::GetParticlePosOrProperties() {
    thrust::host_vector<Real4> posRadH = sysFSI.sphMarkersD2->posRadD;
    std::vector<ChVector<>> pos;
    for (size_t i = 0; i < posRadH.size(); i++) {
        pos.push_back(ChUtilsTypeConvert::Real4ToChVector(posRadH[i]));
    }
    return pos;
}

std::vector<ChVector<>> ChSystemFsi::GetParticleVel() {
    thrust::host_vector<Real3> velH = sysFSI.sphMarkersD2->velMasD;
    std::vector<ChVector<>> vel;
    for (size_t i = 0; i < velH.size(); i++) {
        vel.push_back(ChUtilsTypeConvert::Real3ToChVector(velH[i]));
    }
    return vel;
}

}  // end namespace fsi
}  // end namespace chrono
