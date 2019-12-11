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
// Author: Milad Rakhsha
// =============================================================================

// General Includes
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono general utils
#include "chrono/core/ChTransform.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

// Chrono fsi includes
#include "chrono_fsi/ChFsiTypeConvert.h"
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsJsonInput.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

// Chrono fea includes
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"

// Chrono namespaces
using namespace chrono;
using namespace fea;
using namespace collision;
using namespace filesystem;

using std::cout;
using std::endl;
std::ofstream simParams;

//****************************************************************************************
const std::string out_dir = GetChronoOutputPath() + "FSI_FLEXIBLE_SHELL/";
std::string demo_dir;
std::string MESH_CONNECTIVITY;
// Save data as csv files, turn it on to be able to see the results off-line using paraview

std::vector<std::vector<int>> NodeNeighborElement_mesh;

typedef fsi::Real Real;

// Dimension of the domain
Real bxDim = 3;
Real byDim = 0.2;
Real bzDim = 1.5;

// Dimension of the fluid domain
Real fxDim = 1;
Real fyDim = byDim;
Real fzDim = 1;

void SaveParaViewFiles(fsi::ChSystemFsi& myFsiSystem,
                       ChSystemSMC& mphysicalSystem,
                       std::shared_ptr<fea::ChMesh> my_mesh,
                       std::vector<std::vector<int>> NodeNeighborElementMesh,
                       fsi::SimParams* paramsH,
                       int next_frame,
                       double mTime);
void Create_MB_FE(ChSystemSMC& mphysicalSystem, fsi::ChSystemFsi& myFsiSystem, fsi::SimParams* paramsH);
//****************************************************************************************
void ShowUsage() {
    cout << "usage: ./demo_FSI_Flexible_Shell <json_file>" << endl;
}

int main(int argc, char* argv[]) {
    ChSystemSMC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(&mphysicalSystem, true, fsi::ChFluidDynamics::Integrator::IISPH);
    fsi::SimParams* paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Flexible_Shell.json");
    if (argc == 1 && fsi::utils::ParseJSON(inputJson.c_str(), paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
    } else if (argc == 2 && fsi::utils::ParseJSON(argv[1], paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
    } else {
        ShowUsage();
        return 1;
    }
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);
    paramsH->cMin = fsi::mR3(-bxDim, -byDim, -bzDim) - fsi::mR3(paramsH->HSML * 5);
    paramsH->cMax = fsi::mR3(bxDim, byDim, 1.2 * bzDim) + fsi::mR3(paramsH->HSML * 5);

    // call FinalizeDomainCreating to setup the binning for neighbor search or write your own
    fsi::utils::FinalizeDomainCreating(paramsH);

    if (strcmp(paramsH->out_name, "Undefined") == 0)
        demo_dir = out_dir + "Paraview/";
    else
        demo_dir = out_dir + paramsH->out_name + "/";

    MESH_CONNECTIVITY = demo_dir + "Flex_MESH.vtk";

    cout << MESH_CONNECTIVITY;

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(demo_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    const std::string removeFiles = (std::string("rm ") + demo_dir + "/* ");
    system(removeFiles.c_str());
    if (argc == 2) {
        const std::string copyInputs = (std::string("cp ") + argv[1] + " " + demo_dir);
        system(copyInputs.c_str());
    }

    paramsH->MULT_INITSPACE_Shells = 1;

    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    utils::GridSampler<> sampler(initSpace0);

    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0 * initSpace0, fzDim / 2 + 1 * initSpace0);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);

    utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    int numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        myFsiSystem.GetDataManager()->AddSphMarker(fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML),
                                                   fsi::mR3(1e-10),
                                                   fsi::mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, -1.0));
    }

    int numPhases = myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray.size();

    if (numPhases != 0) {
        std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
        std::cin.get();
        return -1;
    } else {
        myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray.push_back(mI4(0, numPart, -1, -1));
        myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray.push_back(mI4(numPart, numPart, 0, 0));
    }

    // ******************************* Create MBD or FE model ******************************
    ChVector<> gravity = ChVector<>(paramsH->gravity.x, paramsH->gravity.y, paramsH->gravity.z);
    Create_MB_FE(mphysicalSystem, myFsiSystem, paramsH);
    auto my_mesh = myFsiSystem.GetFsiMesh();

    myFsiSystem.Finalize();

    int step_count = 0;
    double mTime = 0;
#undef CHRONO_MKL
#ifdef CHRONO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
    mkl_solver->LockSparsityPattern(true);
    mphysicalSystem.SetSolver(mkl_solver);
#else
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    mphysicalSystem.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    mphysicalSystem.SetSolverForceTolerance(1e-10);
#endif

    //    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::HHT);
    //    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(mphysicalSystem.GetTimestepper());
    //    mystepper->SetAlpha(-0.2);
    //    mystepper->SetMaxiters(1000);
    //    mystepper->SetAbsTolerances(1e-5);
    //    mystepper->SetMode(ChTimestepperHHT::POSITION);
    //    mystepper->SetScaling(true);

    //    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    stepEnd = 1000000;
    SaveParaViewFiles(myFsiSystem, mphysicalSystem, my_mesh, NodeNeighborElement_mesh, paramsH, 0, mTime);

    Real time = 0;
    bool isAdaptive = false;
    Real Global_max_dT = paramsH->dT_Max;
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int next_frame = std::floor((time + 1e-6) / frame_time) + 1;
        double next_frame_time = next_frame * frame_time;
        double max_allowable_dt = next_frame_time - time;
        if (max_allowable_dt > 1e-6)
            paramsH->dT_Max = std::min(Global_max_dT, max_allowable_dt);
        else
            paramsH->dT_Max = Global_max_dT;

        printf("next_frame is:%d,  max dt is set to %f\n", next_frame, paramsH->dT_Max);
        if (tStep < 5 && paramsH->Adaptive_time_stepping) {
            paramsH->Adaptive_time_stepping = false;
            isAdaptive = true;
        }
        myFsiSystem.DoStepDynamics_FSI();
        paramsH->Adaptive_time_stepping = isAdaptive;
        time += paramsH->dT;
        SaveParaViewFiles(myFsiSystem, mphysicalSystem, my_mesh, NodeNeighborElement_mesh, paramsH, next_frame, time);

        if (time > paramsH->tFinal)
            break;
    }

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid/flexible bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------
void Create_MB_FE(ChSystemSMC& mphysicalSystem, fsi::ChSystemFsi& myFsiSystem, fsi::SimParams* paramsH) {
    mphysicalSystem.Set_G_acc(ChVector<>(0, 0, 0));
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    // Set common material Properties
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    ground->SetMaterialSurface(mysurfmaterial);

    ground->GetCollisionModel()->ClearModel();
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    // Bottom and top wall
    ChVector<> sizeBottom(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> posBot(0, 0, -2 * initSpace0);
    ChVector<> posTop(0, 0, bzDim + 2 * initSpace0);

    // left and right Wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);

    // Front and back Wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 1 * initSpace0);

    // MBD representation of walls
    utils::AddBoxGeometry(ground.get(), sizeBottom, posBot, QUNIT, true);
    utils::AddBoxGeometry(ground.get(), size_YZ, pos_xp, QUNIT, true);
    utils::AddBoxGeometry(ground.get(), size_YZ, pos_xn, QUNIT, true);
    utils::AddBoxGeometry(ground.get(), size_XZ, pos_yp, QUNIT, true);
    utils::AddBoxGeometry(ground.get(), size_XZ, pos_yn, QUNIT, true);
    mphysicalSystem.AddBody(ground);

    // Fluid representation of walls
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posBot, QUNIT, sizeBottom, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posTop, QUNIT, sizeBottom, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_xp, QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_xn, QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yp, QUNIT, size_XZ, 13);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yn, QUNIT, size_XZ, 13);

    // ******************************* Flexible bodies ***********************************
    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    std::vector<std::vector<int>> _1D_elementsNodes_mesh;
    std::vector<std::vector<int>> _2D_elementsNodes_mesh;

    int numFlexBody = 1;
    // Geometry of the plate
    double plate_lenght_x = 0.025;
    double plate_lenght_y = byDim;
    double plate_lenght_z = initSpace0 * 6;
    ChVector<> center_plate(bxDim / 8 + 3 * initSpace0, 0.0, plate_lenght_z / 2 + 1 * initSpace0);

    // Specification of the mesh
    int numDiv_x = 1;
    int numDiv_y = 2;
    int numDiv_z = 3;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_y * numDiv_z;
    int TotalNumNodes = (numDiv_y + 1) * (numDiv_z + 1);
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    _2D_elementsNodes_mesh.resize(TotalNumElements);
    NodeNeighborElement_mesh.resize(TotalNumNodes);
    // Create and add the nodes

    for (int k = 0; k < N_z; k++) {
        for (int j = 0; j < N_y; j++) {
            double loc_x = center_plate.x();
            double loc_y = j * dy - plate_lenght_y / 2 + center_plate.y();
            double loc_z = k * dz - plate_lenght_z / 2 + center_plate.z();
            // Node direction
            double dir_x = 1;
            double dir_y = 0;
            double dir_z = 0;

            // Create the node
            auto node =
                chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z));

            node->SetMass(0);
            // Fix all nodes along the axis X=0
            if (k == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes - 1));

    // Create an isotropic material.
    // All layers for all elements share the same material.
    double rho = 8000;
    double E = 5e6;
    double nu = 0.3;
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);
    // Create the elements

    int num_elem = 0;
    for (int k = 0; k < numDiv_z; k++) {
        for (int j = 0; j < numDiv_y; j++) {
            int node0 = (j + 0) + N_y * (k + 0);
            int node1 = (j + 1) + N_y * (k + 0);
            int node2 = (j + 1) + N_y * (k + 1);
            int node3 = (j + 0) + N_y * (k + 1);

            _2D_elementsNodes_mesh[num_elem].push_back(node0);
            _2D_elementsNodes_mesh[num_elem].push_back(node1);
            _2D_elementsNodes_mesh[num_elem].push_back(node2);
            _2D_elementsNodes_mesh[num_elem].push_back(node3);
            NodeNeighborElement_mesh[node0].push_back(num_elem);
            NodeNeighborElement_mesh[node1].push_back(num_elem);
            NodeNeighborElement_mesh[node2].push_back(num_elem);
            NodeNeighborElement_mesh[node3].push_back(num_elem);
            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

            // Set element dimensions
            element->SetDimensions(dy, dz);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dx, 0 * CH_C_DEG_TO_RAD, mat);

            // Set other element properties
            element->SetAlphaDamp(0.2);    // Structural damping for this element
            element->SetGravityOn(false);  // turn internal gravitational force calculation off

            // Add element to mesh
            my_mesh->AddElement(element);
            ChVector<> center = 0.25 * (element->GetNodeA()->GetPos() + element->GetNodeB()->GetPos() +
                                        element->GetNodeC()->GetPos() + element->GetNodeD()->GetPos());
            cout << "Adding element" << num_elem << "  with center:  " << center.x() << " " << center.y() << " "
                 << center.z() << endl;
            num_elem++;
        }
    }

    // Add the mesh to the system
    mphysicalSystem.Add(my_mesh);

    // fluid representation of flexible bodies
    bool multilayer = true;
    bool removeMiddleLayer = false;
    bool add1DElem = false;
    bool add2DElem = true;
    fsi::utils::AddBCE_FromMesh(myFsiSystem.GetDataManager(), paramsH, my_mesh, myFsiSystem.GetFsiNodesPtr(),
                                myFsiSystem.GetFsiCablesPtr(), myFsiSystem.GetFsiShellsPtr(), NodeNeighborElement_mesh,
                                _1D_elementsNodes_mesh, _2D_elementsNodes_mesh, add1DElem, add2DElem, multilayer,
                                removeMiddleLayer, 0, 0);

    myFsiSystem.SetShellElementsNodes(_2D_elementsNodes_mesh);
    myFsiSystem.SetFsiMesh(my_mesh);
    fea::ChMeshExporter::writeMesh(my_mesh, MESH_CONNECTIVITY);
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFiles(fsi::ChSystemFsi& myFsiSystem,
                       ChSystemSMC& mphysicalSystem,
                       std::shared_ptr<fea::ChMesh> my_mesh,
                       std::vector<std::vector<int>> NodeNeighborElementMesh,
                       fsi::SimParams* paramsH,
                       int next_frame,
                       double mTime) {
    static double exec_time;
    int out_steps = std::ceil((1.0 / paramsH->dT) / paramsH->out_fps);
    exec_time += mphysicalSystem.GetTimerStep();
    int num_contacts = mphysicalSystem.GetNcontacts();
    double frame_time = 1.0 / paramsH->out_fps;
    static int out_frame = 0;

    if (std::abs(mTime - (next_frame)*frame_time) < 1e-7) {
        fsi::utils::PrintToFile(myFsiSystem.GetDataManager()->sphMarkersD2.posRadD,
                                myFsiSystem.GetDataManager()->fsiGeneralData.vis_vel_SPH_D,
                                myFsiSystem.GetDataManager()->sphMarkersD2.rhoPresMuD,
                                myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray,
                                myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray_FEA, demo_dir, true);

        cout << "-------------------------------------\n" << endl;
        cout << "             Output frame:   " << next_frame << endl;
        cout << "             Time:           " << mTime << endl;
        cout << "-------------------------------------\n" << endl;

        char SaveAsBuffer[256];  // The filename buffer.
        snprintf(SaveAsBuffer, sizeof(char) * 256, (demo_dir + "/flex_body.%d.vtk").c_str(), next_frame);
        char MeshFileBuffer[256];  // The filename buffer.
        snprintf(MeshFileBuffer, sizeof(char) * 256, ("%s"), MESH_CONNECTIVITY.c_str());
        fea::ChMeshExporter::writeFrame(my_mesh, SaveAsBuffer, MESH_CONNECTIVITY);

        out_frame++;
    }
}
