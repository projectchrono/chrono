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

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fea;
using namespace chrono::collision;
using namespace chrono::fsi;

//****************************************************************************************
const std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Elements/";
std::string MESH_CONNECTIVITY = out_dir + "Flex_MESH.vtk";

// Save data for visualization
bool output = true;
double out_fps = 20;

std::vector<std::vector<int>> NodeNeighborElement_mesh;

// Dimension of the domain
double bxDim = 3;
double byDim = 0.2;
double bzDim = 1.5;

// Dimension of the fluid domain
double fxDim = 1;
double fyDim = byDim;
double fzDim = 1;
bool flexible_elem_1D = false;

// Final simulation time
double t_end = 10.0;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

//----------------------------------------------------------------------------------------

void Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI);

//----------------------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(sysMBS);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Flexible_Elements_I2SPH.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_Flexible_Shell <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    ChVector<> cMin = ChVector<>(-bxDim, -byDim, -bzDim) - ChVector<>(sysFSI.GetKernelLength() * 5);
    ChVector<> cMax = ChVector<>(bxDim, byDim, 1.2 * bzDim) + ChVector<>(sysFSI.GetKernelLength() * 5);
    sysFSI.SetBoundaries(cMin, cMax);

    // Setup the output directory for FSI data
    sysFSI.SetOutputDirectory(out_dir);

    // ******************************* Create Fluid region ****************************************
    ////paramsH->MULT_INITSPACE_Shells = 1;
    auto initSpace0 = sysFSI.GetInitialSpacing();
    chrono::utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0 * initSpace0, fzDim / 2 + 1 * initSpace0);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    chrono::utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysFSI.AddSPHParticle(points[i]);
    }

    // Create solids
    Create_MB_FE(sysMBS, sysFSI);
    sysFSI.Initialize();
    auto my_mesh = sysFSI.GetFsiMesh();

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI single wheel demo");
        fsi_vis.SetCameraPosition(ChVector<>(bxDim / 8, -3, 0.25), ChVector<>(bxDim / 8, 0.0, 0.25));
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.Initialize();
    }

    // Set MBS solver
#undef CHRONO_PARDISO_MKL
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sysMBS.SetSolver(mkl_solver);
#else
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sysMBS.SetSolver(solver);
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);

    sysMBS.SetSolverForceTolerance(1e-10);
#endif

    //    sysMBS.SetTimestepperType(ChTimestepper::Type::HHT);
    //    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sysMBS.GetTimestepper());
    //    mystepper->SetAlpha(-0.2);
    //    mystepper->SetMaxiters(1000);
    //    mystepper->SetAbsTolerances(1e-5);
    //    mystepper->SetMode(ChTimestepperHHT::POSITION);
    //    mystepper->SetScaling(true);

    //    sysMBS.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);

    // Simulation loop
    double dT = sysFSI.GetStepSize();

    unsigned int output_steps = (unsigned int)(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * dT));

    double time = 0.0;
    int current_step = 0;

    bool isAdaptive = false;

    ChTimer<> timer;
    timer.start();
    while (time < t_end) {
        std::cout << current_step << " time: " << time << std::endl;

        if (output && current_step % output_steps == 0) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            static int counter = 0;
            std::string filename = out_dir + "/vtk/flex_body." + std::to_string(counter++) + ".vtk";
            fea::ChMeshExporter::writeFrame(my_mesh, (char*)filename.c_str(), MESH_CONNECTIVITY);
        }

        // Render SPH particles
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        if (current_step < 5 && sysFSI.GetAdaptiveTimeStepping()) {
            sysFSI.SetAdaptiveTimeStepping(false);
            isAdaptive = true;
        }
        sysFSI.DoStepDynamics_FSI();
        sysFSI.SetAdaptiveTimeStepping(isAdaptive);

        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid/flexible bodies, and if fsi, their
// bce representation are created and added to the systems
void Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    sysMBS.Set_G_acc(ChVector<>(0.0, 0, 0));
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

    ground->GetCollisionModel()->ClearModel();
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Bottom and top wall
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 2 * initSpace0);
    ChVector<> pos_zn(0, 0, -2 * initSpace0);

    // left and right Wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);

    // Front and back Wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 1 * initSpace0);

    // MBD representation of walls
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XY, pos_zn, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xn, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yn, QUNIT, true);
    sysMBS.AddBody(ground);

    // Fluid representation of walls
    sysFSI.AddBoxBCE(ground, pos_zn, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(ground, pos_zp, QUNIT, size_XY, 12);
    sysFSI.AddBoxBCE(ground, pos_xp, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(ground, pos_xn, QUNIT, size_YZ, 23);
    sysFSI.AddBoxBCE(ground, pos_yp, QUNIT, size_XZ, 13);
    sysFSI.AddBoxBCE(ground, pos_yn, QUNIT, size_XZ, 13);

    // ******************************* Flexible bodies ***********************************
    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<fea::ChMesh>();
    std::vector<std::vector<int>> _1D_elementsNodes_mesh;
    std::vector<std::vector<int>> _2D_elementsNodes_mesh;
    if (flexible_elem_1D) {
        double E = 1e8;
        double rho = 8000;
        /*================== Cable Elements =================*/
        auto msection_cable = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable->SetDiameter(initSpace0);
        msection_cable->SetYoungModulus(E);
        msection_cable->SetDensity(rho);
        msection_cable->SetBeamRaleyghDamping(0.02);

        ChBuilderCableANCF builder;
        double loc_x = -0.3;  //+bxDim / 8 + 3 * initSpace0;
        builder.BuildBeam_FSI(
            my_mesh,         // the mesh where to put the created nodes and elements
            msection_cable,  // the ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
            15,              // the number of ChElementBeamANCF_3333 to create
            ChVector<>(loc_x, 0.0, initSpace0 * 15),  // the 'A' point in space (beginning of beam)
            ChVector<>(loc_x, 0.0, initSpace0),  // the 'B' point in space (end of beam) _1D__2D_elementsNodes_mesh,
            _1D_elementsNodes_mesh, NodeNeighborElement_mesh);

        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(builder.GetLastBeamNodes().back());
        auto pos_const = chrono_types::make_shared<ChLinkPointFrame>();
        pos_const->Initialize(node, ground);
        sysMBS.Add(pos_const);

        auto dir_const = chrono_types::make_shared<ChLinkDirFrame>();
        dir_const->Initialize(node, ground);
        dir_const->SetDirectionInAbsoluteCoords(node->D);
        sysMBS.Add(dir_const);

    } else {
        // Geometry of the plate
        double plate_lenght_x = 0.02;
        double plate_lenght_y = byDim;
        double plate_lenght_z = initSpace0 * 10;
        ChVector<> center_plate(bxDim / 8 + 3 * initSpace0, 0.0, plate_lenght_z / 2 + 1 * initSpace0);

        // Specification of the mesh
        int numDiv_x = 1;
        int numDiv_y = 2;
        int numDiv_z = 6;
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
                auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z),
                                                                     ChVector<>(dir_x, dir_y, dir_z));

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
        double E = 5e7;
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
                auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                                  std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

                // Set element dimensions
                element->SetDimensions(dy, dz);

                // Add a single layers with a fiber angle of 0 degrees.
                element->AddLayer(dx, 0 * CH_C_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(0.05);  // Structural damping for this element

                // Add element to mesh
                my_mesh->AddElement(element);
                ChVector<> center = 0.25 * (element->GetNodeA()->GetPos() + element->GetNodeB()->GetPos() +
                                            element->GetNodeC()->GetPos() + element->GetNodeD()->GetPos());
                std::cout << "Adding element" << num_elem << "  with center:  " << center.x() << " " << center.y()
                          << " " << center.z() << std::endl;
                num_elem++;
            }
        }
    }
    // Add the mesh to the system
    sysMBS.Add(my_mesh);

    // fluid representation of flexible bodies
    bool multilayer = true;
    bool removeMiddleLayer = true;
    bool add1DElem = flexible_elem_1D;
    bool add2DElem = !flexible_elem_1D;
    sysFSI.AddFEAmeshBCE(my_mesh, NodeNeighborElement_mesh, _1D_elementsNodes_mesh, _2D_elementsNodes_mesh, add1DElem,
                         add2DElem, multilayer, removeMiddleLayer, 0, 0);

    if (flexible_elem_1D)
        sysFSI.SetCableElementsNodes(_1D_elementsNodes_mesh);
    else
        sysFSI.SetShellElementsNodes(_2D_elementsNodes_mesh);

    sysFSI.SetFsiMesh(my_mesh);
    fea::ChMeshExporter::writeMesh(my_mesh, MESH_CONNECTIVITY);
}
