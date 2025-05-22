
// ================================================================================
// CHRONO WORKBENCH - github.com/Concrete-Chrono-Development/chrono-preprocessor
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of the code that generated this file is governed by a BSD-style license that
// can be found in the LICENSE file at the top level of the distribution and at
// github.com/Concrete-Chrono-Development/chrono-preprocessor/blob/main/LICENSE
//
// ================================================================================
// Chrono Input File
// ================================================================================
//
//
// ================================================================================

#include <chrono/physics/ChSystemSMC.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/solver/ChIterativeSolverLS.h>
#include <chrono/solver/ChDirectSolverLS.h>
#include <chrono/timestepper/ChTimestepper.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_flow/ChElementSpringP.h"
#include "chrono_flow/ChContinuumHydroThermal2D.h"
#include "chrono_flow/ChContinuumPoisson2D.h"
#include "chrono_flow/ChMeshFileLoaderBeam.h"
#include "chrono_flow/ChNodeFEAxyzPP.h"

#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChContinuumThermal.h"
#include "chrono/fea/ChContinuumElectrostatics.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>
#include <chrono/fea/ChMeshExporter.h>

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/functions/ChFunctionPositionLine.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/physics/ChLinkMotionImposed.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <string>
#include <string_view>
#include <typeinfo>
#include <map>

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::flow;
using namespace irr;

void WriteMesh(std::shared_ptr<ChMesh> mesh, const std::string& mesh_filename) {
    std::ofstream out_stream;
    out_stream.open(mesh_filename, std::ios::out);
    out_stream.precision(7);
    out_stream << std::scientific;

    std::vector<std::vector<int>> BeamElemNodes;

    std::vector<std::shared_ptr<ChNodeFEAbase>> myvector;
    myvector.resize(mesh->GetNumNodes());

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(i));
    }

    int numBeams = 0;

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementSpringP>(mesh->GetElement(iele)))
            numBeams++;
    }
    out_stream << "\nCELLS " << mesh->GetNumElements() << " " << (unsigned int)(numBeams * 3) << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        std::vector<int> mynodes;

        if (auto elementBm = std::dynamic_pointer_cast<ChElementSpringP>(mesh->GetElement(iele))) {
            mynodes.resize(2);
            out_stream << "2 ";
            int nodeOrder[] = {0, 1};
            mynodes[0] = elementBm->GetNode(nodeOrder[0])->GetIndex();
            mynodes[1] = elementBm->GetNode(nodeOrder[1])->GetIndex();
            BeamElemNodes.push_back(mynodes);
            for (int myNodeN = 0; myNodeN < mynodes.size(); myNodeN++) {
                auto nodeA = (elementBm->GetNode(nodeOrder[myNodeN]));
                std::vector<std::shared_ptr<ChNodeFEAbase>>::iterator it;
                it = find(myvector.begin(), myvector.end(), nodeA);
                if (it == myvector.end()) {
                    // name not in vector
                } else {
                    auto index = std::distance(myvector.begin(), it);
                    out_stream << (unsigned int)index << " ";
                }
            }
            out_stream << "\n";
        }
    }

    out_stream << "\nCELL_TYPES " << mesh->GetNumElements() << "\n";

    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementSpringP>(mesh->GetElement(iele)))
            out_stream << "3\n";
    }

    out_stream.close();
}

void WriteFrame(std::shared_ptr<ChMesh> mesh, const std::string& mesh_filename, const std::string& vtk_filename) {
    std::ofstream out_stream;
    out_stream.open(vtk_filename, std::ios::trunc);

    out_stream << "# vtk DataFile Version 2.0" << std::endl;
    out_stream << "Unstructured Grid Example" << std::endl;
    out_stream << "ASCII" << std::endl;
    out_stream << "DATASET UNSTRUCTURED_GRID" << std::endl;

    out_stream << "POINTS " << mesh->GetNumNodes() << " double\n";

    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzPP>(mesh->GetNode(i));
        out_stream << node->GetPos().x() << " " << node->GetPos().y() << " " << node->GetPos().z() << "\n";
    }

    std::ifstream in_stream(mesh_filename);
    out_stream << in_stream.rdbuf();

    int numCell = 0;
    for (unsigned int iele = 0; iele < mesh->GetNumElements(); iele++) {
        if (std::dynamic_pointer_cast<ChElementSpringP>(mesh->GetElement(iele)))
            numCell++;
    }

    // out_stream << "\nCELL_DATA " << numCell << "\n";

    out_stream << "\nPOINT_DATA " << mesh->GetNumNodes() << "\n";
    out_stream << "SCALARS Temperature double\n";
    out_stream << "LOOKUP_TABLE default\n";

    ChVector3<> tempT;
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        tempT = std::dynamic_pointer_cast<ChNodeFEAxyzPP>(mesh->GetNode(i))->GetFieldVal();
        tempT += ChVector3<>(1e-40);
        out_stream << tempT.x() << "\n";
    }

    out_stream << "SCALARS Moisture double\n";
    out_stream << "LOOKUP_TABLE default\n";

    ChVector3<> tempH;
    for (unsigned int i = 0; i < mesh->GetNumNodes(); i++) {
        tempH = std::dynamic_pointer_cast<ChNodeFEAxyzPP>(mesh->GetNode(i))->GetFieldVal();
        tempH += ChVector3<>(1e-40);
        out_stream << tempH.y() << "\n";
    }

    out_stream.close();
}

int main(int argc, char** argv) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;
    // sys.SetNumThreads(8);

    // Create a fixed body (reference)
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    // Prepare output diectory and file name 
    std::filesystem::path path_to_this_file(__FILE__);
    std::string current_dir = path_to_this_file.remove_filename().string();
    std::string out_dir = current_dir + "/out/";
    std::string history_filename = "hist.dat";
    std::string mesh_dir = current_dir;

    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a material, that must be assigned to each element and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumHydroThermal2D>();
    mmaterial->SetSpecificHeatCapacity(1);
    mmaterial->SetDiffusivityConstants(128, 2.5, 128);
    mmaterial->SetDensity(2400);

    // Create mesh
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Load an Abaqus .INP beam mesh file from disk, defining a complicate beam mesh.
    std::cout << "Parsing Abaqus file!" << std::endl;
    std::string MyString = mesh_dir + "prism_100mmx500mm-edgeEle.inp";
    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase>>> node_sets;
    try {
        ChMeshFileLoaderBeam::FromAbaqusFile(my_mesh, MyString.c_str(), mmaterial, node_sets);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 1;
    }

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    /// Select the nodes on the top surface of the mesh
    ChVector3d RightBC(0.0, 0.0, 0.0);
    std::vector<std::shared_ptr<ChNodeFEAxyzPP>> top_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzPP>(my_mesh->GetNode(i));
        auto cz = node->GetPos().z();
        if (cz > 500.0) {
            top_nodes.push_back(node);
            node->SetFixed(true);
            node->SetFieldVal(RightBC);  // field: temperature [K]
        }
    }

    /// Select the nodes on the bottom surface of the mesh
    ChVector3d LeftBC(100.0, 1.0, 100.0);
    std::vector<std::shared_ptr<ChNodeFEAxyzPP>> bottom_nodes;
    for (unsigned int i = 0; i < my_mesh->GetNumNodes(); i++) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzPP>(my_mesh->GetNode(i));
        auto cz = node->GetPos().z();
        if (cz < 0.0) {
            bottom_nodes.push_back(node);
            node->SetFixed(true);
            node->SetFieldVal(LeftBC);  // field: temperature [K]
        }
    }

    // We do not want gravity effect on FEA elements in this demo
    my_mesh->SetAutomaticGravity(false);
    
    // Create the Irrlicht visualization system (doesn't work with Windows currently)
    // auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    // vis->SetWindowSize(1200, 600);
    // vis->SetWindowTitle("Truss FEA test: use ChElementSpring and ChElementBar");
    // vis->Initialize();
    // vis->AddLogo();
    // vis->AddSkyBox();
    // vis->AddLight(ChVector3d(20, 20, 20), 90, ChColor(0.5, 0.5, 0.5));
    // vis->AddCamera(ChVector3d(-1.0, -1.0, -1.0));
    // vis->AttachSystem(&sys);

    /// Create a Chrono solver and set solver settings
    // Use MINRES solver to handle stiffness matrices.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(150);
    solver->SetTolerance(1e-6);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(true);
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    std::ofstream histfile;
    histfile.open(out_dir + history_filename, std::ios::out);
    histfile << "time" << " " << "node" << " " << "temp" << " " << "humidity" << "\n";
    // print temperature at the nodes along x axis and y=0
    for (unsigned int inode = 0; inode < my_mesh->GetNumNodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzPP>(my_mesh->GetNode(inode))) {
            if (abs(mnode->GetPos().x() - 50.0) < 6.6 && abs(mnode->GetPos().y() - 50.0) < 6.6) {
                histfile << "0.0" << " " << mnode->GetPos().z() << " " << mnode->GetFieldVal().x() << " "
                         << mnode->GetFieldVal().y() << "\n";
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Analysis
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "DYNAMIC ANALYSIS STARTED! \n\n";

    double timestep = 6000.0;            // seconds
    double simtime = 10 * 24 * 60 * 60;  // seconds
    int stepnum = 0;
    int VTKint = 10;
    int HISTint = 5;

    // Write VTK file for nodal temperature
    std::string mesh_filename = out_dir + "TempMesh" + std::to_string(stepnum) + ".vtk";
    std::string vtk_filename = out_dir + "TempVTK" + std::to_string(stepnum) + ".vtk";
    WriteMesh(my_mesh, mesh_filename);
    WriteFrame(my_mesh, mesh_filename, vtk_filename);

    std::cout << "TIME STEPPING STARTED! \n\n";
    while (sys.GetChTime() < simtime) {
        sys.DoStepDynamics(timestep);
        stepnum++;

        double sim_time = sys.GetChTime();
        std::cout << "SIMULATION TIME =" << " " << sim_time << std::endl;

        // Write VTK file for nodal temperature
        if (stepnum % VTKint == 0) {
            std::string mesh_filename = out_dir + "TempMesh" + std::to_string(stepnum) + ".vtk";
            std::string vtk_filename = out_dir + "TempVTK" + std::to_string(stepnum) + ".vtk";
            WriteMesh(my_mesh, mesh_filename);
            WriteFrame(my_mesh, mesh_filename, vtk_filename);
        }

        // print temperature at the nodes along z axis and x=y=50.0
        if (stepnum % VTKint == 0) {
            for (unsigned int inode = 0; inode < my_mesh->GetNumNodes(); ++inode) {
                if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzPP>(my_mesh->GetNode(inode))) {
                    if (abs(mnode->GetPos().x() - 50.0) < 6.6 && abs(mnode->GetPos().y() - 50.0) < 6.6) {
                        histfile << sim_time << " " << mnode->GetPos().z() << " " << mnode->GetFieldVal().x() << " "
                                 << mnode->GetFieldVal().y() << "\n";
                    }
                }
            }
        }
    }

    return 0;
}
