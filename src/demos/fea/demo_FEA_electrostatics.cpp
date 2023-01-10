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
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA electrostatics
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChContinuumElectrostatics.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_mesh->SetAutomaticGravity(false);

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElectrostatics>();
    mmaterial->SetPermittivity(1);
    // mmaterial->SetRelativePermettivity(1000.01);

    //
    // Add some TETAHEDRONS:
    //

    // Load an Abaqus .INP tetrahedron mesh file from disk, defining a complicate tetrahedron mesh.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from SolidWorks simulation.

    std::map<std::string, std::vector<std::shared_ptr<ChNodeFEAbase> > > node_sets;

    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh, GetChronoDataFile("fea/electrostatics.INP").c_str(), mmaterial,
                                         node_sets);
    } catch (const ChException &myerr) {
        GetLog() << myerr.what();
        return 1;
    }

    //
    // Set some BOUNDARY CONDITIONS on nodes:
    //

    // Impose potential on all nodes of 1st nodeset (see *NSET section in .imp file)
    auto nboundary = "IMPV1";
    for (unsigned int inode = 0; inode < node_sets.at(nboundary).size(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(node_sets[nboundary][inode])) {
            mnode->SetFixed(true);
            mnode->SetP(0);  // field: potential [V]
        }
    }
    // Impose potential on all nodes of 2nd nodeset (see *NSET section in .imp file)
    nboundary = "IMPV2";
    for (unsigned int inode = 0; inode < node_sets.at(nboundary).size(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(node_sets[nboundary][inode])) {
            mnode->SetFixed(true);
            mnode->SetP(21);  // field: potential [V]
        }
    }

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).

    // This will paint the colored mesh with temperature scale (NODE_P is the scalar field of the Poisson problem)

    auto mvisualizemesh = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_P);  // plot V, potential field
    mvisualizemesh->SetColorscaleMinMax(-0.1, 24);
    my_mesh->AddVisualShapeFEA(mvisualizemesh);

    // This will paint the wireframe
    auto mvisualizemeshB = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizemeshB->SetColorscaleMinMax(-0.1, 24);
    mvisualizemeshB->SetWireframe(true);
    my_mesh->AddVisualShapeFEA(mvisualizemeshB);

    // This will paint the E vector field as line vectors
    auto mvisualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_VECT_DP);
    mvisualizemeshC->SetSymbolsScale(0.00002);
    mvisualizemeshC->SetDefaultSymbolsColor(ChColor(1.0f, 1.0f, 0.4f));
    mvisualizemeshC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizemeshC);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("FEM electrostatics");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddLight(ChVector<>(20, 20, 20), 90, ChColor(0.5f, 0.5f, 0.5f));
    vis->AddLight(ChVector<>(-20, 20, -20), 90, ChColor(0.7f, 0.8f, 0.8f));
    vis->AddCamera(ChVector<>(0., 0.2, -0.3));

    // SIMULATION LOOP

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(300);
    solver->SetTolerance(1e-20);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    sys.SetSolverForceTolerance(1e-20);

    // In electrostatics, you have only a single linear (non transient) solution
    sys.DoStaticLinear();

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
    }

    // Print some node potentials V..
    for (unsigned int inode = 0; inode < my_mesh->GetNnodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(inode))) {
            if (mnode->GetP() < 6.2) {
                // GetLog() << "Node at y=" << mnode->GetPos().y << " has V=" << mnode->GetP() << "\n";
            }
        }
    }

    return 0;
}
