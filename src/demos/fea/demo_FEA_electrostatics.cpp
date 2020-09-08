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
#include "chrono/fea/ChElementHexa_20.h"
#include "chrono/fea/ChElementHexa_8.h"
#include "chrono/fea/ChElementTetra_10.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"FEM electrostatics", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(core::vector3df(20, 20, 20), core::vector3df(-20, 20, -20), 90, 90,
                                 irr::video::SColorf(0.5, 0.5, 0.5));
    application.AddTypicalCamera(core::vector3df(0.f, 0.2f, -0.3f), core::vector3df(0.0f, 0.0f, 0.0f));

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
    } catch (ChException myerr) {
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
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    // This will paint the colored mesh with temperature scale (E_PLOT_NODE_P is the scalar field of the Poisson
    // problem)

    auto mvisualizemesh = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_P);  // plot V, potential field
    mvisualizemesh->SetColorscaleMinMax(-0.1, 24);
    my_mesh->AddAsset(mvisualizemesh);

    // This will paint the wireframe
    auto mvisualizemeshB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshB->SetColorscaleMinMax(-0.1, 24);
    mvisualizemeshB->SetWireframe(true);
    my_mesh->AddAsset(mvisualizemeshB);

    // This will paint the E vector field as line vectors
    auto mvisualizemeshC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_VECT_DP);
    mvisualizemeshC->SetSymbolsScale(0.00002);
    mvisualizemeshC->SetDefaultSymbolsColor(ChColor(1.0f, 1.0f, 0.4f));
    mvisualizemeshC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshC);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // SIMULATION LOOP

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(300);
    solver->SetTolerance(1e-20);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    my_system.SetSolverForceTolerance(1e-20);


    // Note: in electrostatics, here you can have only a single linear (non transient) solution
    // so at this point you must do:

    my_system.DoStaticLinear();

    // Also, in the following while() loop, remove  application.DoStep();
    // so the loop is used just to keep the 3D view alive, to spin & look at the solution.

    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.EndScene();
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
