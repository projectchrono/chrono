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
// FEA visualization using Irrlicht
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChElementTetra_10.h"
#include "chrono/fea/ChElementHexa_8.h"
#include "chrono/fea/ChElementHexa_20.h"
#include "chrono/fea/ChContinuumThermal.h"
#include "chrono/fea/ChContinuumElectrostatics.h"
#include "chrono/fea/ChNodeFEAxyzP.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLinkPointFrame.h"
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
    ChIrrApp application(&my_system, L"FEM thermal", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(core::vector3df(20, 20, 20), core::vector3df(-20, 20, -20), 90, 90,
                                 irr::video::SColorf(0.5, 0.5, 0.5));
    application.AddTypicalCamera(core::vector3df(0, (f32)0.7, -1), core::vector3df(0, (f32)0.4, 0));

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumThermal>();
    mmaterial->SetMassSpecificHeatCapacity(2);
    mmaterial->SetThermalConductivityK(200);

    //
    // Add some TETAHEDRONS:
    //

    // Load a .node file and a .ele  file from disk, defining a complicate tetrahedron mesh.
    // This is much easier than creating all nodes and elements via C++ programming.
    // You can generate these files using the TetGen tool.
    try {
        ChMeshFileLoader::FromTetGenFile(my_mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                         GetChronoDataFile("fea/beam.ele").c_str(), mmaterial);
    } catch (const ChException &myerr) {
        GetLog() << myerr.what();
        return 0;
    }

    for (unsigned int inode = 0; inode < my_mesh->GetNnodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(inode))) {
            mnode->SetPos(mnode->GetPos() * ChVector<>(3, 1, 3));
        }
    }

    //
    // Set some BOUNDARY CONDITIONS on nodes:
    //

    // Impose load on the 180th node
    auto mnode3 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(180));
    mnode3->SetF(20);  // thermal load: heat flux [W] into node

    // Impose field on two top nodes (remember the SetFixed(true); )
    auto mnode1 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(my_mesh->GetNnodes() - 1));
    mnode1->SetFixed(true);
    mnode1->SetP(0.5);  // field: temperature [K]
    auto mnode2 = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(my_mesh->GetNnodes() - 2));
    mnode2->SetFixed(true);
    mnode2->SetP(0.5);  // field: temperature [K]

    // Impose field on the base points:
    for (unsigned int inode = 0; inode < my_mesh->GetNnodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(inode))) {
            if (mnode->GetPos().y() < 0.01) {
                mnode->SetFixed(true);
                mnode->SetP(10);  // field: temperature [K]
            }
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
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_P);
    mvisualizemesh->SetColorscaleMinMax(-1, 12);
    mvisualizemesh->SetShrinkElements(false, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    // This will paint the wireframe
    auto mvisualizemeshB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshB->SetWireframe(true);
    my_mesh->AddAsset(mvisualizemeshB);

    // This will paint the heat flux as line vectors
    auto mvisualizemeshC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_VECT_DP);
    mvisualizemeshC->SetSymbolsScale(0.003);
    mvisualizemeshC->SetDefaultSymbolsColor(ChColor(0.1f, 0.2f, 0.2f));
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

    // Use MINRES solver to handle stiffness matrices.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(150);
    solver->SetTolerance(1e-6);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);

    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    // Note: if you are interested only in a single LINEAR STATIC solution
    // (not a transient thermal solution, but rather the steady-state solution),
    // at this point you can uncomment the following line:
    //
    //  my_system.DoStaticLinear();
    //
    // Also, in the following while() loop, remove  application.DoStep();
    // so you can spin the 3D view and look at the solution.

    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
        if (my_system.GetChTime() > 5)
            break;
    }

    // Print some node temperatures..
    for (unsigned int inode = 0; inode < my_mesh->GetNnodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyzP>(my_mesh->GetNode(inode))) {
            if (mnode->GetPos().x() < 0.01) {
                GetLog() << "Node at y=" << mnode->GetPos().y() << " has T=" << mnode->GetP() << "\n";
            }
        }
    }

    return 0;
}
