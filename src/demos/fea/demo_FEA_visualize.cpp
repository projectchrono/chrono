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

#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChElementTetra_10.h"
#include "chrono/fea/ChElementHexa_8.h"
#include "chrono/fea/ChElementHexa_20.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"

//#include "chrono_matlab/ChMatlabEngine.h"
//#include "chrono_matlab/ChSolverMatlab.h"

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
    ChIrrApp application(&my_system, L"Irrlicht FEM visualization", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, (f32)0.6, -1));

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element,
    // and set its parameters
    auto mmaterial = chrono_types::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.001);
    mmaterial->Set_density(1000);

    //
    // Add some TETAHEDRONS:
    //

    // Load a .node file and a .ele  file from disk, defining a complicate tetrahedron mesh.
    // This is much easier than creating all nodes and elements via C++ programming.
    // You can generate these files using the TetGen tool.
    try {
        ChMeshFileLoader::FromTetGenFile(my_mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                         GetChronoDataFile("fea/beam.ele").c_str(), mmaterial);
    } catch (ChException myerr) {
        GetLog() << myerr.what();
        return 0;
    }

    // Apply a force to a node
    auto mnodelast = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(my_mesh->GetNnodes() - 1));
    mnodelast->SetForce(ChVector<>(50, 0, 50));

    //
    // Add some HEXAHEDRONS (isoparametric bricks):
    //

    ChVector<> hexpos(0, 0, 0);
    double sx = 0.1;
    double sz = 0.1;
    for (int e = 0; e < 6; ++e) {
        double angle = e * (2 * CH_C_PI / 8.0);
        hexpos.z() = 0.3 * cos(angle);
        hexpos.x() = 0.3 * sin(angle);
        ChMatrix33<> hexrot(Q_from_AngAxis(angle, VECT_Y));

        std::shared_ptr<ChNodeFEAxyz> hnode1_lower;
        std::shared_ptr<ChNodeFEAxyz> hnode2_lower;
        std::shared_ptr<ChNodeFEAxyz> hnode3_lower;
        std::shared_ptr<ChNodeFEAxyz> hnode4_lower;

        for (int ilayer = 0; ilayer < 6; ++ilayer) {
            double hy = ilayer * sz;
            auto hnode1 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector<>(0, hy, 0));
            auto hnode2 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector<>(0, hy, sz));
            auto hnode3 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector<>(sx, hy, sz));
            auto hnode4 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector<>(sx, hy, 0));
            my_mesh->AddNode(hnode1);
            my_mesh->AddNode(hnode2);
            my_mesh->AddNode(hnode3);
            my_mesh->AddNode(hnode4);

            if (ilayer > 0) {
                auto helement1 = chrono_types::make_shared<ChElementHexa_8>();
                helement1->SetNodes(hnode1_lower, hnode2_lower, hnode3_lower, hnode4_lower, hnode1, hnode2, hnode3,
                                    hnode4);
                helement1->SetMaterial(mmaterial);

                my_mesh->AddElement(helement1);
            }

            hnode1_lower = hnode1;
            hnode2_lower = hnode2;
            hnode3_lower = hnode3;
            hnode4_lower = hnode4;
        }

        // For example, set an initial displacement to a node:
        hnode4_lower->SetPos(hnode4_lower->GetX0() + hexrot * ChVector<>(0.1, 0.1, 0));

        // Apply a force to a node
        hnode4_lower->SetForce(hexrot * ChVector<>(500, 0, 0));
    }

    //
    // Final touches..
    //

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Create also a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    my_system.Add(truss);

    // Create constraints between nodes and truss
    // (for example, fix to ground all nodes which are near y=0
    for (unsigned int inode = 0; inode < my_mesh->GetNnodes(); ++inode) {
        if (auto mnode = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(inode))) {
            if (mnode->GetPos().y() < 0.01) {
                auto constraint = chrono_types::make_shared<ChLinkPointFrame>();
                constraint->Initialize(mnode, truss);
                my_system.Add(constraint);

                // For example, attach small cube to show the constraint
                auto mboxfloor = chrono_types::make_shared<ChBoxShape>();
                mboxfloor->GetBoxGeometry().Size = ChVector<>(0.005);
                constraint->AddAsset(mboxfloor);

                // Otherwise there is an easier method: just set the node as fixed (but
                // in this way you do not get infos about reaction forces as with a constraint):
                //
                // mnode->SetFixed(true);
            }
        }
    }

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizemesh = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.006);
    my_mesh->AddAsset(mvisualizemeshC);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Simulation loop

    my_system.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);

    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        //	GetLog() << " t =" << my_system.GetChTime() << "  mnode3 pos.y()=" << mnode3->GetPos().y() << "  \n";

        application.EndScene();
    }

    return 0;
}
