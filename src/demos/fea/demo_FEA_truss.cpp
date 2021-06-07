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
// FEA for thin shells of Kirchhoff-Love type, with BST triangle finite elements
//
// =============================================================================

#include <vector>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "FEA_TRUSS";

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create (if needed) output directory
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create two meshes (the second just for containing the 'dumb' massless nodes,
    // so that they can be plot with a different color)
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    auto my_mesh_dumb = chrono_types::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);
    my_system.Add(my_mesh_dumb);

    //
    // BENCHMARK n.1
    //
    // Add a truss made with ChElementSpring FEA elements:
    //

    std::vector<std::shared_ptr<ChNodeFEAxyz> > mass_nodes;
    std::vector<std::shared_ptr<ChNodeFEAxyz> > dumb_nodes;
    std::vector<std::shared_ptr<ChElementSpring> > springs;

    if (true)  // set as 'true' to execute this
    {
        // Create a fixed body (reference)
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetBodyFixed(true);
        my_system.Add(ground);

        int nnodes_x = 20;
        int nnodes_y = 10;
        double L_x = 1;
        double L_y = 0.3;
        double density = 30;

        // stiffness coefficients
        double k_X = 1.0e+1;
        double k_Y = 1.0e+1;
        double k_00 = 1.0e+1;
        double k_10 = 1.0e+1;
        double k_11 = 1.0e+1;
        double k_01 = 1.0e+1;

        // Create mass nodes
        for (int ix = 0; ix < nnodes_x; ++ix) {
            for (int iy = 0; iy < nnodes_y; ++iy) {
                double step_x = L_x / ((double)nnodes_x - 1.0);
                double step_y = L_y / ((double)nnodes_y - 1.0);

                ChVector<> P_mass(ix * step_x, iy * step_y, 0);

                auto mnode = chrono_types::make_shared<ChNodeFEAxyz>(P_mass);
                mnode->SetMass(step_x * step_y * density);
                mnode->SetPos_dt(ChVector<>(0.05, 0, 0));
                if (ix == 0) {
                    mnode->SetFixed(true);
                    // Create a constraint between a node and the truss
                    // auto constraintA = chrono_types::make_shared<ChLinkPointFrame>();
                    // constraintA->Initialize(mnode, ground);  // body to be connected to
                    // my_system.Add(constraintA);
                }
                if (ix == nnodes_x - 1) {
                    mnode->SetForce(ChVector<>(0, -0.00, 0));
                }
                my_mesh->AddNode(mnode);
                mass_nodes.push_back(mnode);  // for later easy reference
            }
        }

        // Create dumb mass-less nodes
        for (int ex = 0; ex < nnodes_x; ++ex) {
            for (int ey = 0; ey < nnodes_y; ++ey) {
                if (ex + 1 < nnodes_x && ey + 1 < nnodes_y) {
                    ChVector<> P_dumb = 1.0 / 4.0 * mass_nodes[ex * nnodes_y + ey]->GetPos() +
                                        1.0 / 4.0 * mass_nodes[(ex + 1) * nnodes_y + ey]->GetPos() +
                                        1.0 / 4.0 * mass_nodes[(ex + 1) * nnodes_y + (ey + 1)]->GetPos() +
                                        1.0 / 4.0 * mass_nodes[ex * nnodes_y + (ey + 1)]->GetPos();

                    auto mnode = chrono_types::make_shared<ChNodeFEAxyz>(P_dumb);
                    mnode->SetMass(0.0);
                    my_mesh_dumb->AddNode(mnode);
                    dumb_nodes.push_back(mnode);  // for later easy reference
                }
            }
        }

        // Create a lattice of springs
        for (int ex = 0; ex < nnodes_x; ++ex) {
            for (int ey = 0; ey < nnodes_y; ++ey) {
                if (ex + 1 < nnodes_x) {
                    auto mspring_X = chrono_types::make_shared<ChElementSpring>();
                    mspring_X->SetNodes(mass_nodes[ex * nnodes_y + ey], mass_nodes[(ex + 1) * nnodes_y + ey]);
                    mspring_X->SetSpringK(k_X);
                    my_mesh->AddElement(mspring_X);
                    springs.push_back(mspring_X);  // for later easy reference
                }
                if (ey + 1 < nnodes_y) {
                    auto mspring_Y = chrono_types::make_shared<ChElementSpring>();
                    mspring_Y->SetNodes(mass_nodes[ex * nnodes_y + ey], mass_nodes[ex * nnodes_y + (ey + 1)]);
                    mspring_Y->SetSpringK(k_Y);
                    my_mesh->AddElement(mspring_Y);
                    springs.push_back(mspring_Y);  // for later easy reference
                }
                if (ex + 1 < nnodes_x && ey + 1 < nnodes_y) {
                    auto mspring_D00 = chrono_types::make_shared<ChElementSpring>();
                    mspring_D00->SetNodes(mass_nodes[ex * nnodes_y + ey], dumb_nodes[ex * (nnodes_y - 1) + ey]);
                    mspring_D00->SetSpringK(k_00);
                    my_mesh->AddElement(mspring_D00);
                    springs.push_back(mspring_D00);  // for later easy reference

                    auto mspring_D01 = chrono_types::make_shared<ChElementSpring>();
                    mspring_D01->SetNodes(mass_nodes[ex * nnodes_y + ey + 1], dumb_nodes[ex * (nnodes_y - 1) + ey]);
                    mspring_D01->SetSpringK(k_01);
                    my_mesh->AddElement(mspring_D01);
                    springs.push_back(mspring_D01);  // for later easy reference

                    auto mspring_D10 = chrono_types::make_shared<ChElementSpring>();
                    mspring_D10->SetNodes(mass_nodes[(ex + 1) * nnodes_y + ey], dumb_nodes[ex * (nnodes_y - 1) + ey]);
                    mspring_D10->SetSpringK(k_10);
                    my_mesh->AddElement(mspring_D10);
                    springs.push_back(mspring_D10);  // for later easy reference

                    auto mspring_D11 = chrono_types::make_shared<ChElementSpring>();
                    mspring_D11->SetNodes(mass_nodes[(ex + 1) * nnodes_y + ey + 1],
                                          dumb_nodes[ex * (nnodes_y - 1) + ey]);
                    mspring_D11->SetSpringK(k_11);
                    my_mesh->AddElement(mspring_D11);
                    springs.push_back(mspring_D11);  // for later easy reference
                }
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

    auto mvisualizeA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeA->SetWireframe(true);
    my_mesh->AddAsset(mvisualizeA);

    auto mvisualizeB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizeB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizeB->SetSymbolsThickness(0.015);
    my_mesh->AddAsset(mvisualizeB);

    auto mvisualizeC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh_dumb.get()));
    mvisualizeC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizeC->SetDefaultSymbolsColor(ChColor(1, 1, 0));
    mvisualizeC->SetSymbolsThickness(0.01);
    my_mesh->AddAsset(mvisualizeC);

    //
    // VISUALIZATION
    //

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Truss FEA test: use ChElementSpring and ChElementBar",
                         core::dimension2d<u32>(1024, 768));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.f, 1.0f, -1.f));

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    // Change solver to MKL
    // auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    // mkl_solver->LockSparsityPattern(true);
    // my_system.SetSolver(mkl_solver);

    // Change solver to GMRES
    auto gmres_solver = chrono_types::make_shared<ChSolverGMRES>();
    gmres_solver->SetMaxIterations(300);
    my_system.SetSolver(gmres_solver);

    double timestep = 0.01;
    application.SetTimestep(timestep);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // draw spring elements as lines
        for (auto mspring : springs) {
            if (mspring->GetSpringK() > 0) {
                tools::drawSegment(application.GetVideoDriver(),
                                        std::dynamic_pointer_cast<ChNodeFEAxyz>(mspring->GetNodeN(0))->GetPos(),
                                        std::dynamic_pointer_cast<ChNodeFEAxyz>(mspring->GetNodeN(1))->GetPos(),
                                        irr::video::SColor(255, 255, 255, 255), true);
            }
            // cut springs if beyond a stress limit
            if (fabs(mspring->GetCurrentForce()) > 100) {
                mspring->SetSpringK(0);
                mspring->SetDamperR(0);
            }
        }

        // quick trick to force all nodes on xy plane without constraints
        for (auto mnode : mass_nodes) {
            auto offposition = mnode->GetPos();
            offposition.z() = 0;
            mnode->SetPos(offposition);
        }
        for (auto mnode : dumb_nodes) {
            auto offposition = mnode->GetPos();
            offposition.z() = 0;
            mnode->SetPos(offposition);
        }

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
