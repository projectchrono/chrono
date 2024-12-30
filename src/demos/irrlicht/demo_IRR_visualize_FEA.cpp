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

#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementSpring.h"
#include "chrono/fea/ChElementBar.h"
#include "chrono/fea/ChElementTetraCorot_4.h"
#include "chrono/fea/ChElementTetraCorot_10.h"
#include "chrono/fea/ChElementHexaCorot_8.h"
#include "chrono/fea/ChElementHexaCorot_20.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshFileLoader.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/assets/ChVisualShapeFEA.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Create a material, that must be assigned to each element and set its parameters
    auto material = chrono_types::make_shared<ChContinuumElastic>();
    material->SetYoungModulus(0.01e9);  // rubber 0.01e9, steel 200e9
    material->SetPoissonRatio(0.3);
    material->SetRayleighDampingBeta(0.001);
    material->SetDensity(1000);

    // Add some TETAHEDRONS from .nmode and .ele input files
    try {
        ChMeshFileLoader::FromTetGenFile(mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                         GetChronoDataFile("fea/beam.ele").c_str(), material);
    } catch (std::exception myerr) {
        std::cerr << myerr.what() << std::endl;
        return 0;
    }

    // Apply a force to a node
    auto nodelast = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(mesh->GetNumNodes() - 1));
    nodelast->SetForce(ChVector3d(50, 0, 50));

    // Add some HEXAHEDRONS (isoparametric bricks)
    ChVector3d hexpos(0, 0, 0);
    double sx = 0.1;
    double sz = 0.1;
    for (int e = 0; e < 6; ++e) {
        double angle = e * (2 * CH_PI / 8.0);
        hexpos.z() = 0.3 * std::cos(angle);
        hexpos.x() = 0.3 * std::sin(angle);
        ChMatrix33<> hexrot(QuatFromAngleY(angle));

        std::shared_ptr<ChNodeFEAxyz> hnode1_lower;
        std::shared_ptr<ChNodeFEAxyz> hnode2_lower;
        std::shared_ptr<ChNodeFEAxyz> hnode3_lower;
        std::shared_ptr<ChNodeFEAxyz> hnode4_lower;

        for (int ilayer = 0; ilayer < 6; ++ilayer) {
            double hy = ilayer * sz;
            auto hnode1 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector3d(0, hy, 0));
            auto hnode2 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector3d(0, hy, sz));
            auto hnode3 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector3d(sx, hy, sz));
            auto hnode4 = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector3d(sx, hy, 0));
            mesh->AddNode(hnode1);
            mesh->AddNode(hnode2);
            mesh->AddNode(hnode3);
            mesh->AddNode(hnode4);

            if (ilayer > 0) {
                auto helement1 = chrono_types::make_shared<ChElementHexaCorot_8>();
                helement1->SetNodes(hnode1_lower, hnode2_lower, hnode3_lower, hnode4_lower, hnode1, hnode2, hnode3,
                                    hnode4);
                helement1->SetMaterial(material);

                mesh->AddElement(helement1);
            }

            hnode1_lower = hnode1;
            hnode2_lower = hnode2;
            hnode3_lower = hnode3;
            hnode4_lower = hnode4;
        }

        // For example, set an initial displacement to a node:
        hnode4_lower->SetPos(hnode4_lower->GetX0() + hexrot * ChVector3d(0.1, 0.1, 0));

        // Apply a force to a node
        hnode4_lower->SetForce(hexrot * ChVector3d(500, 0, 0));
    }

    // Add the mesh to the system
    sys.Add(mesh);

    // Create a truss
    auto truss = chrono_types::make_shared<ChBody>();
    truss->SetFixed(true);
    sys.Add(truss);

    // Create constraints between nodes and truss
    // (for example, fix to ground all nodes which are near y=0)
    for (unsigned int inode = 0; inode < mesh->GetNumNodes(); ++inode) {
        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(inode))) {
            if (node->GetPos().y() < 0.01) {
                auto constraint = chrono_types::make_shared<ChLinkNodeFrame>();
                constraint->Initialize(node, truss);
                sys.Add(constraint);

                // Attach small cube to show the constraint
                auto box = chrono_types::make_shared<ChVisualShapeBox>(0.01, 0.01, 0.01);
                constraint->AddVisualShape(box);

                // Easier method: set the node as fixed (but this does not provide information about reaction forces)
                ////node->SetFixed(true);
            }
        }
    }

    // Visualization of the FEM mesh.
    {
        // Mesh visualization - speed
        auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
        vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
        vis_mesh->SetColorscaleMinMax(0.0, 5.50);
        vis_mesh->SetShrinkElements(true, 0.85);
        vis_mesh->SetSmoothFaces(true);
        mesh->AddVisualShapeFEA(vis_mesh);
    }

    {
        // Mesh visualization - reference configuration (wireframe)
        auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
        vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        vis_mesh->SetWireframe(true);
        vis_mesh->SetDrawInUndeformedReference(true);
        mesh->AddVisualShapeFEA(vis_mesh);
    }

    {
        // Node visualization - positions
        auto vis_nodes = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
        vis_nodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        vis_nodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        vis_nodes->SetSymbolsThickness(0.006);
        mesh->AddVisualShapeFEA(vis_nodes);
    }

    // Create the visualization system
    ChVisualSystemIrrlicht vis;
    vis.SetWindowSize(800, 600);
    vis.SetWindowTitle("Irrlicht FEM visualization");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddTypicalLights();
    vis.AddCamera(ChVector3d(0.0, 0.6, -1.0));
    vis.AttachSystem(&sys);

    // Solver settings
    sys.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetVerbose(false);

    // Simulation loop
    while (vis.Run()) {
        vis.BeginScene();
        vis.Render();
        vis.EndScene();
        sys.DoStepDynamics(0.001);
    }

    return 0;
}
