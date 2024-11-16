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
// Authors: Mike Taylor
// =============================================================================
//
// Demo comparing two different ways to setup a cantilever beam with the
// ANCF 3833 shell element
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChElementShellANCF_3833.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChDirectSolverLS.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    double time_step = 1e-3;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    std::cout << "-----------------------------------------------------------------\n";
    std::cout << " Higher order ANCF Shell Element demo with different constraints \n";
    std::cout << "-----------------------------------------------------------------\n";

    // Mesh properties
    double length = 1.0;       // m
    double width = 0.1;        // m
    double thickness = 0.001;  // m
    double rho = 7810;         // kg/m^3
    double E = 2.1e11;         // Pa
    double nu = 0.3;           // Poisson's Ratio
    int num_elements = 4;      // Number of elements along each cantilever beam

    double dx = length / (num_elements);

    auto material = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu);

    // Create mesh container
    auto mesh = chrono_types::make_shared<ChMesh>();
    sys.Add(mesh);

    // Setup shell normals to initially align with the global z direction with no curvature
    ChVector3d dir1(0, 0, 1);
    ChVector3d Curv1(0, 0, 0);

    //   y
    //   ^
    //   |
    //   D---G---C
    //   |   |   |
    //   H---+---F
    //   |   |   |
    //   A---E---B----> x

    std::shared_ptr<ChNodeFEAxyzDD> nodeA;
    std::shared_ptr<ChNodeFEAxyzDD> nodeB;
    std::shared_ptr<ChNodeFEAxyzDD> nodeC;
    std::shared_ptr<ChNodeFEAxyzDD> nodeD;
    std::shared_ptr<ChNodeFEAxyzDD> nodeE;
    std::shared_ptr<ChNodeFEAxyzDD> nodeF;
    std::shared_ptr<ChNodeFEAxyzDD> nodeG;
    std::shared_ptr<ChNodeFEAxyzDD> nodeH;

    // -------------------------------------
    // Create the first beam, fixing its nodal coordinates on one end
    // -------------------------------------

    // Create the first nodes and fix them completely to ground (Cantilever constraint)
    nodeA = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(0, 0, 0.0), dir1, Curv1);
    nodeA->SetFixed(true);
    mesh->AddNode(nodeA);

    nodeD = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(0, width, 0), dir1, Curv1);
    nodeD->SetFixed(true);
    mesh->AddNode(nodeD);

    nodeH = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(0, 0.5 * width, 0), dir1, Curv1);
    nodeH->SetFixed(true);
    mesh->AddNode(nodeH);

    // Generate the rest of the nodes as well as all of the elements
    for (int i = 1; i <= num_elements; i++) {
        nodeB = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx, 0, 0), dir1, Curv1);
        mesh->AddNode(nodeB);
        nodeC = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx, width, 0), dir1, Curv1);
        mesh->AddNode(nodeC);
        nodeE = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx - 0.5 * dx, 0, 0.0), dir1, Curv1);
        mesh->AddNode(nodeE);
        nodeF = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx, 0.5 * width, 0), dir1, Curv1);
        mesh->AddNode(nodeF);
        nodeG = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx - 0.5 * dx, width, 0), dir1, Curv1);
        mesh->AddNode(nodeG);

        auto element = chrono_types::make_shared<ChElementShellANCF_3833>();
        element->SetNodes(nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH);
        element->SetDimensions(dx, width);
        element->SetAlphaDamp(0.001);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(thickness, 0 * CH_DEG_TO_RAD, material);

        mesh->AddElement(element);

        nodeA = nodeB;
        nodeD = nodeC;
        nodeH = nodeF;
    }
    auto nodetipB_beam1 = nodeB;
    auto nodetipC_beam1 = nodeC;
    auto nodetipF_beam1 = nodeF;

    // Apply a step load at the end of the beam that generates a twist
    nodetipB_beam1->SetForce(ChVector3d(0, 0, -3));
    nodetipC_beam1->SetForce(ChVector3d(0, 0, -2));
    nodetipF_beam1->SetForce(ChVector3d(0, 0, -1));

    // -------------------------------------
    // Create the second beam, fixing its nodal coordinates with constraints
    // Note that these constraints will create different boundary conditions than when completely fixing the nodes
    // -------------------------------------

    // Lateral offset for the second cantilever beam
    double offset = 2.0 * width;

    // Create the ground body to connect the cantilever beam to
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    // Create the first nodes
    nodeA = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(0, offset, 0.0), dir1, Curv1);
    mesh->AddNode(nodeA);

    nodeD = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(0, width + offset, 0), dir1, Curv1);
    mesh->AddNode(nodeD);

    nodeH = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(0, 0.5 * width + offset, 0), dir1, Curv1);
    mesh->AddNode(nodeH);

    // Fix the position of the starting nodes to the ground body
    auto constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
    constraintxyz->Initialize(nodeA, ground);
    sys.Add(constraintxyz);

    constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
    constraintxyz->Initialize(nodeD, ground);
    sys.Add(constraintxyz);

    constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
    constraintxyz->Initialize(nodeH, ground);
    sys.Add(constraintxyz);

    // Fix the position vector gradient coordinate set normal to the surface of the shell to remain parallel to the
    // original axis on the ground body (in this case the z axis)
    auto constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    constraintD->Initialize(nodeA, ground);
    sys.Add(constraintD);

    constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    constraintD->Initialize(nodeD, ground);
    sys.Add(constraintD);

    constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
    constraintD->Initialize(nodeH, ground);
    sys.Add(constraintD);

    // Constrain curvature at the base nodes (keep at initial value)
    nodeA->SetSlope2Fixed(true);
    nodeD->SetSlope2Fixed(true);
    nodeH->SetSlope2Fixed(true);

    // Store the starting nodes so that their coordinates can be inspected
    auto nodebaseA_beam2 = nodeA;
    auto nodebaseD_beam2 = nodeD;
    auto nodebaseH_beam2 = nodeH;

    // Generate the rest of the nodes as well as all of the elements
    for (int i = 1; i <= num_elements; i++) {
        nodeB = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx, 0 + offset, 0), dir1, Curv1);
        mesh->AddNode(nodeB);
        nodeC = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx, width + offset, 0), dir1, Curv1);
        mesh->AddNode(nodeC);
        nodeE = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx - 0.5 * dx, 0 + offset, 0.0), dir1, Curv1);
        mesh->AddNode(nodeE);
        nodeF = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx, 0.5 * width + offset, 0), dir1, Curv1);
        mesh->AddNode(nodeF);
        nodeG =
            chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector3d(i * dx - 0.5 * dx, width + offset, 0), dir1, Curv1);
        mesh->AddNode(nodeG);

        auto element = chrono_types::make_shared<ChElementShellANCF_3833>();
        element->SetNodes(nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH);
        element->SetDimensions(dx, width);
        element->SetAlphaDamp(0.001);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(thickness, 0 * CH_DEG_TO_RAD, material);

        mesh->AddElement(element);

        nodeA = nodeB;
        nodeD = nodeC;
        nodeH = nodeF;
    }
    auto nodetipB_beam2 = nodeB;
    auto nodetipC_beam2 = nodeC;
    auto nodetipF_beam2 = nodeF;

    // Apply a step load at the end of the beam that generates a twist
    nodetipB_beam2->SetForce(ChVector3d(0, 0, -3));
    nodetipC_beam2->SetForce(ChVector3d(0, 0, -2));
    nodetipF_beam2->SetForce(ChVector3d(0, 0, -1));

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto visualizemeshA = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    visualizemeshA->SetColorscaleMinMax(0.0, 5.50);
    visualizemeshA->SetShrinkElements(true, 0.85);
    visualizemeshA->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(visualizemeshA);

    auto visualizemeshB = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    visualizemeshB->SetWireframe(true);
    visualizemeshB->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(visualizemeshB);

    auto visualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    visualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshC->SetSymbolsThickness(0.004);
    mesh->AddVisualShapeFEA(visualizemeshC);

    auto visualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    visualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshD->SetSymbolsScale(1);
    visualizemeshD->SetColorscaleMinMax(-0.5, 5);
    visualizemeshD->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(visualizemeshD);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "ANCF Shells 3833",
                                         ChVector3d(0.5, -0.5, 0.5), ChVector3d(0.5, 0.25, 0.0));

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    solver->UseSparsityPatternLearner(false);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);
    sys.SetSolver(solver);

    // Set up integrator
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxIters(50);
    mystepper->SetAbsTolerances(1e-4, 1e2);
    mystepper->SetStepControl(false);
    mystepper->SetModifiedNewton(true);

    while (vis->Run()) {
        std::cout << "Time: " << sys.GetChTime() << "s. \n";

        std::cout << "  Beam1 Tip Node B vertical position:    " << nodetipB_beam1->GetPos().z() << "\n";
        std::cout << "  Beam2 Tip Node B vertical position:    " << nodetipB_beam2->GetPos().z() << "\n";
        std::cout << "  Delta vertical position (Beam1-Beam2): "
                  << nodetipB_beam1->GetPos().z() - nodetipB_beam2->GetPos().z() << "\n";
        std::cout << "  Beam2 Base Node A Coordinates (xyz):   " << nodebaseA_beam2->GetPos().x() << " "
                  << nodebaseA_beam2->GetPos().y() << " " << nodebaseA_beam2->GetPos().z() << "\n";
        std::cout << "  Beam2 Base Node A Coordinates (D):     " << nodebaseA_beam2->GetSlope1().x() << " "
                  << nodebaseA_beam2->GetSlope1().y() << " " << nodebaseA_beam2->GetSlope1().z() << "\n";
        std::cout << "  Beam2 Base Node A Coordinates (DD):    " << nodebaseA_beam2->GetSlope2().x() << " "
                  << nodebaseA_beam2->GetSlope2().y() << " " << nodebaseA_beam2->GetSlope2().z() << "\n";

        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(time_step);
    }

    return 0;
}
