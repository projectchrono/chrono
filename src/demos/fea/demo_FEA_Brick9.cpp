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
// Authors: Bryan Peterson, Antonio Recuero
// =============================================================================
// Demo for 9-node, large deformation brick element
// The user can run three demos in the main function. 1) Axial dynamics excites
// a beam made up of brick elements axially; 2) BendingQuasiStatic applies a qua-
// sistatic load at a corner of a plate and is used for convergence verification;
// 3) Swinging shell is used for verification of dynamics, rigid body and large
// deformation problems.
// =============================================================================

#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChElementBrick_9.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;

void AxialDynamics();
void BendingQuasiStatic();
void SwingingShell();
int main(int argc, char* argv[]) {
    AxialDynamics();
    // BendingQuasiStatic();
    // SwingingShell();
    return 0;
}
// Axial Dynamic
void AxialDynamics() {
    FILE* outputfile;
    ChSystem my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element", core::dimension2d<u32>(800, 600),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------------------\n";
    GetLog() << "     9-Node, Large Deformation Brick Element with implicit integration \n";
    GetLog() << "-----------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.05;
    double plate_lenght_z = 0.05;

    // Specification of the mesh
    int numDiv_x = 20;
    int numDiv_y = 1;
    int numDiv_z = 1;

    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = 2 * XYNumNodes + TotalNumElements;

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    bool Hencky = true;
    bool Plasticity = true;
    bool DruckerPrager = true;
    double timestep = 1e-4;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;
            // Create the node
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0) {
                node->SetFixed(true);
            }
            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    double force = 0.0;
    // Get a handle to the tip node.
    auto nodetip1 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    auto nodetip2 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1 - numDiv_x - 1));
    auto nodetip3 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1));
    auto nodetip4 = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(XYNumNodes - 1 - numDiv_x - 1));
    nodetip1->SetForce(ChVector<>(0.0, 0.0, 0.0));
    nodetip2->SetForce(ChVector<>(0.0, 0.0, 0.0));
    nodetip3->SetForce(ChVector<>(0.0, 0.0, 0.0));
    nodetip4->SetForce(ChVector<>(0.0, 0.0, 0.0));
    // Create an orthotropic material.
    double rho = 7850.0;
    ChVector<> E(1.0e7, 1.0e7, 1.0e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(3.8461538e6, 3.8461538e6, 3.8461538e6);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x);
    // material->Set_G(G.x);
    material->Set_v(nu.x);
    ChMatrixNM<double, 9, 8> CCPInitial;
    for (int k = 0; k < 8; k++) {
        CCPInitial(0, k) = 1;
        CCPInitial(4, k) = 1;
        CCPInitial(8, k) = 1;
    }
    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementBrick_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.0);    // Structural damping for this element
        element->SetGravityOn(false);  // turn internal gravitational force calculation off
                                       // element->SetStrainFormulation(Hencky);
        element->SetHenckyStrain(Hencky);
        if (Hencky) {
            element->SetPlasticity(Plasticity);
            if (Plasticity) {
                element->SetYieldStress(1e5);
                element->SetHardeningSlope(5e5);
                element->SetCCPInitial(CCPInitial);
                element->SetDruckerPrager(DruckerPrager);
                if (DruckerPrager) {
                    element->SetFriction1(10.0);
                    element->SetFriction2(10.0);
                    element->SetDPType(3);
                }
            }
        }

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    my_system.Add(my_mesh);

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, 0.0));
    my_mesh->SetAutomaticGravity(false);

    // Mark completion of system construction
    my_system.SetupInitial();

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Use the MKL Solver
    ChSolverMKL* mkl_solver_stab = new ChSolverMKL;
    ChSolverMKL* mkl_solver_speed = new ChSolverMKL;
    my_system.ChangeSolverStab(mkl_solver_stab);
    my_system.ChangeSolverSpeed(mkl_solver_speed);
    mkl_solver_stab->SetSparsityPatternLock(true);
    mkl_solver_speed->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetIntegrationType(ChSystem::INT_HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(0.0);
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(1e-5, 1e-2);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(false);
    mystepper->SetScaling(true);
    application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x);
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y);
    fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z);
    fprintf(outputfile, "\n  ");

    double ChTime = 0.0;
    double start = std::clock();
    int Iter = 0;
    while (application.GetDevice()->run() && (my_system.GetChTime() <= 1.0)) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();

        force = 300 * std::sin(my_system.GetChTime() * CH_C_PI) / 4;
        nodetip1->SetForce(ChVector<>(force, 0.0, 0.0));
        nodetip2->SetForce(ChVector<>(force, 0.0, 0.0));
        nodetip3->SetForce(ChVector<>(force, 0.0, 0.0));
        nodetip4->SetForce(ChVector<>(force, 0.0, 0.0));

        application.EndScene();
        Iter += mystepper->GetNumIterations();
        GetLog() << "t = " << my_system.GetChTime() << "\n";
        GetLog() << "Last it: " << mystepper->GetNumIterations() << "\n\n";
        if (!application.GetPaused()) {
            fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().x);
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().y);
            fprintf(outputfile, "%15.7e  ", nodetip1->GetPos().z);
            fprintf(outputfile, "\n  ");
        }
    }
    double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    GetLog() << "Simulation Time: " << duration << "\n";
    GetLog() << Iter << "\n";
}

// QuasiStatic
void BendingQuasiStatic() {
    FILE* outputfile;
    ChSystem my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element: Bending Problem",
                         core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "  9-Node, Large Deformation Brick Element: Bending Problem \n";
    GetLog() << "-----------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();
    int numFlexBody = 1;
    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;
    // Specification of the mesh
    int numDiv_x = 8;
    int numDiv_y = 8;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = 2 * XYNumNodes + TotalNumElements;
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    double timestep = 5e-3;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
            node->SetMass(0);

            // Fix all nodes along the axis X=0
            if (i % (numDiv_x + 1) == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));

    // All layers for all elements share the same material.
    double rho = 500;
    ChVector<> E(2.1e8, 2.1e8, 2.1e8);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(8.0769231e7, 8.0769231e7, 8.0769231e7);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x);
    material->Set_G(G.x);
    material->Set_v(nu.x);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementBrick_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.25);   // Structural damping for this element
        element->SetGravityOn(false);  // Turn internal gravitational force calculation off

        element->SetHenckyStrain(true);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    my_system.Add(my_mesh);

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, 0.0));

    // Mark completion of system construction
    my_system.SetupInitial();

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    // my_system.SetSolverType(ChSystem::SOLVER_MINRES);  // <- NEEDED because other solvers can't
    //                                                   // handle stiffness matrices
    // ChSolverMINRES* msolver = (ChSolverMINRES*)my_system.GetSolverSpeed();
    // msolver->SetDiagonalPreconditioning(true);
    // my_system.SetMaxItersSolverSpeed(100);
    // my_system.SetTolForce(1e-10);

    // Use the MKL Solver
    ChSolverMKL* mkl_solver_stab = new ChSolverMKL;
    ChSolverMKL* mkl_solver_speed = new ChSolverMKL;
    my_system.ChangeSolverStab(mkl_solver_stab);
    my_system.ChangeSolverSpeed(mkl_solver_speed);
    mkl_solver_stab->SetSparsityPatternLock(true);
    mkl_solver_speed->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetIntegrationType(ChSystem::INT_HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(2000);
    mystepper->SetAbsTolerances(5e-5, 1e-1);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(true);
    mystepper->SetScaling(true);
    application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x);
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y);
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z);
    fprintf(outputfile, "\n  ");

    double force = 0.0;
    application.SetPaused(true);
    while (application.GetDevice()->run() && (my_system.GetChTime() <= 2.0)) {
        application.BeginScene();
        application.DrawAll();
        if (!application.GetPaused()) {
            if (my_system.GetChTime() > 1.0) {
                force = -50;
            } else {
                force = -50 * my_system.GetChTime();
            }

            nodetip->SetForce(ChVector<>(0.0, 0.0, force));

            GetLog() << my_system.GetChTime() << " " << nodetip->GetPos().x << " " << nodetip->GetPos().y << " "
                     << nodetip->GetPos().z << "\n";
        }
        application.DoStep();
        GetLog() << "Force: " << force << "\n";
        fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x);
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y);
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z);
        fprintf(outputfile, "\n  ");
        application.EndScene();
    }
}

// Swinging (Bricked) Shell
void SwingingShell() {
    FILE* outputfile;
    ChSystem my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"9-Node, Large Deformation Brick Element: Swinging (Bricked) Shell",
                         core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "--------------------------------------------------------------------\n";
    GetLog() << "--------------------------------------------------------------------\n";
    GetLog() << " 9-Node, Large Deformation Brick Element: Swinging (Bricked) Shell  \n";
    GetLog() << "--------------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.01;

    // Specification of the mesh
    int numDiv_x = 16;
    int numDiv_y = 16;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;

    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int XYNumNodes = (numDiv_x + 1) * (numDiv_y + 1);
    int TotalNumNodes = 2 * XYNumNodes + TotalNumElements;

    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    double timestep = 2e-3;

    // Create and add the nodes
    for (int j = 0; j <= numDiv_z; j++) {
        for (int i = 0; i < XYNumNodes; i++) {
            // Node location
            double loc_x = (i % (numDiv_x + 1)) * dx;
            double loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
            double loc_z = j * dz;

            // Create the node
            auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(loc_x, loc_y, loc_z));
            node->SetMass(0);
            // Fix all nodes along the axis X=0
            if (i == 0 && j == 0)
                node->SetFixed(true);

            // Add node to mesh
            my_mesh->AddNode(node);
        }
    }

    for (int i = 0; i < TotalNumElements; i++) {
        auto node = std::make_shared<ChNodeFEAcurv>(ChVector<>(0.0, 0.0, 0.0), ChVector<>(0.0, 0.0, 0.0),
                                                    ChVector<>(0.0, 0.0, 0.0));
        node->SetMass(0);
        my_mesh->AddNode(node);
    }

    double force = 0.0;
    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(2 * XYNumNodes - 1));
    nodetip->SetForce(ChVector<>(0.0, 0.0, -0.0));

    // All layers for all elements share the same material.
    double rho = 1000;
    ChVector<> E(2.1e7, 2.1e7, 2.1e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto material = std::make_shared<ChContinuumElastic>();
    material->Set_RayleighDampingK(0.0);
    material->Set_RayleighDampingM(0.0);
    material->Set_density(rho);
    material->Set_E(E.x);
    material->Set_G(G.x);
    material->Set_v(nu.x);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        int node1 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        int node2 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x;
        int node3 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        int node4 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + XYNumNodes;
        int node5 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + XYNumNodes;
        int node6 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x + XYNumNodes;
        int node7 = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x + XYNumNodes;
        int node8 = 2 * XYNumNodes + i;

        // Create the element and set its nodes.
        auto element = std::make_shared<ChElementBrick_9>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(node7)),
                          std::dynamic_pointer_cast<ChNodeFEAcurv>(my_mesh->GetNode(node8)));

        // Set element dimensions
        element->SetDimensions(ChVector<>(dx, dy, dz));

        // Add a single layers with a fiber angle of 0 degrees.
        element->SetMaterial(material);

        // Set other element properties
        element->SetAlphaDamp(0.005);  // Structural damping for this element
        element->SetGravityOn(true);   // turn internal gravitational force calculation off

        element->SetHenckyStrain(true);
        element->SetPlasticity(false);

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    my_system.Add(my_mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    my_system.Set_G_acc(ChVector<>(0.0, 0.0, -9.81));
    my_mesh->SetAutomaticGravity(false);

    // Mark completion of system construction
    my_system.SetupInitial();

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    // my_system.SetSolverType(ChSystem::SOLVER_MINRES);  // <- NEEDED because other solvers can't
    //                                                   // handle stiffness matrices
    // ChSolverMINRES* msolver = (ChSolverMINRES*)my_system.GetSolverSpeed();
    // msolver->SetDiagonalPreconditioning(true);
    // my_system.SetMaxItersSolverSpeed(100);
    // my_system.SetTolForce(1e-10);

    // Use the MKL Solver
    ChSolverMKL* mkl_solver_stab = new ChSolverMKL;
    ChSolverMKL* mkl_solver_speed = new ChSolverMKL;
    my_system.ChangeSolverStab(mkl_solver_stab);
    my_system.ChangeSolverSpeed(mkl_solver_speed);
    mkl_solver_stab->SetSparsityPatternLock(true);
    mkl_solver_speed->SetSparsityPatternLock(true);
    my_system.Update();

    // Set the time integrator parameters
    my_system.SetIntegrationType(ChSystem::INT_HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(20);
    mystepper->SetAbsTolerances(1e-6, 1e-1);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetVerbose(true);
    mystepper->SetScaling(true);
    application.SetTimestep(timestep);

    my_system.Setup();
    my_system.Update();

    outputfile = fopen("SolidBenchmark.txt", "w");
    fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x);
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y);
    fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z);
    fprintf(outputfile, "\n  ");

    double ChTime = 0.0;
    while (application.GetDevice()->run() && (my_system.GetChTime() < 2.01)) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        fprintf(outputfile, "%15.7e  ", my_system.GetChTime());
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().x);
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().y);
        fprintf(outputfile, "%15.7e  ", nodetip->GetPos().z);
        fprintf(outputfile, "\n  ");
        application.EndScene();
        if (!application.GetPaused()) {
            GetLog() << my_system.GetChTime() << " " << nodetip->GetPos().x << " " << nodetip->GetPos().y << " "
                     << nodetip->GetPos().z << "\n";
        }
    }
}
