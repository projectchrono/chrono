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
// Authors: Antonio Recuero
// =============================================================================
//
// Demo on using 8-node ANCF shell elements. These demo reproduces the example
// 3.3 of the paper: 'Analysis of higher-order quadrilateral plate elements based
// on the absolute nodal coordinate formulation for three-dimensional elasticity'
// H.C.J. Ebel, M.K.Matikainen, V.V.T. Hurskainen, A.M.Mikkola, Advances in
// Mechanical Engineering, 2017
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChElementShellANCF_8.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    double time_step = 1e-3;

    ChSystemSMC my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, 0.0));
    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"ANCF Shells", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------------\n";
    GetLog() << "  Higher order ANCF Shell Element demo with implicit integration \n";
    GetLog() << "-----------------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    // Geometry of the plate
    double plate_lenght_x = 1.0;
    double plate_lenght_y = 2.0;
    double plate_lenght_z = 0.01;
    // Specification of the mesh
    int numDiv_x = 4;
    int numDiv_y = 1;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 2) + numDiv_x * (numDiv_y + 1);
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;

    // Nodes at the edge of the plated beam
    for (int i = 0; i < (2 * numDiv_x + 1) * (numDiv_y + 1); i++) {
        // Node location
        double loc_x = (i % ((numDiv_x)*2 + 1)) * dx / 2;
        double loc_y = (i / ((numDiv_x)*2 + 1)) % (numDiv_y + 1) * dy;
        double loc_z = 0.0;

        // Node direction
        double dir_x = 0;
        double dir_y = 0;
        double dir_z = 1;
        double curvz_x = 0.0;
        double curvz_y = 0.0;
        double curvz_z = 0.0;
        // Create the node
        auto node = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z),
                                                     ChVector<>(curvz_x, curvz_y, curvz_z));
        node->SetMass(0);

        // Fix all nodes along the axis X=0
        if (i % ((numDiv_x)*2 + 1) == 0)
            node->SetFixed(true);

        // Add node to mesh
        my_mesh->AddNode(node);
    }
    // Nodes at the center of plated beam
    for (int i = 0; i < (numDiv_x + 1) * (numDiv_y); i++) {
        // Node location
        double loc_x = (i % ((numDiv_x) + 1)) * dx;
        double loc_y = (i / ((numDiv_x) + 1)) % (numDiv_y + 1) * dy + dy / 2;
        double loc_z = 0.0;

        // Node direction
        double dir_x = 0;
        double dir_y = 0;
        double dir_z = 1;
        double curvz_x = 0.0;
        double curvz_y = 0.0;
        double curvz_z = 0.0;
        // Create the node
        auto node = chrono_types::make_shared<ChNodeFEAxyzDD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z),
                                                     ChVector<>(curvz_x, curvz_y, curvz_z));
        node->SetMass(0);
        // Fix all nodes along the axis X=0
        if (i % (numDiv_x + 1) == 0)
            node->SetFixed(true);

        // Add node to mesh
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip1 =
        std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode((2 * numDiv_x + 1) * (numDiv_y + 1) - 1));
    auto nodetip2 = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode((2 * numDiv_x + 1) * (numDiv_y)-1));
    auto nodetip3 = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode((2 * numDiv_x + 1) * (numDiv_y + 1) + ((TotalNumElements - 1) / numDiv_x) * (numDiv_x + 1) + ((TotalNumElements-1) % numDiv_x) + 1));


    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho = 500;
    ChVector<> E(2.1e11, 2.1e11, 2.1e11);
    ChVector<> nu(0.0, 0.0, 0.0);
    ChVector<> G(E.x() / (2 * (1 + nu.x())), E.x() / (2 * (1 + nu.x())), E.x() / (2 * (1 + nu.x())));
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / (numDiv_x)) * (2 * N_x - 1) + 2 * (i % numDiv_x);
        int node1 = (i / (numDiv_x)) * (2 * N_x - 1) + 2 * (i % numDiv_x) + 2;
        int node2 = (i / (numDiv_x)) * (2 * N_x - 1) + 2 * (i % numDiv_x) + 2 * N_x + 1;
        int node3 = (i / (numDiv_x)) * (2 * N_x - 1) + 2 * (i % numDiv_x) + 2 * N_x - 1;

        int node4 = (i / (numDiv_x)) * (2 * N_x - 1) + 2 * (i % numDiv_x) + 1;
        int node5 = (2 * numDiv_x + 1) * (numDiv_y + 1) + (i / numDiv_x) * (numDiv_x + 1) + (i % numDiv_x) + 1;
        int node6 = (i / (numDiv_x)) * (2 * N_x - 1) + 2 * (i % numDiv_x) + 2 * N_x;
        int node7 = (2 * numDiv_x + 1) * (numDiv_y + 1) + (i / numDiv_x) * (numDiv_x + 1) + (i % numDiv_x);

        GetLog() << "Node 0: " << node0 << "\n";
        GetLog() << "Node 1: " << node1 << "\n";
        GetLog() << "Node 2: " << node2 << "\n";
        GetLog() << "Node 3: " << node3 << "\n";
        GetLog() << "Node 4: " << node4 << "\n";
        GetLog() << "Node 5: " << node5 << "\n";
        GetLog() << "Node 6: " << node6 << "\n";
        GetLog() << "Node 7: " << node7 << "\n";
        GetLog() << "Node 1 End: " << (2 * numDiv_x + 1) * (numDiv_y + 1) - 1 << "\n";
        GetLog() << "Node 2 End: " << (2 * numDiv_x + 1) * (numDiv_y)-1 << "\n";
        GetLog() << "Node 3 End: " << (2 * numDiv_x + 1) * (numDiv_y + 1) + ((TotalNumElements - 1) / numDiv_x) * (numDiv_x + 1) + ((TotalNumElements - 1) % numDiv_x) + 1 << "\n";
        GetLog() << "Node 1 Location: " << nodetip1->GetPos().x() << " " << nodetip1->GetPos().y() << "  " << nodetip1->GetPos().z() << "\n";
        GetLog() << "Node 2 Location: " << nodetip2->GetPos().x() << " " << nodetip2->GetPos().y() << "  " << nodetip2->GetPos().z() << "\n";
        GetLog() << "Node 3 Location: " << nodetip3->GetPos().x() << " " << nodetip3->GetPos().y() << "  " << nodetip3->GetPos().z() << "\n";

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF_8>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node3)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node4)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node5)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node6)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzDD>(my_mesh->GetNode(node7)));

        // Set element dimensions
        element->SetDimensions(dx, dy);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        element->SetAlphaDamp(0.005);  // Structural damping for this element
        element->SetGravityOn(false);  // turn internal gravitational force calculation off

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    my_system.Add(my_mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

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
    mvisualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
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
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(300);
    solver->SetTolerance(1e-14);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(true);

    // Set up integrator
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(10000);
    mystepper->SetAbsTolerances(1e-05);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);
    application.SetTimestep(time_step);

    while (application.GetDevice()->run()) {
        std::cout << "Time: " << my_system.GetChTime() << "s. \n";
        if (my_system.GetChTime() < 0.1) {
            nodetip1->SetForce(ChVector<>(0, 0, -20.0/3 * my_system.GetChTime()));
            nodetip2->SetForce(ChVector<>(0, 0, -20.0/3 * my_system.GetChTime()));
            nodetip3->SetForce(ChVector<>(0, 0, -20.0/3 * my_system.GetChTime()));
		} else {
			nodetip1->SetForce(ChVector<>(0, 0, -2 / 3.0));
			nodetip2->SetForce(ChVector<>(0, 0, -2 / 3.0));
			nodetip3->SetForce(ChVector<>(0, 0, -2 / 3.0));
		}

        GetLog() << "Node tip vertical position: " << nodetip1->GetPos().z()
				<< "\n";

		auto element = std::dynamic_pointer_cast<ChElementShellANCF_8>(
				my_mesh->GetElement(TotalNumElements - 1));
		const ChStrainStress3D strainStressOut =
				element->EvaluateSectionStrainStress(ChVector<double> (0, 0, 0), 0);

		std::cout << "Strain xx: " << strainStressOut.strain[0] << " \n";
		std::cout << "Strain yy: " << strainStressOut.strain[1] << " \n";
		std::cout << "Strain xy: " << strainStressOut.strain[2] << " \n";

		std::cout << "Stress xx: " << strainStressOut.stress[0] << " \n";
		std::cout << "Stress yy: " << strainStressOut.stress[1] << " \n";
		std::cout << "Stress xy: " << strainStressOut.stress[2] << " \n";

        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
