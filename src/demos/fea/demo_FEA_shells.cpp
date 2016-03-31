//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//
//   Demo code about
//
//     - FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)

// Include some headers used by this tutorial...

#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/lcp/ChLcpIterativePMINRES.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChElementShellEANS4.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Shells FEA", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    //my_system.Set_G_acc(VNULL); or 
    my_mesh->SetAutomaticGravity(false);

    //
    // Add an ANCF SHELL:
    //
    if(false)
    {
        double shell_thickness = 0.01;
        double shell_L = 0.4;
        double shell_W = 0.2;

        // Create the nodes (each with position & normal to shell)
        auto hnodeancf1 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0.02, 0), ChVector<>(0, 1, 0));
        auto hnodeancf2 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(shell_L, 0, 0), ChVector<>(0, 1, 0));
        auto hnodeancf3 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, shell_W), ChVector<>(0, 1, 0));
        auto hnodeancf4 = std::make_shared<ChNodeFEAxyzD>(ChVector<>(shell_L, 0, shell_W), ChVector<>(0, 1, 0));

        my_mesh->AddNode(hnodeancf1);
        my_mesh->AddNode(hnodeancf2);
        my_mesh->AddNode(hnodeancf3);
        my_mesh->AddNode(hnodeancf4);

        // Create the element

        auto elementancf1 = std::make_shared<ChElementShellANCF>();
        my_mesh->AddElement(elementancf1);

        // Set its nodes
        elementancf1->SetNodes(hnodeancf1, hnodeancf2, hnodeancf3, hnodeancf4);

        // Set element dimensions
        elementancf1->SetDimensions(shell_L, shell_W);

        // Create an orthotropic material 
        double rho = 500;
        ChVector<> E(2.1e7, 2.1e7, 2.1e7);
        ChVector<> nu(0.3, 0.3, 0.3);
        ChVector<> G(8.0769231e6, 8.0769231e6, 8.0769231e6);
        auto mat = std::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

        // Add a single layers with a fiber angle of 0 degrees.
        elementancf1->AddLayer(shell_thickness, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        elementancf1->SetAlphaDamp(0.0);    // Structural damping for this element
        elementancf1->SetGravityOn(false);  // turn internal gravitational force calculation off

        // Apply a lumped force to a node:
        hnodeancf3->SetForce(ChVector<>(0, 50, 0));

        hnodeancf1->SetFixed(true);
        hnodeancf2->SetFixed(true);
    }

    //
    // Add an EANS SHELL:
    //
    if (true)
    {
        double shell_thickness = 0.01;
        double shell_L = 0.4;
        double shell_W = 0.2;

        // Create an orthotropic material 
        double rho = 500;
        double E = 2.1e7;
        double nu = 0.3; 
        auto mat = std::make_shared<ChMaterialShellEANS>(shell_thickness,
                                                         rho, 
                                                         E, 
                                                         nu,
                                                         1,
                                                         0.01);

        // Create the nodes (each with position & normal to shell)
        auto hnodeeans1 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0)));
        auto hnodeeans2 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(shell_L, 0, 0)));
        auto hnodeeans3 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, shell_W, 0 )));
        auto hnodeeans4 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(shell_L, shell_W, 0)));

        my_mesh->AddNode(hnodeeans1);
        my_mesh->AddNode(hnodeeans2);
        my_mesh->AddNode(hnodeeans3);
        my_mesh->AddNode(hnodeeans4);

        hnodeeans1->SetFixed(true);
        hnodeeans2->SetFixed(true);

        // Create the element

        auto elementeans = std::make_shared<ChElementShellEANS4>();
        my_mesh->AddElement(elementeans);

        // Set its nodes
        elementeans->SetNodes(hnodeeans1, hnodeeans2, hnodeeans3, hnodeeans4);
/*
        elementeans->SetNodeAreferenceRot(Q_from_AngAxis(-CH_C_PI_2, VECT_X));
        elementeans->SetNodeBreferenceRot(Q_from_AngAxis(-CH_C_PI_2, VECT_X));
        elementeans->SetNodeCreferenceRot(Q_from_AngAxis(-CH_C_PI_2, VECT_X));
        elementeans->SetNodeDreferenceRot(Q_from_AngAxis(-CH_C_PI_2, VECT_X));
*/
        // Set element dimensions
        //elementeans->SetDimensions(shell_L, shell_W); // not needed, already set at initialization from initial pos of nodes

        // Add a single layers with a fiber angle of 0 degrees.
        elementeans->AddLayer(shell_thickness, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        elementeans->SetAlphaDamp(0.0);    // Structural damping for this element

        // Apply a lumped force to a node:
       // hnodeeans3->SetPos(hnodeeans3->GetPos()+ChVector<>(0, 0, 0.01));
       // hnodeeans4->SetPos(hnodeeans4->GetPos()+ChVector<>(0, 0, 0.01));
        hnodeeans3->SetForce(ChVector<>(0, 3000, 0));
        hnodeeans4->SetForce(ChVector<>(0, 3000, 0));
       // hnodeeans3->SetForce(ChVector<>(0, 0, 50));
      //  hnodeeans4->SetForce(ChVector<>(0, 0, 50));
      // hnodeeans3->SetTorque(ChVector<>(0.2, 0, 0));
      // hnodeeans4->SetTorque(ChVector<>(0.2, 0, 0));
       // hnodeeans4->SetMass(2000);


        /*
        auto hnodeeans5 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, shell_W*2, 0 )));
        auto hnodeeans6 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(shell_L, shell_W*2, 0)));

        my_mesh->AddNode(hnodeeans5);
        my_mesh->AddNode(hnodeeans6);

        // Create the element

        auto elementeansb = std::make_shared<ChElementShellEANS4>();
        my_mesh->AddElement(elementeansb);

        // Set its nodes
        elementeansb->SetNodes(hnodeeans3, hnodeeans4, hnodeeans5, hnodeeans6);

        // Set element dimensions
        //elementeans->SetDimensions(shell_L, shell_W); // not needed, already set at initialization from initial pos of nodes

        // Add a single layers with a fiber angle of 0 degrees.
        elementeansb->AddLayer(shell_thickness, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        elementeansb->SetAlphaDamp(0.0);    // Structural damping for this element

        hnodeeans5->SetForce(ChVector<>(0, 1000, 0));
        hnodeeans6->SetForce(ChVector<>(0, 1000, 0));
        //hnodeeans5->SetTorque(ChVector<>(5, 0, 0));
        //hnodeeans6->SetTorque(ChVector<>(5, 0, 0));
        //hnodeeans6->SetMass(2000);
        */
    }

    

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizeshellA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizeshellA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);// not yet working
    // mvisualizeshellA->SetColorscaleMinMax(-0.4,0.4);
    mvisualizeshellA->SetSmoothFaces(true);
	mvisualizeshellA->SetWireframe(true);
	my_mesh->AddAsset(mvisualizeshellA);

    auto mvisualizeshellB = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    // mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);// not yet working
    mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizeshellB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizeshellB->SetSymbolsThickness(0.006);
    mvisualizeshellB->SetSymbolsScale(0.01);
    mvisualizeshellB->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizeshellB);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Mark completion of system construction
    my_system.SetupInitial();

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED THIS or Matlab or MKL solver
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(200);
	my_system.SetIterLCPmaxItersStab(200);
	my_system.SetTolForce(1e-13);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);

    // Change type of integrator:
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    /*
    my_system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetAbsTolerances(1e-6);
    }
    */
    application.SetTimestep(0.01);
    application.SetPaused(true);
    my_system.Setup();
    my_system.Update();
    //my_system.DoStaticLinear();

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}


