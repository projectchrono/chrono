

// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//   Demos code about
//
//     - FEA using ANCF (introduction to dynamics)
//
//// Include some headers used by this tutorial...
#include "chrono/physics/ChSystem.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono_fea/ChElementSpring.h"
#include "chrono_fea/ChElementBrick.h"
#include "chrono_fea/ChElementBar.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_irrlicht/ChIrrApp.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Brick Elements", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();

    application.AddTypicalCamera(core::vector3df(1.2f, 0.6f, 0.3f),   // camera location
                                 core::vector3df(0.2f, -0.2f, 0.f));  // "look at" location

    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "     Brick Elements demo with implicit integration \n";
    GetLog() << "-----------------------------------------------------------\n";

    // The physical system: it contains all physical objects.
    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();
    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 1;
    double plate_lenght_z = 0.05;  // small thickness
    // Specification of the mesh
    int numDiv_x = 4;
    int numDiv_y = 4;
    int numDiv_z = 1;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    int N_z = numDiv_z + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1) * (numDiv_z + 1);
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z / numDiv_z;
    int MaxMNUM = 1;
    int MTYPE = 1;
    int MaxLayNum = 1;

    ChMatrixDynamic<double> COORDFlex(TotalNumNodes, 3);
    ChMatrixDynamic<double> VELCYFlex(TotalNumNodes, 3);
    ChMatrixDynamic<int> NumNodes(TotalNumElements, 8);
    ChMatrixDynamic<int> LayNum(TotalNumElements, 1);
    ChMatrixDynamic<int> NDR(TotalNumNodes, 3);
    ChMatrixDynamic<double> ElemLengthXY(TotalNumElements, 3);
    ChMatrixNM<double, 10, 12> MPROP;

    //!------------------------------------------------!
    //!------------ Read Material Data-----------------!
    //!------------------------------------------------!

    for (int i = 0; i < MaxMNUM; i++) {
        MPROP(i, 0) = 500;      // Density [kg/m3]
        MPROP(i, 1) = 2.1E+05;  // H(m)
        MPROP(i, 2) = 0.3;      // nu
    }
    auto mmaterial = std::make_shared<ChContinuumElastic>();
    mmaterial->Set_RayleighDampingK(0.0);
    mmaterial->Set_RayleighDampingM(0.0);
    mmaterial->Set_density(MPROP(0, 0));
    mmaterial->Set_E(MPROP(0, 1));
    mmaterial->Set_G(MPROP(0, 1) / (2 + 2 * MPROP(0, 2)));
    mmaterial->Set_v(MPROP(0, 2));
    //!------------------------------------------------!
    //!--------------- Element data--------------------!
    //!------------------------------------------------!
    for (int i = 0; i < TotalNumElements; i++) {
        // All the elements belong to the same layer, e.g layer number 1.
        LayNum(i, 0) = 1;
        // Node number of the 4 nodes which creates element i.
        // The nodes are distributed this way. First in the x direction for constant
        // y when max x is reached go to the
        // next level for y by doing the same   distribution but for y+1 and keep
        // doing until y limit is reached. Node
        // number start from 1.

        NumNodes(i, 0) = (i / (numDiv_x)) * (N_x) + i % numDiv_x;
        NumNodes(i, 1) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + 1;
        NumNodes(i, 2) = (i / (numDiv_x)) * (N_x)+i % numDiv_x + 1 + N_x;
        NumNodes(i, 3) = (i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x;
        NumNodes(i, 4) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 0);
        NumNodes(i, 5) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 1);
        NumNodes(i, 6) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 2);
        NumNodes(i, 7) = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes(i, 3);

        // Let's keep the element length a fixed number in both direction. (uniform
        // distribution of nodes in both direction)

        ElemLengthXY(i, 0) = dx;
        ElemLengthXY(i, 1) = dy;
        ElemLengthXY(i, 2) = dz;

        if (MaxLayNum < LayNum(i, 0)) { 
            MaxLayNum = LayNum(i, 0);
        }
    }
    //!----------------------------------------------!
    //!--------- NDR,COORDFlex,VELCYFlex-------------!
    //!----------------------------------------------!

    for (int i = 0; i < TotalNumNodes; i++) {
        // If the node is the first node from the left side fix the x,y,z degree of
        // freedom. 1 for constrained 0 for ...
        //-The NDR array is used to define the degree of freedoms that are
        // constrained in the 6 variable explained above.
        NDR(i, 0) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 1) = (i % (numDiv_x + 1) == 0) ? 1 : 0;
        NDR(i, 2) = (i % (numDiv_x + 1) == 0) ? 1 : 0;

        //-COORDFlex are the initial coordinates for each node,
        // the first three are the position
        COORDFlex(i, 0) = (i % (numDiv_x + 1)) * dx;
        COORDFlex(i, 1) = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy;
        COORDFlex(i, 2) = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz;

        //-VELCYFlex is essentially the same as COORDFlex, but for the initial
        // velocity instead of position.
        // let's assume zero initial velocity for nodes
        VELCYFlex(i, 0) = 0;
        VELCYFlex(i, 1) = 0;
        VELCYFlex(i, 2) = 0;
    }

    // Adding the nodes to the mesh
    int i = 0;
    while (i < TotalNumNodes) {
        auto node = std::make_shared<ChNodeFEAxyz>(ChVector<>(COORDFlex(i, 0), COORDFlex(i, 1), COORDFlex(i, 2)));
        node->SetMass(0.0);
        my_mesh->AddNode(node);
        if (NDR(i, 0) == 1 && NDR(i, 1) == 1 && NDR(i, 2) == 1) {
            node->SetFixed(true);
        }
        i++;
    }
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(TotalNumNodes - 1));

    int elemcount = 0;
    while (elemcount < TotalNumElements) {
        auto element = std::make_shared<ChElementBrick>();
        ChMatrixNM<double, 3, 1> InertFlexVec;  // read element length, used in ChElementBrick
        InertFlexVec.Reset();
        InertFlexVec(0, 0) = ElemLengthXY(elemcount, 0);
        InertFlexVec(1, 0) = ElemLengthXY(elemcount, 1);
        InertFlexVec(2, 0) = ElemLengthXY(elemcount, 2);
        element->SetInertFlexVec(InertFlexVec);
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 0))),
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 1))),
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 2))),
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 3))),
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 4))),
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 5))),
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 6))),
            std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(NumNodes(elemcount, 7))));

        element->SetMaterial(mmaterial);
        element->SetElemNum(elemcount);            // for EAS
        element->SetGravityOn(true);     // turn gravity on/off from within the element
        element->SetMooneyRivlin(false);  // turn on/off Mooney Rivlin (Linear Isotropic by default)
        ChMatrixNM<double, 9, 1> stock_alpha_EAS;  //
        stock_alpha_EAS.Reset();
        element->SetStockAlpha(stock_alpha_EAS(0, 0), stock_alpha_EAS(1, 0), stock_alpha_EAS(2, 0),
                               stock_alpha_EAS(3, 0), stock_alpha_EAS(4, 0), stock_alpha_EAS(5, 0),
                               stock_alpha_EAS(6, 0), stock_alpha_EAS(7, 0), stock_alpha_EAS(8, 0));
        my_mesh->AddElement(element);
        elemcount++;
    }
    // Deactivate automatic gravity in mesh
    my_mesh->SetAutomaticGravity(false);
    my_system.Set_G_acc(ChVector<>(0,0,-9.81));
    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // Options for visualization in irrlicht
    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_P);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(false);
    my_mesh->AddAsset(mvisualizemesh);

    auto mvisualizemeshref = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshC->SetSymbolsThickness(0.015);
    my_mesh->AddAsset(mvisualizemeshC);

    auto mvisualizemeshD = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NONE);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Mark completion of system construction
    my_system.SetupInitial();

    // Perform a dynamic time integration:
    my_system.SetSolverType(ChSolver::Type::MINRES);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetDiagonalPreconditioning(true);
    my_system.SetMaxItersSolverSpeed(1000);
    my_system.SetTolForce(1e-08);

    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(100);
    mystepper->SetAbsTolerances(1e-5);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);
    application.SetTimestep(0.004);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
