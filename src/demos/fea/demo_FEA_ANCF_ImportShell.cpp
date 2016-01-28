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

#include "chrono/lcp/ChLcpIterativeMINRES.h"
//#include "chrono_mkl/ChLcpMklSolver.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/utils/ChMeshImport.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#include <iostream>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <functional>

using namespace chrono;
using namespace fea;
using namespace chrono::irrlicht;
using namespace irr;
using namespace scene;
using namespace std;

bool addConstrain = false;
bool addForce = false;
bool addGravity = false;
bool addPressure = true;
bool showBone = true;
bool addFixed = true;
double time_step = 0.01;
int scaleFactor = 1;
double dz = 0.01;

int main(int argc, char* argv[]) {
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"ANCF Shells", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();

    application.AddTypicalCamera(core::vector3df(-1.f, 0.f, -0.5f),  // camera location
                                 core::vector3df(0.0f, 0.f, 0.f));   // "look at" location
    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "   		Articular Cartilage Modeling   \n";
    GetLog() << "-----------------------------------------------------------\n\n";

    // Creating Rigid bod
    GetLog() << "	Adding the Bone as a Rigid Body ...\n";
    ChSharedPtr<ChBody> Body_Bone;  // Bone
    Body_Bone = ChSharedPtr<ChBody>(new ChBody);
    my_system.AddBody(Body_Bone);
    Body_Bone->SetIdentifier(1);
    Body_Bone->SetBodyFixed(true);
    Body_Bone->SetCollide(false);
    Body_Bone->SetMass(1);
    Body_Bone->SetInertiaXX(ChVector<>(1, 0.2, 1));
    Body_Bone->SetPos(ChVector<>(0, 0, 0));
    ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
    mobjmesh->SetFilename(GetChronoDataFile("fea/bone35.obj"));
    if (showBone) {
        Body_Bone->AddAsset(mobjmesh);
    }
    GetLog() << "-----------------------------------------------------------\n\n";

    // The physical system: it contains all physical objects.
    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.

    int TotalNumNodes, TotalNumElements, TottalNumBEdges;
    std::vector<int> BC_NODES;

    GetLog() << "	Adding the Membrane Using ANCF Shell Elements...  \n\n";

    // Creating the membrane shell
    ChSharedPtr<ChMaterialShellANCF> material(new ChMaterialShellANCF(500, 1e7, 0.3));
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);
    ChVector<> Center(0, 0, 0);
    ChMatrix33<> rot_transform(1);

    try {
        my_mesh->LoadANCFShellFromGMFFile(GetChronoDataFile("fea/Torus.mesh").c_str(), material, BC_NODES, Center,
                                          rot_transform, scaleFactor, true, false, false);
    } catch (ChException myerr) {
        GetLog() << myerr.what();
        return 0;
    }

    TotalNumNodes = my_mesh->GetNnodes();
    TotalNumElements = my_mesh->GetNelements();

    for (int ele = 0; ele < TotalNumElements; ele++) {
        ChSharedPtr<ChElementShellANCF> element(my_mesh->GetElement(ele).DynamicCastTo<ChElementShellANCF>());

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, material);
        // Set other element properties
        element->SetAlphaDamp(0.08);   // Structural damping for this element
        element->SetGravityOn(false);  // gravitational forces
    }

    if (addFixed) {
        for (int node = 0; node < BC_NODES.size(); node++) {
            ChSharedPtr<ChNodeFEAxyzD> FixedNode(my_mesh->GetNode(BC_NODES[node]).DynamicCastTo<ChNodeFEAxyzD>());
            FixedNode->SetFixed(true);

            //            if (FixedNode->GetPos().x < -0.65)
            //                FixedNode->SetFixed(true);
            //            if (FixedNode->GetPos().x > 0.68)
            //                FixedNode->SetForce(ChVector<>(10, 0, 0));
        }
    }

    if (addConstrain) {
        for (int node = 0; node < BC_NODES.size(); node++) {
            ChSharedPtr<ChLinkPointFrame> NodePosBone;
            ChSharedPtr<ChLinkDirFrame> NodeDirBone;
            ChSharedPtr<ChNodeFEAxyzD> ConstrainedNode;
            ConstrainedNode =
                ChSharedPtr<ChNodeFEAxyzD>(my_mesh->GetNode(BC_NODES[node]).DynamicCastTo<ChNodeFEAxyzD>());
            NodePosBone = ChSharedPtr<ChLinkPointFrame>(new ChLinkPointFrame);
            NodePosBone->Initialize(ConstrainedNode, Body_Bone);
            my_system.Add(NodePosBone);

            NodeDirBone = ChSharedPtr<ChLinkDirFrame>(new ChLinkDirFrame);
            NodeDirBone->Initialize(ConstrainedNode, Body_Bone);
            NodeDirBone->SetDirectionInAbsoluteCoords(ConstrainedNode->D);
            my_system.Add(NodeDirBone);
        }
    }

    if (addForce) {
        ChSharedPtr<ChNodeFEAxyzD> forceNode(my_mesh->GetNode(0).DynamicCastTo<ChNodeFEAxyzD>());
        forceNode->SetForce(ChVector<>(0.0, 1, 0.0));
    }
    if (addPressure) {
        // First: loads must be added to "load containers",
        // and load containers must be added to your ChSystem
        ChSharedPtr<ChLoadContainer> Mloadcontainer(new ChLoadContainer);
        // Add constant pressure using ChLoaderPressure (preferred for simple, constant pressure)
        for (int NoElmPre = 0; NoElmPre < TotalNumElements; NoElmPre++) {
            ChSharedPtr<ChLoad<ChLoaderPressure>> faceload(
                new ChLoad<ChLoaderPressure>(my_mesh->GetElement(NoElmPre).StaticCastTo<ChElementShellANCF>()));
            faceload->loader.SetPressure(350);
            faceload->loader.SetStiff(false);
            faceload->loader.SetIntegrationPoints(2);
            Mloadcontainer->Add(faceload);
        }
        my_system.Add(Mloadcontainer);
    }

    // Switch off mesh class gravity
    my_mesh->SetAutomaticGravity(addGravity);
    my_system.Set_G_acc(ChVector<>(0, 0.0, -9.8));

    // Add the mesh to the system
    my_system.Add(my_mesh);

    // Mark completion of system construction
    my_system.SetupInitial();

    ////////////////////////////////////////
    // Options for visualization in irrlicht
    ////////////////////////////////////////
    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemesh(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetShrinkElements(true, 0.85);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshref(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemeshref->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshref->SetWireframe(true);
    mvisualizemeshref->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(mvisualizemeshref);

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshC(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.0004 * scaleFactor);
    my_mesh->AddAsset(mvisualizemeshC);

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshD(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    // mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED);
    mvisualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    mvisualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshD->SetSymbolsScale(1);
    mvisualizemeshD->SetColorscaleMinMax(-0.5, 5);
    mvisualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    GetLog() << "\n\nREADME\n\n"
             << " - Press SPACE to start dynamic simulation \n - Press F10 for nonlinear statics - Press F11 for "
                "linear statics. \n";

    // at beginning, no analysis is running..
    //    application.SetPaused(true);
    //    int AccuNoIterations = 0;
    //    application.SetStepManage(true);
    //    application.SetTimestep(time_step);
    //    application.SetTryRealtime(true);

    // ---------------
    // Simulation loop
    // ---------------

    // Setup solver
        my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
        ChLcpIterativeMINRES* msolver = (ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
        msolver->SetDiagonalPreconditioning(true);
        my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of
        my_system.SetIterLCPmaxItersSpeed(460);
        my_system.SetIterLCPmaxItersStab(1000);
        my_system.SetTolForce(1e-6);
        msolver->SetVerbose(false);

//
//    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;  // MKL Solver option
//    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
//    my_system.ChangeLcpSolverStab(mkl_solver_stab);
//    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
//    mkl_solver_speed->SetSparsityPatternLock(true);
//    mkl_solver_stab->SetSparsityPatternLock(true);

    my_system.SetIntegrationType(ChSystem::INT_HHT);
    ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>();
    mystepper->SetAlpha(-0.02);
    mystepper->SetMaxiters(100);
    mystepper->SetTolerance(1e-06);
    mystepper->SetMode(ChTimestepperHHT::POSITION);
    mystepper->SetScaling(true);
    //    mystepper->SetVerbose(true);

    application.SetTimestep(time_step);
    //    ChSharedPtr<ChNodeFEAxyzD> sampleNode(my_mesh->GetNode(89).DynamicCastTo<ChNodeFEAxyzD>());
    //    double y0 = sampleNode->pos.y;
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        //        std::cout << "Time t = " << my_system.GetChTime() << "s \t";
        //        std::cout << "pos.y = " << sampleNode->pos.y - y0 << "vs. " << -0.5 * 9.8 *
        //        pow(my_system.GetChTime(),
        //        2)
        //                  << "\n";
        double t_s = my_system.GetChTime();

        application.DoStep();
        application.EndScene();
    }

    return 0;
}
