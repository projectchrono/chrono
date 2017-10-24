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
// FEA for 3D beams
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementBeamIGA.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;


    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.

    auto msection = std::make_shared<ChBeamSectionAdvanced>();

    double beam_wy = 0.012;
    double beam_wz = 0.025;
    msection->SetAsRectangularSection(beam_wy, beam_wz);
    msection->SetYoungModulus(0.01e9);
    msection->SetGshearModulus(0.01e9 * 0.3);
    msection->SetBeamRaleyghDamping(0.000);
    // msection->SetCentroid(0,0.02);
    // msection->SetShearCenter(0,0.1);
    // msection->SetSectionRotation(45*CH_C_RAD_TO_DEG);

    //
    // Add some IGA BEAMS:
    //

    double beam_L = 0.1;

    auto hnode1 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0)));
    auto hnode2 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L, 0, 0)));
    auto hnode3 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 2, 0, 0)));

    my_mesh->AddNode(hnode1);
    my_mesh->AddNode(hnode2);
    my_mesh->AddNode(hnode3);

    // spline lineare con tre control points e 5 knots= {0 0 1 2 2}
    auto belement1 = std::make_shared<ChElementBeamIGA>();

    belement1->SetNodesLinear(hnode1, hnode2, 0,0, 1,2);
    belement1->SetSection(msection);

    my_mesh->AddElement(belement1);

    auto belement2 = std::make_shared<ChElementBeamIGA>();

    belement2->SetNodesLinear(hnode2, hnode3, 0,1, 2,2);
    belement2->SetSection(msection);

    my_mesh->AddElement(belement2);

    // Apply a force or a torque to a node:
    // hnode2->SetForce(ChVector<>(4, 2, 0));
    // hnode3->SetTorque( ChVector<>(0, -0.04, 0));

    // Fix a node to ground:
    hnode1->SetFixed(true);



    //
    // Final touches..
    //

    // We do not want gravity effect on FEA elements in this demo
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    /*
    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);
    */

    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);



    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"IGA beams (SPACE for dynamics, F10 / F11 statics)", core::dimension2d<u32>(800, 600),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

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
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(460);
    my_system.SetMaxItersSolverStab(460);
    my_system.SetTolForce(1e-13);

    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);


    application.SetTimestep(0.001);

    GetLog() << "\n\n\n===========STATICS======== \n\n\n";

//    application.GetSystem()->DoStaticLinear();

    GetLog() << "BEAM RESULTS (LINEAR STATIC ANALYSIS) \n\n";

    ChVector<> F, M;
    ChMatrixDynamic<> displ;

    belement1->GetStateBlock(displ);
    GetLog() << displ;
    for (double eta = -1; eta <= 1; eta += 0.4) {
        belement1->EvaluateSectionForceTorque(eta, F, M);
        GetLog() << "  b1_at " << eta << " Mx=" << M.x() << " My=" << M.y() << " Mz=" << M.z() << " Tx=" << F.x()
                 << " Ty=" << F.y() << " Tz=" << F.z() << "\n";
    }
    GetLog() << "\n";
    belement2->GetStateBlock(displ);
    for (double eta = -1; eta <= 1; eta += 0.4) {
        belement2->EvaluateSectionForceTorque(eta, F, M);
        GetLog() << "  b2_at " << eta << " Mx=" << M.x() << " My=" << M.y() << " Mz=" << M.z() << " Tx=" << F.x()
                 << " Ty=" << F.y() << " Tz=" << F.z() << "\n";
    }

    GetLog() << "Node 3 coordinate x= " << hnode3->Frame().GetPos().x() << "    y=" << hnode3->Frame().GetPos().y()
             << "    z=" << hnode3->Frame().GetPos().z() << "\n\n";

    GetLog() << "Press SPACE bar to start/stop dynamic simulation \n\n";
    GetLog() << "Press F10 for nonlinear static solution \n\n";
    GetLog() << "Press F11 for linear static solution \n\n";

    application.SetPaused(true);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
