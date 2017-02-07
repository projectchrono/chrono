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
//     - FEA for 3D beams and constrains

// Include some headers used by this tutorial...

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkRackpinion.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualization.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_matlab/ChMatlabEngine.h"
#include "chrono_matlab/ChSolverMatlab.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystem my_system;

    double scales = 100;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Beams and constraints", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, (f32)(scales * 0.01), (f32)(scales * 0.01)));
    application.GetSceneManager()->getActiveCamera()->setNearValue(0.001f);
    application.GetSceneManager()->getActiveCamera()->setFarValue((f32)(scales * 0.03));

    double thickZ = scales * 0.00015;
    double hbarW = scales * 0.00070;
    double hbarL1 = scales * 0.00381;
    double hbarL2 = scales * 0.00387;
    double hbarL3 = scales * 0.00381;
    double vbarL = scales * 0.01137;
    double vbarW = scales * 0.00006;
    double Rpinion = scales * 0.00040;
    double OffPin = scales * 0.00050;
    double Rbalance = scales * 0.00500;
    double Wbalance = scales * 0.00015;
    bool simple_rack = false;

    ChVector<> vAh(-hbarL1 - hbarL2 * 0.5, vbarL, 0);
    ChVector<> vBh(-hbarL2 * 0.5, vbarL, 0);
    ChVector<> vCh(hbarL2 * 0.5, vbarL, 0);
    ChVector<> vDh(hbarL1 + hbarL2 * 0.5, vbarL, 0);
    ChVector<> vAl(-hbarL1 - hbarL2 * 0.5, 0, 0);
    ChVector<> vBl(-hbarL2 * 0.5, 0, 0);
    ChVector<> vCl(hbarL2 * 0.5, 0, 0);
    ChVector<> vDl(hbarL1 + hbarL2 * 0.5, 0, 0);
    ChVector<> vP(0, -Rpinion - hbarW * 0.5, 0);

    // Create a truss:
    auto body_truss = std::make_shared<ChBody>();

    body_truss->SetBodyFixed(true);

    my_system.AddBody(body_truss);

    /*
    // Attach a 'box' shape asset for visualization.
    auto mboxtruss = std::make_shared<ChBoxShape>();
    mboxtruss->GetBoxGeometry().Pos  = ChVector<>(-0.01, 0,0);
    mboxtruss->GetBoxGeometry().SetLengths( ChVector<>(0.02, 0.2, 0.1) );
    body_truss->AddAsset(mboxtruss);
    */

    // Create a FEM mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create the horizontal beams

    auto msectionH = std::make_shared<ChBeamSectionAdvanced>();

    msectionH->SetDensity(7000);  //***TEST*** must be 7k
    msectionH->SetYoungModulus(200.0e9);
    msectionH->SetGwithPoissonRatio(0.32);
    msectionH->SetAsRectangularSection(hbarW, thickZ);
    msectionH->SetBeamRaleyghDamping(0.00);

    ChBuilderBeam builder;

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msectionH,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      2,                     // the number of ChElementBeamEuler to create
                      vAh,                   // the 'Ah' point in space (beginning of beam)
                      vBh,                   // the 'Bh' point in space (end of beam)
                      ChVector<>(0, 1, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam
    std::shared_ptr<ChNodeFEAxyzrot> node_Ah = builder.GetLastBeamNodes().front();
    std::shared_ptr<ChNodeFEAxyzrot> node_Bh = builder.GetLastBeamNodes().back();

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msectionH,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      2,                     // the number of ChElementBeamEuler to create
                      node_Bh,               // the 'Bh' point in space (beginning of beam)
                      vCh,                   // the 'Ch' point in space (end of beam)
                      ChVector<>(0, 1, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam
    std::shared_ptr<ChNodeFEAxyzrot> node_Ch = builder.GetLastBeamNodes().back();

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msectionH,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      2,                     // the number of ChElementBeamEuler to create
                      node_Ch,               // the 'Ch' point in space (beginning of beam)
                      vDh,                   // the 'Dh' point in space (end of beam)
                      ChVector<>(0, 1, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam
    std::shared_ptr<ChNodeFEAxyzrot> node_Dh = builder.GetLastBeamNodes().back();

    // Create the vertical flexible beams

    auto msectionV = std::make_shared<ChBeamSectionAdvanced>();

    msectionV->SetDensity(7000);  //***TEST*** must be 7k
    msectionV->SetYoungModulus(200.0e9);
    msectionV->SetGwithPoissonRatio(0.32);
    msectionV->SetAsRectangularSection(vbarW, thickZ);
    msectionV->SetBeamRaleyghDamping(0.00);

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msectionV,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      6,                     // the number of ChElementBeamEuler to create
                      node_Ah,               // the 'Ah' point in space (beginning of beam)
                      vAl,                   // the 'Al' point in space (end of beam)
                      ChVector<>(1, 0, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam
    std::shared_ptr<ChNodeFEAxyzrot> node_Al = builder.GetLastBeamNodes().back();

    node_Al->SetFixed(true);

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msectionV,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      6,                     // the number of ChElementBeamEuler to create
                      node_Dh,               // the 'Dh' point in space (beginning of beam)
                      vDl,                   // the 'Dl' point in space (end of beam)
                      ChVector<>(1, 0, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam
    std::shared_ptr<ChNodeFEAxyzrot> node_Dl = builder.GetLastBeamNodes().back();

    node_Dl->SetFixed(true);

    // Create the inner vertical flexible beams

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msectionV,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      6,                     // the number of ChElementBeamEuler to create
                      node_Bh,               // the 'Bh' point in space (beginning of beam)
                      vBl,                   // the 'Bl' point in space (end of beam)
                      ChVector<>(1, 0, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam
    std::shared_ptr<ChNodeFEAxyzrot> node_Bl = builder.GetLastBeamNodes().back();

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msectionV,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      6,                     // the number of ChElementBeamEuler to create
                      node_Ch,               // the 'Dh' point in space (beginning of beam)
                      vCl,                   // the 'Dl' point in space (end of beam)
                      ChVector<>(1, 0, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam
    std::shared_ptr<ChNodeFEAxyzrot> node_Cl = builder.GetLastBeamNodes().back();

    // Create the rack

    if (simple_rack) {
        builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                          msectionH,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                          1,                     // the number of ChElementBeamEuler to create
                          node_Bl,               // the 'Cl' point in space (beginning of beam)
                          node_Cl,               // the 'Dl' point in space (end of beam)
                          ChVector<>(0, 1, 0));  // the 'Y' up direction of the section for the beam
    }

    //
    // Final touches to mesh..
    //

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);  // E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-30, 30);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.001);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    //
    // The balance and the rigid rach
    //

    if (!simple_rack) {
        auto rack = std::make_shared<ChBodyEasyBox>(hbarL2, hbarW, thickZ, 7000, false);
        rack->SetPos(0.5 * (vBl + vCl));
        my_system.Add(rack);

        auto constr_B = std::make_shared<ChLinkMateGeneric>();
        constr_B->Initialize(node_Bl, rack, false, node_Bl->Frame(), node_Bl->Frame());
        my_system.Add(constr_B);

        auto constr_C = std::make_shared<ChLinkMateGeneric>();
        constr_C->Initialize(node_Cl, rack, false, node_Cl->Frame(), node_Cl->Frame());
        my_system.Add(constr_C);

        auto balance = std::make_shared<ChBodyEasyCylinder>(Rbalance, Wbalance, 7000, false);
        balance->SetPos(vP + ChVector<>(0, 0, -OffPin));
        balance->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
        for (int i = 0; i < 6; ++i) {
            double phi = CH_C_2PI * (i / 6.0);
            auto vshape = std::make_shared<ChCylinderShape>();
            vshape->GetCylinderGeometry().p1 =
                ChVector<>(sin(phi) * Rbalance * 0.8, Wbalance, cos(phi) * Rbalance * 0.8);
            vshape->GetCylinderGeometry().p2 = vshape->GetCylinderGeometry().p1 + ChVector<>(0, 2 * Wbalance, 0);
            vshape->GetCylinderGeometry().rad = Rbalance * 0.1;
            balance->AddAsset(vshape);
        }
        auto vshaft = std::make_shared<ChCylinderShape>();
        vshaft->GetCylinderGeometry().p1 = vP + ChVector<>(0, -OffPin * 10, 0);
        vshaft->GetCylinderGeometry().p2 = vP + ChVector<>(0, OffPin * 10, 0);
        vshaft->GetCylinderGeometry().rad = Rpinion;
        balance->AddAsset(vshaft);
        auto mcol = std::make_shared<ChColorAsset>();
        mcol->SetColor(ChColor(0.5f, 0.9f, 0.9f));
        balance->AddAsset(mcol);

        my_system.Add(balance);

        auto revolute = std::make_shared<ChLinkLockRevolute>();
        std::shared_ptr<ChBody> mbalance = balance;
        revolute->Initialize(mbalance, body_truss, ChCoordsys<>(vP + ChVector<>(0, 0, -0.01)));

        my_system.Add(revolute);

        auto constr_rack = std::make_shared<ChLinkRackpinion>();
        constr_rack->Initialize(balance, rack, false, ChFrame<>(), ChFrame<>());

        ChFrameMoving<> f_pin_abs(vP);
        ChFrameMoving<> f_rack_abs(vP + ChVector<>(0, 0.1, 0));
        ChFrameMoving<> f_pin;
        ChFrameMoving<> f_rack;
        balance->TransformParentToLocal(f_pin_abs, f_pin);
        rack->TransformParentToLocal(f_rack_abs, f_rack);
        constr_rack->SetPinionRadius(Rpinion);
        constr_rack->SetPinionFrame(f_pin);
        constr_rack->SetRackFrame(f_rack);

        my_system.Add(constr_rack);

        balance->SetWvel_par(ChVector<>(0, 0, 1.5));
    }

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
    my_system.SetSolverWarmStarting(true);
    my_system.SetMaxItersSolverSpeed(400);
    my_system.SetMaxItersSolverStab(400);
    my_system.SetTolForce(1e-25);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(true);
    msolver->SetDiagonalPreconditioning(false);

    //***TEST***
    ChMatlabEngine matlab_engine;
    auto matlab_solver = std::make_shared<ChSolverMatlab>(matlab_engine);
    my_system.SetSolver(matlab_solver);

    my_system.Set_G_acc(ChVector<>(0, 0, 0));

    // Do a static solution
    application.SetPaused(true);

    GetLog() << "STATIC linear solve ----\n";
    node_Cl->SetForce(ChVector<>(50, 0, 0));
    // application.GetSystem()->DoStaticLinear();
    node_Cl->SetForce(ChVector<>(0, 0, 0));

    if (simple_rack) {
        node_Cl->SetForce(ChVector<>(50, 0, 0));
        application.GetSystem()->DoStaticNonlinear(12);
        node_Cl->SetForce(ChVector<>(0, 0, 0));
    }

    application.SetTimestep(0.01);
    application.SetVideoframeSaveInterval(10);
    application.SetSymbolscale(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 10, 10, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_Z),
                             video::SColor(50, 90, 100, 100), true);

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
