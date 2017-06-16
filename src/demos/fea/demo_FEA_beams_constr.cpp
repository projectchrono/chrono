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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// FEA for 3D beams and constraints
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace irr;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystemNSC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Beams and constraints", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

    double L = 1;
    double H = 0.25;
    double K = 0.05;
    ChVector<> vA(0, 0, 0);
    ChVector<> vC(L, 0, 0);
    ChVector<> vB(L, -H, 0);
    ChVector<> vG(L - K, -H, 0);
    ChVector<> vd(0, 0, 0.0001);

    // Create a truss:
    auto body_truss = std::make_shared<ChBody>();
    body_truss->SetBodyFixed(true);

    my_system.AddBody(body_truss);

    // Attach a 'box' shape asset for visualization.
    auto mboxtruss = std::make_shared<ChBoxShape>();
    mboxtruss->GetBoxGeometry().Pos = ChVector<>(-0.01, 0, 0);
    mboxtruss->GetBoxGeometry().SetLengths(ChVector<>(0.02, 0.2, 0.1));
    body_truss->AddAsset(mboxtruss);

    // Create body for crank
    auto body_crank = std::make_shared<ChBody>();

    body_crank->SetPos((vB + vG) * 0.5);
    my_system.AddBody(body_crank);

    // Attach a 'box' shape asset for visualization.
    auto mboxcrank = std::make_shared<ChBoxShape>();
    mboxcrank->GetBoxGeometry().Pos = ChVector<>(0, 0, 0);
    mboxcrank->GetBoxGeometry().SetLengths(ChVector<>(K, 0.02, 0.02));
    body_crank->AddAsset(mboxcrank);

    // Create a motor between the truss
    // and the crank:
    auto constr_motor = std::make_shared<ChLinkEngine>();
    constr_motor->Initialize(body_truss, body_crank, ChCoordsys<>(vG));
    my_system.Add(constr_motor);

    class ChFunction_myf : public ChFunction {
      public:
        virtual ChFunction_myf* Clone() const override { return new ChFunction_myf(); }

        virtual double Get_y(double x) const override {
            if (x > 0.4)
                return CH_C_PI;
            else
                return -CH_C_PI * (1.0 - cos(CH_C_PI * x / 0.4)) / 2.0;
        }
    };

    auto f_ramp = std::make_shared<ChFunction_myf>();
    constr_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    constr_motor->Set_rot_funct(f_ramp);

    // Create a FEM mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create the horizontal beam
    auto msection1 = std::make_shared<ChBeamSectionAdvanced>();

    double beam_wy = 0.10;
    double beam_wz = 0.01;
    msection1->SetDensity(2700);
    msection1->SetYoungModulus(73.0e9);
    msection1->SetGwithPoissonRatio(0.3);
    msection1->SetAsRectangularSection(beam_wy, beam_wz);
    msection1->SetBeamRaleyghDamping(0.000);

    // This helps creating sequences of nodes and ChElementBeamEuler elements:
    ChBuilderBeam builder;

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection1,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      16,                    // the number of ChElementBeamEuler to create
                      vA,                    // the 'A' point in space (beginning of beam)
                      vC,                    // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0));  // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam,
    // For example say you want to fix the A end and apply a force to the B end:
    builder.GetLastBeamNodes().front()->SetFixed(true);

    auto node_tip = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().back());
    auto node_mid = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes()[7]);

    // Create the vertical beam
    auto msection2 = std::make_shared<ChBeamSectionAdvanced>();

    double hbeam_d = 0.024;
    msection2->SetDensity(2700);
    msection2->SetYoungModulus(73.0e9);
    msection1->SetGwithPoissonRatio(0.3);
    msection2->SetBeamRaleyghDamping(0.000);
    msection2->SetAsCircularSection(hbeam_d);

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection2,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      3,                     // the number of ChElementBeamEuler to create
                      vC + vd,               // the 'A' point in space (beginning of beam)
                      vB + vd,               // the 'B' point in space (end of beam)
                      ChVector<>(1, 0, 0));  // the 'Y' up direction of the section for the beam
    auto node_top = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().front());
    auto node_down = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().back());

    // Create a constraint between the vertical and horizontal beams:
    auto constr_bb = std::make_shared<ChLinkMateGeneric>();
    constr_bb->Initialize(node_top, node_tip, false, node_top->Frame(), node_top->Frame());
    my_system.Add(constr_bb);

    constr_bb->SetConstrainedCoords(true, true, true,      // x, y, z
                                    false, false, false);  // Rx, Ry, Rz

    // For example, attach small shape to show the constraint
    auto msphereconstr2 = std::make_shared<ChSphereShape>();
    msphereconstr2->GetSphereGeometry().rad = 0.01;
    constr_bb->AddAsset(msphereconstr2);

    // Create a beam as a crank
    auto msection3 = std::make_shared<ChBeamSectionAdvanced>();

    double crankbeam_d = 0.048;
    msection3->SetDensity(2700);
    msection3->SetYoungModulus(73.0e9);
    msection3->SetGwithPoissonRatio(0.3);
    msection3->SetBeamRaleyghDamping(0.000);
    msection3->SetAsCircularSection(crankbeam_d);

    builder.BuildBeam(my_mesh,               // the mesh where to put the created nodes and elements
                      msection3,             // the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
                      3,                     // the number of ChElementBeamEuler to create
                      vG + vd,               // the 'A' point in space (beginning of beam)
                      vB + vd,               // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0));  // the 'Y' up direction of the section for the beam

    auto node_crankG = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().front());
    auto node_crankB = std::shared_ptr<ChNodeFEAxyzrot>(builder.GetLastBeamNodes().back());
    // Create a constraint between the crank beam and body crank:
    auto constr_cbd = std::make_shared<ChLinkMateGeneric>();
    constr_cbd->Initialize(node_crankG, body_crank, false, node_crankG->Frame(), node_crankG->Frame());
    my_system.Add(constr_cbd);

    constr_cbd->SetConstrainedCoords(true, true, true,   // x, y, z
                                     true, true, true);  // Rx, Ry, Rz

    // Create a constraint between the vertical beam and the crank beam:
    auto constr_bc = std::make_shared<ChLinkMateGeneric>();
    constr_bc->Initialize(node_down, node_crankB, false, node_crankB->Frame(), node_crankB->Frame());
    my_system.Add(constr_bc);

    constr_bc->SetConstrainedCoords(true, true, true,    // x, y, z
                                    true, true, false);  // Rx, Ry, Rz

    // For example, attach small shape to show the constraint
    auto msphereconstr3 = std::make_shared<ChSphereShape>();
    msphereconstr3->GetSphereGeometry().rad = 0.01;
    constr_bc->AddAsset(msphereconstr3);

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
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!
    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MX);
    mvisualizebeamA->SetColorscaleMinMax(-500, 500);
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

    // Use a solver that can handle stiffnss matrices:

    //***TEST***
    /*
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);
    my_system.SetMaxItersSolverSpeed(600);
    my_system.SetMaxItersSolverStab(600);
    my_system.SetTolForce(1e-20);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(true);
    msolver->SetDiagonalPreconditioning(false);
    */

    //***TEST***
    /*ChMatlabEngine matlab_engine;
    auto matlab_solver = std::make_shared<ChSolverMatlab>(matlab_engine);
    my_system.SetSolver(matlab_solver);*/

    //***TEST***
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);

    application.SetTimestep(0.0005);
    application.SetVideoframeSaveInterval(10);

    // Use the following for less numerical damping, 2nd order accuracy (but slower)
    // my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetVerbose(true);
        mystepper->SetStepControl(false);
    }

    // Output data
    chrono::ChStreamOutAsciiFile file_out1("benchmark_CE_buckling_mid.dat");

    while (application.GetDevice()->run()) {
        // builder.GetLastBeamNodes().back()->SetTorque(ChVector<>(0, 0, 0.1 * application.GetSystem()->GetChTime()));

        application.BeginScene();

        application.DrawAll();

        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.05, 0.05, 20, 20, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_Z),
                             video::SColor(50, 90, 90, 90), true);

        application.DoStep();

        // Save output for the first 0.4 seconds
        if (application.GetSystem()->GetChTime() <= 0.4) {
            file_out1 << application.GetSystem()->GetChTime() << " " << node_mid->GetPos().z() << " "
                      << node_mid->GetWvel_par().x() << "\n";
        }

        application.EndScene();
    }

    return 0;
}
