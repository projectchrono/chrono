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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Beams (SPACE for dynamics, F10 / F11 statics)", core::dimension2d<u32>(800, 600),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.

    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

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
    // Add some EULER-BERNOULLI BEAMS:
    //

    double beam_L = 0.1;

    auto hnode1 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(0, 0, 0)));
    auto hnode2 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L, 0, 0)));
    auto hnode3 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 2, 0, 0)));

    my_mesh->AddNode(hnode1);
    my_mesh->AddNode(hnode2);
    my_mesh->AddNode(hnode3);

    auto belement1 = chrono_types::make_shared<ChElementBeamEuler>();

    belement1->SetNodes(hnode1, hnode2);
    belement1->SetSection(msection);

    my_mesh->AddElement(belement1);

    auto belement2 = chrono_types::make_shared<ChElementBeamEuler>();

    belement2->SetNodes(hnode2, hnode3);
    belement2->SetSection(msection);

    my_mesh->AddElement(belement2);

    // Apply a force or a torque to a node:
    hnode2->SetForce(ChVector<>(4, 2, 0));
    // hnode3->SetTorque( ChVector<>(0, -0.04, 0));

    // Fix a node to ground:
    // hnode1->SetFixed(true);
    auto mtruss = chrono_types::make_shared<ChBody>();
    mtruss->SetBodyFixed(true);
    my_system.Add(mtruss);

    auto constr_bc = chrono_types::make_shared<ChLinkMateGeneric>();
    constr_bc->Initialize(hnode3, mtruss, false, hnode3->Frame(), hnode3->Frame());
    my_system.Add(constr_bc);
    constr_bc->SetConstrainedCoords(true, true, true,   // x, y, z
                                    true, true, true);  // Rx, Ry, Rz

    auto constr_d = chrono_types::make_shared<ChLinkMateGeneric>();
    constr_d->Initialize(hnode1, mtruss, false, hnode1->Frame(), hnode1->Frame());
    my_system.Add(constr_d);
    constr_d->SetConstrainedCoords(false, true, true,     // x, y, z
                                   false, false, false);  // Rx, Ry, Rz

    //
    // Add some EULER-BERNOULLI BEAMS (the fast way!)
    //

    // Shortcut!
    // This ChBuilderBeamEuler helper object is very useful because it will
    // subdivide 'beams' into sequences of finite elements of beam type, ex.
    // one 'beam' could be made of 5 FEM elements of ChElementBeamEuler class.
    // If new nodes are needed, it will create them for you.
    ChBuilderBeamEuler builder;

    // Now, simply use BuildBeam to create a beam from a point to another:
    builder.BuildBeam(my_mesh,                   // the mesh where to put the created nodes and elements
                      msection,                  // the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                      5,                         // the number of ChElementBeamEuler to create
                      ChVector<>(0, 0, -0.1),    // the 'A' point in space (beginning of beam)
                      ChVector<>(0.2, 0, -0.1),  // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0));      // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam,
    // For example say you want to fix the A end and apply a force to the B end:
    builder.GetLastBeamNodes().back()->SetFixed(true);
    builder.GetLastBeamNodes().front()->SetForce(ChVector<>(0, -1, 0));

    // Again, use BuildBeam for creating another beam, this time
    // it uses one node (the last node created by the last beam) and one point:
    builder.BuildBeam(my_mesh, msection, 5,
                      builder.GetLastBeamNodes().front(),  // the 'A' node in space (beginning of beam)
                      ChVector<>(0.2, 0.1, -0.1),          // the 'B' point in space (end of beam)
                      ChVector<>(0, 1, 0));                // the 'Y' up direction of the section for the beam

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
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);
    */

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
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

    // THE SIMULATION LOOP

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-14);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);

    my_system.SetSolverForceTolerance(1e-13);

    // Change type of integrator:
    ////auto stepper = chrono_types::make_shared<ChTimestepperHHT>();
    ////my_system.SetTimestepper(stepper);
    ////stepper->SetAlpha(-0.2);
    ////stepper->SetMaxiters(6);
    ////stepper->SetAbsTolerances(1e-12);
    ////stepper->SetVerbose(true);
    ////stepper->SetStepControl(false);

    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    application.SetTimestep(0.001);

    GetLog() << "\n\n\n===========STATICS======== \n\n\n";

    application.GetSystem()->DoStaticLinear();

    GetLog() << "BEAM RESULTS (LINEAR STATIC ANALYSIS) \n\n";

    ChVector<> F, M;
    ChVectorDynamic<> displ;

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

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
