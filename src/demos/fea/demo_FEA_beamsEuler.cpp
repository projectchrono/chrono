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
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

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
    msection->SetShearModulus(0.01e9 * 0.3);
    msection->SetRayleighDamping(0.000);
    // msection->SetCentroid(0,0.02);
    // msection->SetShearCenter(0,0.1);
    // msection->SetSectionRotation(45*CH_RAD_TO_DEG);

    // These are for the external loads (define here to help using ChStaticNonLinearIncremental later)
    ChVector3d F_node_1(9, 2, 0);
    ChVector3d F_node_2(0, -2, 0);
    std::shared_ptr<ChNodeFEAxyzrot> loaded_node_1;
    std::shared_ptr<ChNodeFEAxyzrot> loaded_node_2;

    //
    // Add some EULER-BERNOULLI BEAMS:
    //

    double beam_L = 0.1;

    auto hnode1 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(0, 0, 0)));
    auto hnode2 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(beam_L, 0, 0)));
    auto hnode3 = chrono_types::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector3d(beam_L * 2, 0, 0)));

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
    loaded_node_1 = hnode2;
    loaded_node_1->SetForce(F_node_1);

    // Fix a node to ground:
    // hnode1->SetFixed(true);
    auto mtruss = chrono_types::make_shared<ChBody>();
    mtruss->SetFixed(true);
    sys.Add(mtruss);

    auto constr_bc = chrono_types::make_shared<ChLinkMateFix>();
    constr_bc->Initialize(hnode3, mtruss, false, hnode3->Frame(), hnode3->Frame());
    sys.Add(constr_bc);

    auto constr_d = chrono_types::make_shared<ChLinkMateGeneric>();
    constr_d->Initialize(hnode1, mtruss, false, hnode1->Frame(), hnode1->Frame());
    sys.Add(constr_d);
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
    builder.BuildBeam(my_mesh,   // the mesh where to put the created nodes and elements
                      msection,  // the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                      5,         // the number of ChElementBeamEuler to create
                      ChVector3d(0, 0, -0.1),    // the 'A' point in space (beginning of beam)
                      ChVector3d(0.2, 0, -0.1),  // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0));      // the 'Y' up direction of the section for the beam

    // After having used BuildBeam(), you can retrieve the nodes used for the beam.
    // For example say you want to fix the A end and apply a force to the B end:
    builder.GetLastBeamNodes().back()->SetFixed(true);
    loaded_node_2 = builder.GetLastBeamNodes().front();
    loaded_node_2->SetForce(F_node_2);

    // Again, use BuildBeam for creating another beam, this time it uses one node (the last node created by the last
    // beam) and one point:
    builder.BuildBeam(my_mesh, msection, 5,
                      builder.GetLastBeamNodes().front(),  // the 'A' node in space (beginning of beam)
                      ChVector3d(0.2, 0.1, -0.1),          // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0));                // the 'Y' up direction of the section for the beam

    // No gravity effect on FEA elements in this demo
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh asset that is internally managed) by
    // setting  proper coordinates and vertex colors as in the FEM elements. Such a triangle mesh can be rendered by
    // Irrlicht or POVray or whatever postprocessor that can handle a colored ChVisualShapeTriangleMesh).

    /*
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);
    */

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>();
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);

    // Create the run-time visualization system
    auto vis =
        CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Euler Beams", ChVector3d(-0.1, 0.2, -0.2));

    // THE SIMULATION LOOP

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-14);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);
    solver->SetTolerance(1e-14);

    // Change type of integrator
    /*
    auto stepper = chrono_types::make_shared<ChTimestepperHHT>();
    sys.SetTimestepper(stepper);
    stepper->SetAlpha(-0.2);
    stepper->SetMaxIters(6);
    stepper->SetAbsTolerances(1e-12);
    stepper->SetVerbose(true);
    stepper->SetStepControl(false);
    */

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    std::cout << "\n\n===========STATICS======== \n" << std::endl;

    if (false) {
        std::cout << "BEAM RESULTS (LINEAR STATIC ANALYSIS)\n" << std::endl;
        sys.DoStaticLinear();
    }
    if (false) {
        std::cout << "BEAM RESULTS (NON-LINEAR STATIC ANALYSIS, basic)\n" << std::endl;
        sys.DoStaticNonlinear(20);
    }
    if (true) {
        std::cout << "BEAM RESULTS (NON-LINEAR STATIC INCREMENTAL ANALYSIS)\n" << std::endl;

        // Instead of using sys.DoStaticNonLinear(), which is quite basic, we will use ChStaticNonLinearIncremental.
        // This requires a custom callback for incrementing the external loads:

        class MyCallback : public ChStaticNonLinearIncremental::LoadIncrementCallback {
          public:
            // Perform updates on the model. This is called before each load scaling.
            // Here we will update all "external" relevan loads.
            virtual void OnLoadScaling(const double load_scaling,              // ranging from 0 to 1
                                       const int iteration_n,                  // actual number of load step
                                       ChStaticNonLinearIncremental* analysis  // back-pointer to this analysis
            ) {
                // Scale the external loads. In our example, just two forces.
                // Note: if gravity is used, consider scaling also gravity effect, e.g:
                //    sys.SetGravitationalAcceleration(load_scaling * ChVector3d(0,-9.8,0))
                cb_loaded_node_1->SetForce(load_scaling * cb_F_node_1);
                cb_loaded_node_2->SetForce(load_scaling * cb_F_node_2);
            }
            // helper data for the callback
            ChVector3d cb_F_node_1;
            ChVector3d cb_F_node_2;
            std::shared_ptr<ChNodeFEAxyzrot> cb_loaded_node_1;
            std::shared_ptr<ChNodeFEAxyzrot> cb_loaded_node_2;
        };

        // Create the callback object, and set some helper data structures.
        auto my_load_callback = chrono_types::make_shared<MyCallback>();
        my_load_callback->cb_loaded_node_1 = loaded_node_1;
        my_load_callback->cb_loaded_node_2 = loaded_node_2;
        my_load_callback->cb_F_node_1 = F_node_1;
        my_load_callback->cb_F_node_2 = F_node_2;

        // Create the nonlinear static analysis for incremental external loads.
        ChStaticNonLinearIncremental static_analysis;
        static_analysis.SetLoadIncrementCallback(my_load_callback);
        static_analysis.SetVerbose(true);
        // outer loop. More steps helps the inner Newton loop that will need less iterations, but maybe slower.
        static_analysis.SetIncrementalSteps(8);
        // inner loop (Newton iterations). In good situations should converge with 5-20 iterations.
        static_analysis.SetMaxIterationsNewton(20);
        // check Newton monotonicity after 1 step, reduce stepsize if not met.
        static_analysis.SetAdaptiveNewtonON(1, 1.0);
        // slower than default 1.0, but avoids the risk of using too much the adaptive Newton stepsize
        static_analysis.SetNewtonDamping(0.75);
        static_analysis.SetResidualTolerance(1e-7);

        // Do the nonlinear statics.
        sys.DoStaticAnalysis(static_analysis);
    }

    ChVector3d F, M;
    ChVectorDynamic<> displ;

    belement1->GetStateBlock(displ);
    std::cout << displ;
    for (double eta = -1; eta <= 1; eta += 0.4) {
        belement1->EvaluateSectionForceTorque(eta, F, M);
        std::cout << "  b1_at " << eta << " Mx=" << M.x() << " My=" << M.y() << " Mz=" << M.z() << " Tx=" << F.x()
                  << " Ty=" << F.y() << " Tz=" << F.z() << std::endl;
    }
    std::cout << std::endl;
    belement2->GetStateBlock(displ);
    for (double eta = -1; eta <= 1; eta += 0.4) {
        belement2->EvaluateSectionForceTorque(eta, F, M);
        std::cout << "  b2_at " << eta << " Mx=" << M.x() << " My=" << M.y() << " Mz=" << M.z() << " Tx=" << F.x()
                  << " Ty=" << F.y() << " Tz=" << F.z() << std::endl;
    }

    std::cout << "Node 3 coordinate x= " << hnode3->Frame().GetPos().x() << "    y=" << hnode3->Frame().GetPos().y()
              << "    z=" << hnode3->Frame().GetPos().z() << std::endl
              << std::endl;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.001);
    }

    return 0;
}
