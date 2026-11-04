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
//   Demo code about
//     - collisions and constraints using *explicit* lumped-mass timestepper + PSOR
//       constraint stabilization
//     - a second-order explicit integrator (ex Heun 2nd RK) can simulate vibratory
//       phenomena in FEA with good precision, and at the same time non-smooth 
//       contacts are enforced with a projected Gauss-Seidel approach (PSOR) that 
//       is very cheap and stable.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/solver/ChSolverLumpedPSOR.h"  
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;


// Create a bunch of Chrono rigid bodies that represent bricks in a Jenga tower
void create_jengatower_bodies(ChSystemNSC& sys, std::shared_ptr<ChContactMaterialNSC> mat) {

    // Create bricks
    for (int bi = 0; bi < 10; bi += 2) {
        auto mrigidBody1 = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 14,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody1->SetPos(ChVector3d(-5, 1.0 + bi * 2.0, 0));
        sys.Add(mrigidBody1);

        auto mrigidBody2 = chrono_types::make_shared<ChBodyEasyBox>(2, 2, 14,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody2->SetPos(ChVector3d(5, 1.0 + bi * 2.0, 0));
        sys.Add(mrigidBody2);

        auto mrigidBody3 = chrono_types::make_shared<ChBodyEasyBox>(14, 2, 2,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody3->SetPos(ChVector3d(0, 3.0 + bi * 2.0, 5));
        sys.Add(mrigidBody3);

        auto mrigidBody4 = chrono_types::make_shared<ChBodyEasyBox>(14, 2, 2,  // x,y,z size
                                                                    100,       // density
                                                                    true,      // visualization?
                                                                    true,      // collision?
                                                                    mat);      // contact material
        mrigidBody4->SetPos(ChVector3d(0, 3.0 + bi * 2.0, -5));
        sys.Add(mrigidBody4);
    }


    // Create a ball that will collide with tower
    auto mrigidBall = chrono_types::make_shared<ChBodyEasySphere>(2.5,     // radius
                                                                  100,  // density
                                                                  true,  // visualization?
                                                                  true,  // collision?
                                                                  mat);  // contact material
    mrigidBall->SetPos(ChVector3d(0, 2.2, -12));
    //mrigidBall->SetPosDt(ChVector3d(0, 0, 2));  // set initial speed
    mrigidBall->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    sys.Add(mrigidBall);
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the physical system
    ChSystemNSC sys;
    sys.SetGravityY();
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.2);
    ChCollisionModel::SetDefaultSuggestedMargin(0.02);

    // Create a contact material that will be shared among all collision shapes
    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    contact_mat->SetFriction(0.4f);
    contact_mat->SetRestitution(0.3f);
    // contact_mat->SetCompliance(5e-10f);
    // contact_mat->SetComplianceT(5e-10f);
    // contact_mat->SetDampingF(0.001f);


    // Create the floor using fixed rigid body of 'box' type:
    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(250, 4, 250,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visualization?
                                                                true,         // collision?
                                                                contact_mat);         // contact material
    mrigidFloor->SetPos(ChVector3d(0, -2, 0));
    mrigidFloor->SetFixed(true);

    sys.Add(mrigidFloor);

    // Create all the rigid bodies.
    create_jengatower_bodies (sys, contact_mat);

    auto mesh = chrono_types::make_shared<ChMesh>();

    // Create a section, i.e. thickness and material properties for beams
    auto msection = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();
    double beam_wy = 0.9;
    double beam_wz = 0.9;
    msection->SetAsRectangularSection(beam_wy, beam_wz);
    msection->SetYoungModulus(0.01e9);  // soft material, just for test with not so small dt
    msection->SetShearModulus(0.01e9 * 0.3);
    msection->SetRayleighDamping(0.001);


    ChBuilderBeamEuler ebuilder;
    // Create a beam from a point to another
    ebuilder.BuildBeam(mesh,                     // the mesh where to put the created nodes and elements
                      msection,                  // the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                      7,                         // the number of ChElementBeamEuler to create
                      ChVector3d(-6, 20, -4),      // point A (beginning of beam)
                      ChVector3d(12, 20, -4),     // point B (end of beam)
                      ChVector3d(0, 1, 0));      // the 'Y' up direction of the section for the beam

    //ebuilder.GetLastBeamNodes().back()->SetFixed(true);

    // Create the contact surface
    auto contact_cloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(contact_mat);
    contact_cloud->AddAllNodes(*mesh, 0.5);  // use larger point size to approximate beam section radius
    mesh->AddContactSurface(contact_cloud);

    sys.Add(mesh);

    
    auto vis_beam_A = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_A->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    vis_beam_A->SetSmoothFaces(true);
    vis_beam_A->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_beam_A);
    
    mesh->SetAutomaticGravity(true);

    // Create a swinging arm with constraints
    auto arm = chrono_types::make_shared<ChBodyEasyBox>(14, 4, 4,  // x,y,z size
                                                                1000,         // density
                                                                true,         // visualization?
                                                                true,         // collision?
                                                        contact_mat);  // contact material
    arm->SetPos(ChVector3d(-18, 5, 8));
    sys.Add(arm);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(arm, mrigidFloor, ChFrame<>(ChVector3d(-11, 5, 8), Q_ROTATE_Z_TO_Y));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, -0.5));
    sys.Add(motor);



    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Bricks test");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddLight(ChVector3d(70, 120, -90), 290, ChColor(0.7f, 0.7f, 0.7f));
            vis_irr->AddLight(ChVector3d(30, 80, 60), 190, ChColor(0.7f, 0.8f, 0.8f));
            vis_irr->AddCamera(ChVector3d(-15, 14, -30), ChVector3d(0, 5, 0));
            //vis_irr->SetSymbolScale(0.0001f);
            //vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetWindowTitle("VSG Bricks Demo");
            vis_vsg->EnableSkyTexture(SkyMode::BOX);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->AddCamera(ChVector3d(-30, 28, -60), ChVector3d(0, 5, 0));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Prepare the physical system for the simulation
    
    // Typically, with IMPLICIT time steppers, finite elements bring in the tangent
    // stiffness and tangent damping matrices, thus requiring linear system solves.
    // An alternative here...
    // In this demo we want to use an EXPLICIT timestepper with DIAGONAL LUMPED mass,
    // so that each time step does not require the solution of a linear system. Thus
    // each time step is very fast to compute (but the time step must be small enough 
    // to satisfy stability conditions: many cheap small time steps are required).
    // However, how to deal with constraints? 
    // Option a) use ChSolverLumped and turn on a penalty method as in 
    //           stepper->SetConstraintsAsPenaltyON(..)
    //           This would skip constraint jacobian matrices completely, but it can
    //           be difficult to adjust the penalty parameter. Too high: divergence.
    // Option b) use ChSolverLumpedPSOR as in this demo! No constraint penalty is 
    //           used. Constraints are enforced by N simple projected Gauss-Seidel
    //           iterations, with N being typically 1 or 2 to keep the step cost as
    //           low as possible, close to option a). This is more stable and does not
    //           require to adjust any penalty parameter. Another advantage is that
    //           we can still use nonsmooth frictional contacts of ChSystemNSC. 


    // Enable the use of a diagonal mass matrix (lumped mass) for the system
    // by using the ChSolverLumpedPSOR solver.
    auto solver = chrono_types::make_shared<ChSolverLumpedPSOR>();
    solver->SetMaxIterations(1);   // a small n of iterations often is enough!
    solver->EnableWarmStart(true); // this allows the cheap correction to accumulate during the many small explicit timesteps
    sys.SetSolver(solver);

    // Choose one of the various explicit time steppers.  
    //auto estepper = chrono_types::make_shared<ChTimestepperEulerSemiImplicit>(&sys);
    //auto estepper = chrono_types::make_shared<ChTimestepperRungeKutta>(&sys);
    auto estepper = chrono_types::make_shared<ChTimestepperHeun>(&sys);
    //auto estepper = chrono_types::make_shared<ChTimestepperLeapfrog>(&sys); 

    // Use the "projected" explicit solver, that wraps the conventional explicit solver above.
    // This timestepper should be used together with the ChSolverLumpedPSOR solver, see above.
    auto stepper = chrono_types::make_shared<ChTimestepperExplicitProjected>(&sys, estepper);
    stepper->SetMaxPenetrationRecoverySpeed(0.1); // Set max recovery speed here, not in sys.SetMaxPenetrationRecoverySpeed(0.1) as usual. 
    stepper->SetStabilizationFactor(0.8); // A good value is 0.8. Close to 1 means immediate constr. recovery, but may cause oscillations
    sys.SetTimestepper(stepper);

    sys.SetMinBounceSpeed(0.2);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        
        // As always with explicit time steppers, the time step must be very small to avoid divergence. 
        // Possible optimization: perform the vis->BeginScene(); vis->Render(); vis->EndScene(); only every N steps.
        sys.DoStepDynamics(0.001);
    }

    return 0;
}
