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
// Demonstration of the gear constraint (ChLinkGear) as a method to impose a
// transmission ratio between two shafts as they were connected by gears,
// without the need of performing collision detection between gear teeth
// geometries (which would be inefficient)
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Contact material shared among all bodies
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Shared visualization material
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetKdTexture(GetChronoDataFile("textures/pinkwhite.png"));

    // Create all the rigid bodies.

    double radA = 2;
    double radB = 4;

    // ...the truss
    auto mbody_truss = chrono_types::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, true, false, mat);
    sys.Add(mbody_truss);
    mbody_truss->SetBodyFixed(true);
    mbody_truss->SetPos(ChVector<>(0, 0, 3));

    // ...the rotating bar support for the two epicycloidal wheels
    auto mbody_train = chrono_types::make_shared<ChBodyEasyBox>(8, 1.5, 1.0, 1000, true, false, mat);
    sys.Add(mbody_train);
    mbody_train->SetPos(ChVector<>(3, 0, 0));

    // ...which must rotate respect to truss along Z axis, in 0,0,0,
    auto link_revoluteTT = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revoluteTT->Initialize(mbody_truss, mbody_train, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    sys.AddLink(link_revoluteTT);

    // ...the first gear
    auto mbody_gearA =
        chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radA, 0.5, 1000, true, false, mat);
    sys.Add(mbody_gearA);
    mbody_gearA->SetPos(ChVector<>(0, 0, -1));
    mbody_gearA->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    // for aesthetic reasons, also add a thin cylinder only as a visualization:
    auto mshaft_shape = chrono_types::make_shared<ChCylinderShape>(radA * 0.4, 13);
    mbody_gearA->AddVisualShape(mshaft_shape, ChFrame<>(ChVector<>(0, 3.5, 0), Q_from_AngX(CH_C_PI_2)));

    // ...impose rotation speed between the first gear and the fixed truss
    auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motor->Initialize(mbody_gearA, mbody_truss, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    link_motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(6));
    sys.AddLink(link_motor);

    // ...the second gear
    double interaxis12 = radA + radB;
    auto mbody_gearB =
        chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radB, 0.4, 1000, true, false, mat);
    sys.Add(mbody_gearB);
    mbody_gearB->SetPos(ChVector<>(interaxis12, 0, -1));
    mbody_gearB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    mbody_gearB->GetVisualShape(0)->SetMaterial(0, vis_mat);

    // ... the second gear is fixed to the rotating bar
    auto link_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revolute->Initialize(mbody_gearB, mbody_train, ChCoordsys<>(ChVector<>(interaxis12, 0, 0), QUNIT));
    sys.AddLink(link_revolute);

    // ...the gear constraint between the two wheels A and B.
    //    As transmission ratio (=speed of wheel B / speed of wheel A) to enter in  Set_tau(), we
    //    could use whatever positive value we want: the ChLinkGear will compute the two radii of the
    //    wheels for its 'hidden' computations, given the distance between the two axes. However, since
    //    we already build two '3D cylinders' bodies -just for visualization reasons!- with radA and radB,
    //    we must enter Set_tau(radA/radB).
    //    Also, note that the initial position of the constraint has no importance (simply use CSYSNORM),
    //    but we must set where the two axes are placed in the local coordinates of the two wheels, so
    //    we use Set_local_shaft1() and pass some local ChFrame. Note that, since the Z axis of that frame
    //    will be considered the axis of the wheel, we must rotate the frame 90° with Q_from_AngAxis(), because
    //    we created the wheel with ChBodyEasyCylinder() which created a cylinder with Y as axis.
    auto link_gearAB = chrono_types::make_shared<ChLinkGear>();
    link_gearAB->Initialize(mbody_gearA, mbody_gearB, CSYSNORM);
    link_gearAB->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAB->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAB->Set_tau(radA / radB);
    link_gearAB->Set_checkphase(true);
    sys.AddLink(link_gearAB);

    // ...the gear constraint between the second wheel B and a large wheel C with inner teeth, that
    //    does not necessarily need to be created as a new body because it is the 'fixed' part of the
    //    epicycloidal reducer, so, as wheel C, we will simply use the ground object 'mbody_truss'.
    double radC = 2 * radB + radA;
    auto link_gearBC = chrono_types::make_shared<ChLinkGear>();
    link_gearBC->Initialize(mbody_gearB, mbody_truss, CSYSNORM);
    link_gearBC->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearBC->Set_local_shaft2(ChFrame<>(ChVector<>(0, 0, -4), QUNIT));
    link_gearBC->Set_tau(radB / radC);
    link_gearBC->Set_epicyclic(true);  // <-- this means: use a wheel with internal teeth!
    sys.AddLink(link_gearBC);

    // ...the bevel gear at the side,
    double radD = 5;
    auto mbody_gearD =
        chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radD, 0.8, 1000, true, false, mat);
    sys.Add(mbody_gearD);
    mbody_gearD->SetPos(ChVector<>(-10, 0, -9));
    mbody_gearD->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
    mbody_gearD->GetVisualShape(0)->SetMaterial(0, vis_mat);

    // ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate
    //     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
    auto link_revoluteD = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revoluteD->Initialize(mbody_gearD, mbody_truss,
                               ChCoordsys<>(ChVector<>(-10, 0, -9), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    sys.AddLink(link_revoluteD);

    // ... Let's make a 1:1 gear between wheel A and wheel D as a bevel gear: Chrono does not require
    //     special info for this case -the position of the two shafts and the transmission ratio are enough-
    auto link_gearAD = chrono_types::make_shared<ChLinkGear>();
    link_gearAD->Initialize(mbody_gearA, mbody_gearD, CSYSNORM);
    link_gearAD->Set_local_shaft1(ChFrame<>(ChVector<>(0, -7, 0), chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAD->Set_local_shaft2(ChFrame<>(ChVector<>(0, -7, 0), chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAD->Set_tau(1);
    sys.AddLink(link_gearAD);

    // ...the pulley at the side,
    double radE = 2;
    auto mbody_pulleyE =
        chrono_types::make_shared<ChBodyEasyCylinder>(geometry::ChAxis::Y, radE, 0.8, 1000, true, false, mat);
    sys.Add(mbody_pulleyE);
    mbody_pulleyE->SetPos(ChVector<>(-10, -11, -9));
    mbody_pulleyE->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
    mbody_pulleyE->GetVisualShape(0)->SetMaterial(0, vis_mat);

    // ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate
    //     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
    auto link_revoluteE = chrono_types::make_shared<ChLinkLockRevolute>();
    link_revoluteE->Initialize(mbody_pulleyE, mbody_truss,
                               ChCoordsys<>(ChVector<>(-10, -11, -9), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    sys.AddLink(link_revoluteE);

    // ... Let's make a synchro belt constraint between pulley D and pulley E. The user must be
    //     sure that the two shafts are parallel in absolute space. Also, interaxial distance should not change.
    auto link_pulleyDE = chrono_types::make_shared<ChLinkPulley>();
    link_pulleyDE->Initialize(mbody_gearD, mbody_pulleyE, CSYSNORM);
    link_pulleyDE->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_pulleyDE->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_pulleyDE->Set_r1(radD);
    link_pulleyDE->Set_r2(radE);
    link_pulleyDE->Set_checkphase(
        true);  // synchro belts don't tolerate slipping: this avoids it as numerical errors accumulate.
    sys.AddLink(link_pulleyDE);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Gears and pulleys");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(12, 15, -20));
    vis->AddTypicalLights();

    // Prepare the physical system for the simulation

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    // Simulation loop

    double timestep = 0.001;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();

        // .. draw also some circle lines representing gears - just for aesthetical reasons

        tools::drawCircle(vis.get(), link_gearBC->Get_r2(),
                          (link_gearBC->Get_local_shaft2() >> *link_gearBC->GetBody2()).GetCoord(), ChColor(1, 0, 0),
                          50, true);
        tools::drawCircle(vis.get(), link_gearAD->Get_r1(),
                          (link_gearAD->Get_local_shaft1() >> *link_gearAD->GetBody1()).GetCoord(), ChColor(1, 0, 0),
                          30, true);
        tools::drawCircle(vis.get(), link_gearAD->Get_r2(),
                          (link_gearAD->Get_local_shaft2() >> *link_gearAD->GetBody2()).GetCoord(), ChColor(1, 0, 0),
                          30, true);

        tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(link_gearAB->GetMarker2()->GetAbsCoord().pos, QUNIT));
        tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(link_gearAD->GetMarker2()->GetAbsCoord().pos, QUNIT));
        tools::drawCircle(vis.get(), 0.1, ChCoordsys<>(link_gearBC->GetMarker2()->GetAbsCoord().pos, QUNIT));

        // ..draw also some segments for a simplified representation of pulley
        tools::drawSegment(vis.get(), link_pulleyDE->Get_belt_up1(), link_pulleyDE->Get_belt_up2(), ChColor(0, 1, 0),
                           true);
        tools::drawSegment(vis.get(), link_pulleyDE->Get_belt_low1(), link_pulleyDE->Get_belt_low2(), ChColor(0, 1, 0),
                           true);

        vis->EndScene();

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
