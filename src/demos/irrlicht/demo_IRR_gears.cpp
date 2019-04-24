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

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Gears and pulleys", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(12, 15, -20));

    // Create all the rigid bodies.

    double radA = 2;
    double radB = 4;

    // ...the truss
    auto mbody_truss = std::make_shared<ChBodyEasyBox>(20, 10, 2, 1000, false, true);
    mphysicalSystem.Add(mbody_truss);
    mbody_truss->SetBodyFixed(true);
    mbody_truss->SetPos(ChVector<>(0, 0, 3));

    // ...a texture asset that will be shared among the four wheels
    auto cylinder_texture = std::make_shared<ChTexture>(GetChronoDataFile("pinkwhite.png"));

    // ...the rotating bar support for the two epicycloidal wheels
    auto mbody_train = std::make_shared<ChBodyEasyBox>(8, 1.5, 1.0, 1000, false, true);
    mphysicalSystem.Add(mbody_train);
    mbody_train->SetPos(ChVector<>(3, 0, 0));

    // ...which must rotate respect to truss along Z axis, in 0,0,0,
    auto link_revoluteTT = std::make_shared<ChLinkLockRevolute>();
    link_revoluteTT->Initialize(mbody_truss, mbody_train,
                                ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
    mphysicalSystem.AddLink(link_revoluteTT);

    // ...the first gear
    auto mbody_gearA = std::make_shared<ChBodyEasyCylinder>(radA, 0.5, 1000, false, true);
    mphysicalSystem.Add(mbody_gearA);
    mbody_gearA->SetPos(ChVector<>(0, 0, -1));
    mbody_gearA->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    mbody_gearA->AddAsset(cylinder_texture);
    // for aesthetic reasons, also add a thin cylinder only as a visualization:
    auto mshaft_shape = std::make_shared<ChCylinderShape>();
    mshaft_shape->GetCylinderGeometry().p1 = ChVector<>(0,-3,0);
    mshaft_shape->GetCylinderGeometry().p2 = ChVector<>(0, 10,0);
    mshaft_shape->GetCylinderGeometry().rad = radA*0.4;
    mbody_gearA->AddAsset(mshaft_shape);

    // ...impose rotation speed between the first gear and the fixed truss
    auto link_motor = std::make_shared<ChLinkMotorRotationSpeed>();
    link_motor->Initialize(mbody_gearA, mbody_truss, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    link_motor->SetSpeedFunction(std::make_shared<ChFunction_Const>(6));
    mphysicalSystem.AddLink(link_motor);

    // ...the second gear
    double interaxis12 = radA + radB;
    auto mbody_gearB = std::make_shared<ChBodyEasyCylinder>(radB, 0.4, 1000, false, true);
    mphysicalSystem.Add(mbody_gearB);
    mbody_gearB->SetPos(ChVector<>(interaxis12, 0, -1));
    mbody_gearB->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    mbody_gearB->AddAsset(cylinder_texture);

    // ... the second gear is fixed to the rotating bar
    auto link_revolute = std::make_shared<ChLinkLockRevolute>();
    link_revolute->Initialize(mbody_gearB, mbody_train,
                              ChCoordsys<>(ChVector<>(interaxis12, 0, 0), QUNIT));
    mphysicalSystem.AddLink(link_revolute);

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
    auto link_gearAB = std::make_shared<ChLinkGear>();
    link_gearAB->Initialize(mbody_gearA, mbody_gearB, CSYSNORM);
    link_gearAB->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAB->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAB->Set_tau(radA / radB);
    link_gearAB->Set_checkphase(true);
    mphysicalSystem.AddLink(link_gearAB);

    // ...the gear constraint between the second wheel B and a large wheel C with inner teeth, that
    //    does not necessarily need to be created as a new body because it is the 'fixed' part of the
    //    epicycloidal reducer, so, as wheel C, we will simply use the ground object 'mbody_truss'.
    double radC = 2 * radB + radA;
    auto link_gearBC = std::make_shared<ChLinkGear>();
    link_gearBC->Initialize(mbody_gearB, mbody_truss, CSYSNORM);
    link_gearBC->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearBC->Set_local_shaft2(ChFrame<>(ChVector<>(0, 0, -4), QUNIT));
    link_gearBC->Set_tau(radB / radC);
    link_gearBC->Set_epicyclic(true);  // <-- this means: use a wheel with internal teeth!
    mphysicalSystem.AddLink(link_gearBC);

    // ...the bevel gear at the side,
    double radD = 5;
    auto mbody_gearD = std::make_shared<ChBodyEasyCylinder>(radD, 0.8, 1000, false, true);
    mphysicalSystem.Add(mbody_gearD);
    mbody_gearD->SetPos(ChVector<>(-10, 0, -9));
    mbody_gearD->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
    mbody_gearD->AddAsset(cylinder_texture);

    // ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate
    //     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
    auto link_revoluteD = std::make_shared<ChLinkLockRevolute>();
    link_revoluteD->Initialize(mbody_gearD, mbody_truss,
                               ChCoordsys<>(ChVector<>(-10, 0, -9), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    mphysicalSystem.AddLink(link_revoluteD);

    // ... Let's make a 1:1 gear between wheel A and wheel D as a bevel gear: Chrono does not require
    //     special info for this case -the position of the two shafts and the transmission ratio are enough-
    auto link_gearAD = std::make_shared<ChLinkGear>();
    link_gearAD->Initialize(mbody_gearA, mbody_gearD, CSYSNORM);
    link_gearAD->Set_local_shaft1(ChFrame<>(ChVector<>(0, -7, 0), chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAD->Set_local_shaft2(ChFrame<>(ChVector<>(0, -7, 0), chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_gearAD->Set_tau(1);
    mphysicalSystem.AddLink(link_gearAD);

    // ...the pulley at the side,
    double radE = 2;
    auto mbody_pulleyE = std::make_shared<ChBodyEasyCylinder>(radE, 0.8, 1000, false, true);
    mphysicalSystem.Add(mbody_pulleyE);
    mbody_pulleyE->SetPos(ChVector<>(-10, -11, -9));
    mbody_pulleyE->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
    mbody_pulleyE->AddAsset(cylinder_texture);

    // ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate
    //     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
    auto link_revoluteE = std::make_shared<ChLinkLockRevolute>();
    link_revoluteE->Initialize(mbody_pulleyE, mbody_truss,
                               ChCoordsys<>(ChVector<>(-10, -11, -9), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
    mphysicalSystem.AddLink(link_revoluteE);

    // ... Let's make a synchro belt constraint between pulley D and pulley E. The user must be
    //     sure that the two shafts are parallel in absolute space. Also, interaxial distance should not change.
    auto link_pulleyDE = std::make_shared<ChLinkPulley>();
    link_pulleyDE->Initialize(mbody_gearD, mbody_pulleyE, CSYSNORM);
    link_pulleyDE->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_pulleyDE->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI / 2, VECT_X)));
    link_pulleyDE->Set_r1(radD);
    link_pulleyDE->Set_r2(radE);
    link_pulleyDE->Set_checkphase(true); // synchro belts don't tolerate slipping: this avoids it as numerical errors accumulate.
    mphysicalSystem.AddLink(link_pulleyDE);


    
    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Prepare the physical system for the simulation

    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_PROJECTED);

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    application.SetStepManage(true);
    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        // .. draw also some circle lines representing gears - just for aesthetical reasons

        ChIrrTools::drawCircle(application.GetVideoDriver(), link_gearBC->Get_r2(),
                               (link_gearBC->Get_local_shaft2() >> *link_gearBC->GetBody2()).GetCoord(),
                               video::SColor(255, 255, 0, 0), 50, true);
        ChIrrTools::drawCircle(application.GetVideoDriver(), link_gearAD->Get_r1(),
                               (link_gearAD->Get_local_shaft1() >> *link_gearAD->GetBody1()).GetCoord(),
                               video::SColor(255, 255, 0, 0), 30, true);
        ChIrrTools::drawCircle(application.GetVideoDriver(), link_gearAD->Get_r2(),
                               (link_gearAD->Get_local_shaft2() >> *link_gearAD->GetBody2()).GetCoord(),
                               video::SColor(255, 255, 0, 0), 30, true);

        ChIrrTools::drawCircle(application.GetVideoDriver(), 0.1,
                               ChCoordsys<>(link_gearAB->GetMarker2()->GetAbsCoord().pos, QUNIT));
        ChIrrTools::drawCircle(application.GetVideoDriver(), 0.1,
                               ChCoordsys<>(link_gearAD->GetMarker2()->GetAbsCoord().pos, QUNIT));
        ChIrrTools::drawCircle(application.GetVideoDriver(), 0.1,
                               ChCoordsys<>(link_gearBC->GetMarker2()->GetAbsCoord().pos, QUNIT));

        // ..draw also some segments for a simplified representation of pulley
        ChIrrTools::drawSegment(application.GetVideoDriver(), link_pulleyDE->Get_belt_up1(),
                                link_pulleyDE->Get_belt_up2(), video::SColor(255, 0, 255, 0), true);
        ChIrrTools::drawSegment(application.GetVideoDriver(), link_pulleyDE->Get_belt_low1(),
                                link_pulleyDE->Get_belt_low2(), video::SColor(255, 0, 255, 0), true);

        // ADVANCE THE SIMULATION FOR ONE STEP

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
