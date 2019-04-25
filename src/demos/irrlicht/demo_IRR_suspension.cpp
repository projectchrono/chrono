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
// Demo code about
// - modeling a complex mechanism (a quarter car model)
// - using the ChLinkSpring to make spring-damper system
// - using the ChLinkDistance class to represent long and thin massless rods,
//   whose mass is negligible for dynamical analysis (as often happens in
//   mechanisms) so they can be modeled as 'distance' constraints instead of
//   making a thin body with small mass and two spherical joints at the end
//   (wihch would be much less efficient from the computational point of view).
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChSystemNSC.h"

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

// First of all, define a class for the 'car' (that is, a set of
// bodies and links which are grouped within this class; so it is
// easier to manage data structures in this example).

class MySimpleCar {
  public:
    // THE DATA

    double throttle;          // actual value 0...1 of gas throttle.
    double conic_tau;         // the transmission ratio of the conic gears at the rear axle
    double gear_tau;          // the actual tau of the gear
    double max_motor_torque;  // the max torque of the motor [Nm];
    double max_motor_speed;   // the max rotation speed of the motor [rads/s]

    // .. chassis:
    std::shared_ptr<ChBody> chassis;
    // .. right front suspension:
    std::shared_ptr<ChBody> spindleRF;
    std::shared_ptr<ChBody> wheelRF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteRF;
    std::shared_ptr<ChLinkDistance> link_distRFU1;
    std::shared_ptr<ChLinkDistance> link_distRFU2;
    std::shared_ptr<ChLinkDistance> link_distRFL1;
    std::shared_ptr<ChLinkDistance> link_distRFL2;
    std::shared_ptr<ChLinkSpring> link_springRF;
    std::shared_ptr<ChLinkDistance> link_distRSTEER;
    // .. left front suspension:
    std::shared_ptr<ChBody> spindleLF;
    std::shared_ptr<ChBody> wheelLF;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteLF;
    std::shared_ptr<ChLinkDistance> link_distLFU1;
    std::shared_ptr<ChLinkDistance> link_distLFU2;
    std::shared_ptr<ChLinkDistance> link_distLFL1;
    std::shared_ptr<ChLinkDistance> link_distLFL2;
    std::shared_ptr<ChLinkSpring> link_springLF;
    std::shared_ptr<ChLinkDistance> link_distLSTEER;
    // .. right back suspension:
    std::shared_ptr<ChBody> spindleRB;
    std::shared_ptr<ChBody> wheelRB;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteRB;
    std::shared_ptr<ChLinkDistance> link_distRBU1;
    std::shared_ptr<ChLinkDistance> link_distRBU2;
    std::shared_ptr<ChLinkDistance> link_distRBL1;
    std::shared_ptr<ChLinkDistance> link_distRBL2;
    std::shared_ptr<ChLinkSpring> link_springRB;
    std::shared_ptr<ChLinkDistance> link_distRBlat;
    std::shared_ptr<ChLinkMotorRotationTorque> link_motorL;
    // .. left back suspension:
    std::shared_ptr<ChBody> spindleLB;
    std::shared_ptr<ChBody> wheelLB;
    std::shared_ptr<ChLinkLockRevolute> link_revoluteLB;
    std::shared_ptr<ChLinkDistance> link_distLBU1;
    std::shared_ptr<ChLinkDistance> link_distLBU2;
    std::shared_ptr<ChLinkDistance> link_distLBL1;
    std::shared_ptr<ChLinkDistance> link_distLBL2;
    std::shared_ptr<ChLinkSpring> link_springLB;
    std::shared_ptr<ChLinkDistance> link_distLBlat;
    std::shared_ptr<ChLinkMotorRotationTorque> link_motorR;

    // THE FUNCTIONS

    // Build and initialize the car, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
    MySimpleCar(ChSystemNSC& my_system,        ///< the Chrono physical system
                ISceneManager* msceneManager,  ///< the Irrlicht scene manager for 3d shapes
                IVideoDriver* mdriver          ///< the Irrlicht video driver
    ) {
        throttle = 0;  // initially, gas throttle is 0.
        conic_tau = 0.2;
        gear_tau = 0.3;
        max_motor_torque = 80;
        max_motor_speed = 800;

        // Texture for wheels
        auto texture = std::make_shared<ChTexture>();
        texture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));

        // --- The car body ---
        chassis = std::make_shared<ChBodyEasyBox>(1, 0.5, 3, 1.0, true, true);
        chassis->SetPos(ChVector<>(0, 1, 0));
        chassis->SetMass(150);
        chassis->SetInertiaXX(ChVector<>(4.8, 4.5, 1));
        chassis->SetBodyFixed(false);
        my_system.AddBody(chassis);

        // --- Right Front suspension ---

        // ..the car right-front spindle
        spindleRF = std::make_shared<ChBodyEasyBox>(0.1, 0.4, 0.4, 1.0, false, true);
        spindleRF->SetPos(ChVector<>(1.3, 1, 1));
        spindleRF->SetMass(8);
        spindleRF->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        my_system.AddBody(spindleRF);

        // ..the car right-front wheel
        wheelRF = std::make_shared<ChBodyEasyCylinder>(0.45, 0.3, 1.0, true, true);
        wheelRF->SetPos(ChVector<>(1.5, 1, 1));
        wheelRF->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
        wheelRF->SetMass(3);
        wheelRF->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        wheelRF->GetMaterialSurfaceNSC()->SetFriction(1.0);
        wheelRF->AddAsset(texture);
        my_system.AddBody(wheelRF);

        // .. create the revolute joint between the wheel and the spindle
        link_revoluteRF = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
        link_revoluteRF->Initialize(wheelRF, spindleRF,
                                    ChCoordsys<>(ChVector<>(1.5, 1, 1), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        my_system.AddLink(link_revoluteRF);

        // .. impose distance between two parts (as a massless rod with two spherical joints at the end)
        link_distRFU1 = std::make_shared<ChLinkDistance>();  // right, front, upper, 1
        link_distRFU1->Initialize(chassis, spindleRF, false, ChVector<>(0.5, 1.2, 1.2), ChVector<>(1.25, 1.2, 1));
        my_system.AddLink(link_distRFU1);

        link_distRFU2 = std::make_shared<ChLinkDistance>();  // right, front, upper, 2
        link_distRFU2->Initialize(chassis, spindleRF, false, ChVector<>(0.5, 1.2, 0.8), ChVector<>(1.25, 1.2, 1));
        my_system.AddLink(link_distRFU2);

        link_distRFL1 = std::make_shared<ChLinkDistance>();  // right, front, lower, 1
        link_distRFL1->Initialize(chassis, spindleRF, false, ChVector<>(0.5, 0.8, 1.2), ChVector<>(1.25, 0.8, 1));
        my_system.AddLink(link_distRFL1);

        link_distRFL2 = std::make_shared<ChLinkDistance>();  // right, front, lower, 2
        link_distRFL2->Initialize(chassis, spindleRF, false, ChVector<>(0.5, 0.8, 0.8), ChVector<>(1.25, 0.8, 1));
        my_system.AddLink(link_distRFL2);

        // .. create the spring between the truss and the spindle
        link_springRF = std::make_shared<ChLinkSpring>();
        link_springRF->Initialize(chassis, spindleRF, false, ChVector<>(0.5, 1.2, 1.0), ChVector<>(1.25, 0.8, 1));
        link_springRF->Set_SpringK(28300);
        link_springRF->Set_SpringR(80);
        my_system.AddLink(link_springRF);

        // .. create the rod for steering the wheel
        link_distRSTEER = std::make_shared<ChLinkDistance>();  // right steer
        link_distRSTEER->Initialize(chassis, spindleRF, false, ChVector<>(0.5, 1.21, 1.4), ChVector<>(1.25, 1.21, 1.3));
        my_system.AddLink(link_distRSTEER);

        // --- Left Front suspension ---

        // ..the car right-front spindle
        spindleLF = std::make_shared<ChBodyEasyBox>(0.1, 0.4, 0.4, 1.0, false, true);
        spindleLF->SetPos(ChVector<>(-1.3, 1, 1));
        spindleLF->SetMass(8);
        spindleLF->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        my_system.AddBody(spindleLF);

        // ..the car left-front wheel
        wheelLF = std::make_shared<ChBodyEasyCylinder>(0.45, 0.3, 1.0, true, true);
        wheelLF->SetPos(ChVector<>(-1.5, 1, 1));
        wheelLF->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
        wheelLF->SetMass(3);
        wheelLF->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        wheelLF->GetMaterialSurfaceNSC()->SetFriction(1.0);
        wheelLF->AddAsset(texture);
        my_system.AddBody(wheelLF);

        // .. create the revolute joint between the wheel and the spindle
        link_revoluteLF = std::make_shared<ChLinkLockRevolute>();  // left, front, upper, 1
        link_revoluteLF->Initialize(wheelLF, spindleLF,
                                    ChCoordsys<>(ChVector<>(-1.5, 1, 1), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        my_system.AddLink(link_revoluteLF);

        // .. impose distance between two parts (as a massless rod with two spherical joints at the end)
        link_distLFU1 = std::make_shared<ChLinkDistance>();  // left, front, upper, 1
        link_distLFU1->Initialize(chassis, spindleLF, false, ChVector<>(-0.5, 1.2, 1.2), ChVector<>(-1.25, 1.2, 1));
        my_system.AddLink(link_distLFU1);

        link_distLFU2 = std::make_shared<ChLinkDistance>();  // left, front, upper, 2
        link_distLFU2->Initialize(chassis, spindleLF, false, ChVector<>(-0.5, 1.2, 0.8), ChVector<>(-1.25, 1.2, 1));
        my_system.AddLink(link_distLFU2);

        link_distLFL1 = std::make_shared<ChLinkDistance>();  // left, front, lower, 1
        link_distLFL1->Initialize(chassis, spindleLF, false, ChVector<>(-0.5, 0.8, 1.2), ChVector<>(-1.25, 0.8, 1));
        my_system.AddLink(link_distLFL1);

        link_distLFL2 = std::make_shared<ChLinkDistance>();  // left, front, lower, 2
        link_distLFL2->Initialize(chassis, spindleLF, false, ChVector<>(-0.5, 0.8, 0.8), ChVector<>(-1.25, 0.8, 1));
        my_system.AddLink(link_distLFL2);

        // .. create the spring between the truss and the spindle
        link_springLF = std::make_shared<ChLinkSpring>();
        link_springLF->Initialize(chassis, spindleLF, false, ChVector<>(-0.5, 1.2, 1.0), ChVector<>(-1.25, 0.8, 1));
        link_springLF->Set_SpringK(28300);
        link_springLF->Set_SpringR(80);
        my_system.AddLink(link_springLF);

        // .. create the rod for steering the wheel
        link_distLSTEER = std::make_shared<ChLinkDistance>();  // right steer
        link_distLSTEER->Initialize(chassis, spindleLF, false, ChVector<>(-0.5, 1.21, 1.4),
                                    ChVector<>(-1.25, 1.21, 1.3));
        my_system.AddLink(link_distLSTEER);

        // --- Right Back suspension ---

        // ..the car right-back spindle
        spindleRB = std::make_shared<ChBodyEasyBox>(0.1, 0.4, 0.4, 1.0, false, true);
        spindleRB->SetPos(ChVector<>(1.3, 1, -1));
        spindleRB->SetMass(8);
        spindleRB->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        my_system.AddBody(spindleRB);

        // ..the car right-back wheel
        wheelRB = std::make_shared<ChBodyEasyCylinder>(0.45, 0.3, 1.0, true, true);
        wheelRB->SetPos(ChVector<>(1.5, 1, -1));
        wheelRB->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
        wheelRB->SetMass(3);
        wheelRB->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        wheelRB->GetMaterialSurfaceNSC()->SetFriction(1.0);
        wheelRB->AddAsset(texture);
        my_system.AddBody(wheelRB);

        // .. create the revolute joint between the wheel and the spindle
        link_revoluteRB = std::make_shared<ChLinkLockRevolute>();  // right, back, upper, 1
        link_revoluteRB->Initialize(wheelRB, spindleRB,
                                    ChCoordsys<>(ChVector<>(1.5, 1, -1), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        my_system.AddLink(link_revoluteRB);

        // .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
        link_motorR = std::make_shared<ChLinkMotorRotationTorque>();
        link_motorR->Initialize(wheelRB, chassis,
                                ChFrame<>(ChVector<>(1.5, 1, -1), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        my_system.AddLink(link_motorR);

        // .. impose distance between two parts (as a massless rod with two spherical joints at the end)
        link_distRBU1 = std::make_shared<ChLinkDistance>();  // right, back, upper, 1
        link_distRBU1->Initialize(chassis, spindleRB, false, ChVector<>(0.5, 1.2, -1.2), ChVector<>(1.25, 1.2, -1));
        my_system.AddLink(link_distRBU1);

        link_distRBU2 = std::make_shared<ChLinkDistance>();  // right, back, upper, 2
        link_distRBU2->Initialize(chassis, spindleRB, false, ChVector<>(0.5, 1.2, -0.8), ChVector<>(1.25, 1.2, -1));
        my_system.AddLink(link_distRBU2);

        link_distRBL1 = std::make_shared<ChLinkDistance>();  // right, back, lower, 1
        link_distRBL1->Initialize(chassis, spindleRB, false, ChVector<>(0.5, 0.8, -1.2), ChVector<>(1.25, 0.8, -1));
        my_system.AddLink(link_distRBL1);

        link_distRBL2 = std::make_shared<ChLinkDistance>();  // right, back, lower, 2
        link_distRBL2->Initialize(chassis, spindleRB, false, ChVector<>(0.5, 0.8, -0.8), ChVector<>(1.25, 0.8, -1));
        my_system.AddLink(link_distRBL2);

        // .. create the spring between the truss and the spindle
        link_springRB = std::make_shared<ChLinkSpring>();
        link_springRB->Initialize(chassis, spindleRB, false, ChVector<>(0.5, 1.2, -1.0), ChVector<>(1.25, 0.8, -1));
        link_springRB->Set_SpringK(28300);
        link_springRB->Set_SpringR(80);
        my_system.AddLink(link_springRB);

        // .. create the rod for avoid the steering of the wheel
        link_distRBlat = std::make_shared<ChLinkDistance>();  // right rod
        link_distRBlat->Initialize(chassis, spindleRB, false, ChVector<>(0.5, 1.21, -1.4),
                                   ChVector<>(1.25, 1.21, -1.3));
        my_system.AddLink(link_distRBlat);

        // --- Left Back suspension ---

        // ..the car right-back spindle
        spindleLB = std::make_shared<ChBodyEasyBox>(0.1, 0.4, 0.4, 1.0, false, true);
        spindleLB->SetPos(ChVector<>(-1.3, 1, -1));
        spindleLB->SetMass(8);
        spindleLB->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        my_system.AddBody(spindleLB);

        // ..the car left-back wheel
        wheelLB = std::make_shared<ChBodyEasyCylinder>(0.45, 0.3, 1.0, true, true);
        wheelLB->SetPos(ChVector<>(-1.5, 1, -1));
        wheelLB->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
        wheelLB->SetMass(3);
        wheelLB->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2));
        wheelLB->GetMaterialSurfaceNSC()->SetFriction(1.0);
        wheelLB->AddAsset(texture);
        my_system.AddBody(wheelLB);

        // .. create the revolute joint between the wheel and the spindle
        link_revoluteLB = std::make_shared<ChLinkLockRevolute>();  // left, back, upper, 1
        link_revoluteLB->Initialize(wheelLB, spindleLB,
                                    ChCoordsys<>(ChVector<>(-1.5, 1, -1), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        my_system.AddLink(link_revoluteLB);

        // .. create the motor transmission joint between the wheel and the truss (assuming small changes of alignment)
        link_motorL = std::make_shared<ChLinkMotorRotationTorque>();
        link_motorL->Initialize(wheelLB, chassis,
                                ChFrame<>(ChVector<>(-1.5, 1, -1), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
        my_system.AddLink(link_motorL);

        // .. impose distance between two parts (as a massless rod with two spherical joints at the end)
        link_distLBU1 = std::make_shared<ChLinkDistance>();  // left, front, upper, 1
        link_distLBU1->Initialize(chassis, spindleLB, false, ChVector<>(-0.5, 1.2, -1.2), ChVector<>(-1.25, 1.2, -1));
        my_system.AddLink(link_distLBU1);

        link_distLBU2 = std::make_shared<ChLinkDistance>();  // left, back, upper, 2
        link_distLBU2->Initialize(chassis, spindleLB, false, ChVector<>(-0.5, 1.2, -0.8), ChVector<>(-1.25, 1.2, -1));
        my_system.AddLink(link_distLBU2);

        link_distLBL1 = std::make_shared<ChLinkDistance>();  // left, back, lower, 1
        link_distLBL1->Initialize(chassis, spindleLB, false, ChVector<>(-0.5, 0.8, -1.2), ChVector<>(-1.25, 0.8, -1));
        my_system.AddLink(link_distLBL1);

        link_distLBL2 = std::make_shared<ChLinkDistance>();  // left, back, lower, 2
        link_distLBL2->Initialize(chassis, spindleLB, false, ChVector<>(-0.5, 0.8, -0.8), ChVector<>(-1.25, 0.8, -1));
        my_system.AddLink(link_distLBL2);

        // .. create the spring between the truss and the spindle
        link_springLB = std::make_shared<ChLinkSpring>();
        link_springLB->Initialize(chassis, spindleLB, false, ChVector<>(-0.5, 1.2, -1.0), ChVector<>(-1.25, 0.8, -1));
        link_springLB->Set_SpringK(28300);
        link_springLB->Set_SpringR(80);
        my_system.AddLink(link_springLB);

        // .. create the rod for avoid the steering of the wheel
        link_distLBlat = std::make_shared<ChLinkDistance>();  // right
        link_distLBlat->Initialize(chassis, spindleLB, false, ChVector<>(-0.5, 1.21, -1.4),
                                   ChVector<>(-1.25, 1.21, -1.3));
        my_system.AddLink(link_distLBlat);
    }

    // Delete the car object, deleting also all bodies corresponding to
    // the various parts and removing them from the physical system.  Also
    // removes constraints from the system.
    ~MySimpleCar() {
        ChSystem* mysystem = spindleRF->GetSystem();  // trick to get the system here
        // When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
        // it is also automatically removed from the ChSystemNSC (the ChSystemNSC::RemoveBody() is
        // automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).

        // For links, just remove them from the ChSystemNSC using ChSystemNSC::RemoveLink()
        mysystem->RemoveLink(link_revoluteRF);
        mysystem->RemoveLink(link_distRFU1);
        mysystem->RemoveLink(link_distRFU2);
        mysystem->RemoveLink(link_distRFL1);
        mysystem->RemoveLink(link_distRFL2);
        mysystem->RemoveLink(link_springRF);
        mysystem->RemoveLink(link_distRSTEER);

        mysystem->RemoveLink(link_revoluteLF);
        mysystem->RemoveLink(link_distLFU1);
        mysystem->RemoveLink(link_distLFU2);
        mysystem->RemoveLink(link_distLFL1);
        mysystem->RemoveLink(link_distLFL2);
        mysystem->RemoveLink(link_springLF);
        mysystem->RemoveLink(link_distLSTEER);

        mysystem->RemoveLink(link_revoluteRB);
        mysystem->RemoveLink(link_distRBU1);
        mysystem->RemoveLink(link_distRBU2);
        mysystem->RemoveLink(link_distRBL1);
        mysystem->RemoveLink(link_distRBL2);
        mysystem->RemoveLink(link_springRB);
        mysystem->RemoveLink(link_distRBlat);
        mysystem->RemoveLink(link_motorR);

        mysystem->RemoveLink(link_revoluteLB);
        mysystem->RemoveLink(link_distLBU1);
        mysystem->RemoveLink(link_distLBU2);
        mysystem->RemoveLink(link_distLBL1);
        mysystem->RemoveLink(link_distLBL2);
        mysystem->RemoveLink(link_springLB);
        mysystem->RemoveLink(link_distLBlat);
        mysystem->RemoveLink(link_motorL);
    }

    // This can be used, at each time step, to compute the actual value of torque
    // transmitted to the wheels, according to gas throttle / speed / gear value.
    // The following is a very simplified model (the torque curve of the motor is linear
    // and no latency or inertial or clutch effects in gear train are considered.)
    double ComputeWheelTorque() {
        // Assume clutch is never used. Given the kinematics of differential,
        // the speed of the engine transmission shaft is the average of the two wheel speeds,
        // multiplied the conic gear transmission ratio inversed:
        double shaftspeed =
            (1.0 / this->conic_tau) * 0.5 * (link_motorL->GetMotorRot_dt() + link_motorR->GetMotorRot_dt());
        // The motorspeed is the shaft speed multiplied by gear ratio inversed:
        double motorspeed = (1.0 / this->gear_tau) * shaftspeed;
        // The torque depends on speed-torque curve of the motor: here we assume a
        // very simplified model a bit like in DC motors:
        double motortorque = max_motor_torque - motorspeed * (max_motor_torque / max_motor_speed);
        // Motor torque is linearly modulated by throttle gas value:
        motortorque = motortorque * this->throttle;
        // The torque at motor shaft:
        double shafttorque = motortorque * (1.0 / this->gear_tau);
        // The torque at wheels - for each wheel, given the differential transmission,
        // it is half of the shaft torque  (multiplied the conic gear transmission ratio)
        double singlewheeltorque = 0.5 * shafttorque * (1.0 / this->conic_tau);
        // Set the wheel torque in both 'motor' links, connecting the wheels to the truss;
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(link_motorL->GetTorqueFunction()))
            mfun->Set_yconst(singlewheeltorque);
        if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(link_motorR->GetTorqueFunction()))
            mfun->Set_yconst(singlewheeltorque);
        // debug:print infos on screen:
        // GetLog() << "motor torque="<< motortorque<< "  speed=" << motorspeed << "  wheel torqe=" << singlewheeltorque
        // <<"\n";
        // If needed, return also the value of wheel torque:
        return singlewheeltorque;
    }
};

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChSystemNSC* asystem, IrrlichtDevice* adevice, MySimpleCar* acar) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        msystem = asystem;
        mdevice = adevice;
        mcar = acar;

        adevice->setEventReceiver(this);

        // ..add a GUI slider to control gas throttle via mouse
        scrollbar_throttle = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 85, 150, 100), 0, 100);
        scrollbar_throttle->setMax(100);
        scrollbar_throttle->setPos(0);
        text_throttle = mdevice->getGUIEnvironment()->addStaticText(L"Throttle", rect<s32>(150, 85, 250, 100), false);

        // ..add a GUI slider to control steering via mouse
        scrollbar_steer = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 105, 150, 120), 0, 101);
        scrollbar_steer->setMax(100);
        scrollbar_steer->setPos(50);

        // ..add a GUI text and GUI slider to control the stiffness
        scrollbar_FspringK = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 125, 150, 140), 0, 102);
        scrollbar_FspringK->setMax(100);
        scrollbar_FspringK->setPos((s32)(50 + 50.0 * (acar->link_springRF->Get_SpringK() - 80000.0) / 60000.0));
        text_FspringK =
            mdevice->getGUIEnvironment()->addStaticText(L"Spring K [N/m]:", rect<s32>(150, 125, 250, 140), false);

        // ..add a GUI text and GUI slider to control the damping
        scrollbar_FdamperR = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 145, 150, 160), 0, 103);
        scrollbar_FdamperR->setMax(100);
        scrollbar_FdamperR->setPos((s32)(50 + 50.0 * (acar->link_springRF->Get_SpringR() - 800.0) / 800.0));
        text_FdamperR =
            mdevice->getGUIEnvironment()->addStaticText(L"Damper R [Ns/m]:", rect<s32>(150, 145, 250, 160), false);

        // ..add a GUI text and GUI slider to control the original undeformed spring length
        scrollbar_FspringL = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 165, 150, 180), 0, 104);
        scrollbar_FspringL->setMax(100);
        scrollbar_FspringL->setPos((s32)(50 + 50.0 * (acar->link_springRF->Get_SpringRestLength() - 0.9) / 0.1));
        text_FspringL =
            mdevice->getGUIEnvironment()->addStaticText(L"Spring L [m]:", rect<s32>(150, 165, 250, 180), false);
    }

    bool OnEvent(const SEvent& event) {
        // check if user moved the sliders with mouse..
        if (event.EventType == EET_GUI_EVENT) {
            s32 id = event.GUIEvent.Caller->getID();
            IGUIEnvironment* env = mdevice->getGUIEnvironment();

            switch (event.GUIEvent.EventType) {
                case EGET_SCROLL_BAR_CHANGED:
                    if (id == 101)  // id of 'steer' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newsteer = 0.18 * (((double)(pos - 50)) / 50.0);
                        // set the steering, moving horizontally the endpoints of the steer rod endpoint on truss.
                        this->mcar->link_distRSTEER->SetEndPoint1Rel(ChVector<>(0.5 + newsteer, 0.21, 1.4));
                        this->mcar->link_distLSTEER->SetEndPoint1Rel(ChVector<>(-0.5 + newsteer, 0.21, 1.4));
                    }
                    if (id == 102)  // id of 'spring stiffness' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newstiff = 80000 + 60000 * (((double)(pos - 50)) / 50.0);
                        // set the stiffness of all 4 springs
                        this->mcar->link_springRF->Set_SpringK(newstiff);
                        this->mcar->link_springLF->Set_SpringK(newstiff);
                        this->mcar->link_springRB->Set_SpringK(newstiff);
                        this->mcar->link_springLB->Set_SpringK(newstiff);

                        // show stiffness as formatted text in interface screen
                        char message[50];
                        sprintf(message, "Spring K [N/m]: %g", newstiff);
                        text_FspringK->setText(core::stringw(message).c_str());
                    }
                    if (id == 103)  // id of 'damping' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newdamping = 800 + 800 * (((double)(pos - 50)) / 50.0);
                        // set the damping of all 4 springs
                        this->mcar->link_springRF->Set_SpringR(newdamping);
                        this->mcar->link_springLF->Set_SpringR(newdamping);
                        this->mcar->link_springRB->Set_SpringR(newdamping);
                        this->mcar->link_springLB->Set_SpringR(newdamping);

                        // show stiffness as formatted text in interface screen
                        char message[50];
                        sprintf(message, "Damping R [Ns/m]: %g", newdamping);
                        text_FdamperR->setText(core::stringw(message).c_str());
                    }
                    if (id == 104)  // id of 'spring rest length' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newlength = 0.9 + 0.1 * (((double)(pos - 50)) / 50.0);
                        // set the rest length of all 4 springs
                        this->mcar->link_springRF->Set_SpringRestLength(newlength);
                        this->mcar->link_springLF->Set_SpringRestLength(newlength);
                        this->mcar->link_springRB->Set_SpringRestLength(newlength);
                        this->mcar->link_springLB->Set_SpringRestLength(newlength);

                        // show stiffness as formatted text in interface screen
                        char message[50];
                        sprintf(message, "Spring L [m]: %g", newlength);
                        text_FspringL->setText(core::stringw(message).c_str());
                    }
                    if (id == 100)  // id of 'throttle' slider..
                    {
                        s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
                        double newthrottle = ((double)(pos)) / 100.0;
                        // Set the throttle value of car (the torque transmitted
                        // to wheels depends on throttle, speed, transmission gear, so
                        // it will sent to the link_motorR and link_motorL only when
                        // computed by MySimplifiedCar::ComputeWheelTorque(),
                        this->mcar->throttle = newthrottle;
                    }
                    break;
                default:
                    break;
            }
        }

        return false;
    }

  private:
    ChSystemNSC* msystem;
    IrrlichtDevice* mdevice;
    MySimpleCar* mcar;

    IGUIScrollBar* scrollbar_steer;
    IGUIStaticText* text_FspringK;
    IGUIScrollBar* scrollbar_FspringK;
    IGUIStaticText* text_FdamperR;
    IGUIScrollBar* scrollbar_FdamperR;
    IGUIStaticText* text_FspringL;
    IGUIScrollBar* scrollbar_FspringL;
    IGUIStaticText* text_throttle;
    IGUIScrollBar* scrollbar_throttle;
};

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //

    // 1- Create a Chrono physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;

    // 2.- Create the Irrlicht visualization.
    ChIrrApp application(&my_system, L"Simple vehicle suspension", core::dimension2d<u32>(640, 480), false);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -6));

    // 3- Create the rigid bodies of the simpified car suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.

    // ..the world
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("blu.png"));

    auto my_ground = std::make_shared<ChBodyEasyBox>(60, 2, 60, 1.0, true, true);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->SetBodyFixed(true);
    my_ground->GetMaterialSurfaceNSC()->SetSfriction(1.0);
    my_ground->GetMaterialSurfaceNSC()->SetKfriction(1.0);
    my_ground->AddAsset(texture);
    my_system.AddBody(my_ground);

    // ..some obstacles on the ground:
    for (int i = 0; i < 6; i++) {
        auto my_obstacle = std::make_shared<ChBodyEasyBox>(1, 0.1, 0.5, 60.0, true, true);
        my_obstacle->SetPos(ChVector<>(20 * ChRandom(), 2, 20 * ChRandom()));
        my_obstacle->SetMass(3);
        my_system.AddBody(my_obstacle);
    }

    // ..the car (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleCar* mycar = new MySimpleCar(my_system, application.GetSceneManager(), application.GetVideoDriver());

    //
    // CREATE A CUSTOM COMPOSITE MATERIAL
    //

    // Suppose you want that some places have different friction coefficient values,
    // how can you do this? To change the default combination law for the friction
    // coefficient of a contact pair, use a 'callback object'. This will be called for
    // each contact point, as it is created, and can be used to modify the composite
    // contact material properties.

    class MyContactCallback : public ChContactContainer::AddContactCallback {
      public:
        virtual void OnAddContact(const collision::ChCollisionInfo& contactinfo,
                                  ChMaterialComposite* const material) override {
            // Downcast to appropriate composite material type
            auto mat = static_cast<ChMaterialCompositeNSC* const>(material);

            if (contactinfo.vpA.x() > 0)
                mat->static_friction = 0.7f;  // On the right of the plane, less friction...
            else
                mat->static_friction = 1.0f;  // On the left of the plane, more friction...
        };
    };

    // Use the above callback to process each contact as it is created.
    MyContactCallback mycontact_callback;
    my_system.GetContactContainer()->RegisterAddContactCallback(&mycontact_callback);

    // Bind visualization assets.
    application.AssetBindAll();
    application.AssetUpdateAll();

    //
    // USER INTERFACE
    //

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object -see above.
    MyEventReceiver receiver(&my_system, application.GetDevice(), mycar);

    //
    // SETTINGS
    //

    my_system.SetMaxItersSolverSpeed(20);  // the higher, the easier to keep the constraints 'mounted'.

    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    // This will help choosing an integration step which matches the
    // real-time step of the simulation..
    ChRealtimeStepTimer m_realtime_timer;

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        // .. draw a grid (rotated so that it's horizontal)
        ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30, 30,
                             ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 130, 130), true);

        // .. draw GUI user interface items (sliders, buttons) belonging to Irrlicht screen, if any
        application.GetIGUIEnvironment()->drawAll();

        // .. draw the distance constraints (the massless rods) as simplified lines
        // .. draw the spring constraints as simplified spring helix
        for (auto link : my_system.Get_linklist()) {
            if (auto linkdist = std::dynamic_pointer_cast<ChLinkDistance>(link)) {
                ChIrrTools::drawSegment(application.GetVideoDriver(), linkdist->GetEndPoint1Abs(),
                                        linkdist->GetEndPoint2Abs(), video::SColor(255, 0, 20, 0), true);
            } else if (auto linkspring = std::dynamic_pointer_cast<ChLinkSpring>(link)) {
                ChIrrTools::drawSpring(application.GetVideoDriver(), 0.03, linkspring->GetEndPoint1Abs(),
                                       linkspring->GetEndPoint2Abs(), video::SColor(255, 150, 20, 20), 80, 5, true);
            }
        }

        // The torque applied to wheels, using the ChLinkMotorRotationTorque links between
        // wheels and truss, depends on many parameters (gear, throttle, etc):
        mycar->ComputeWheelTorque();

        // HERE CHRONO INTEGRATION IS PERFORMED: THE
        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
        // STEP:

        my_system.DoStepDynamics(m_realtime_timer.SuggestSimulationStep(0.005));

        // Irrlicht must finish drawing the frame
        application.EndScene();
    }

    if (mycar)
        delete mycar;

    return 0;
}
