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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about creating a power train using the '1D' items of ChShaft class
// (rotating parts that have only one degree of freedom and one inertia value).
// This is an easier alternative to creating full 3D ChBody objects that rotate
// on revolute joints, etc.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChShaftsTorqueConverter.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChShaftsTorque.h"
#include "chrono/physics/ChShaftsThermalEngine.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    if (false) {
        //
        // EXAMPLE 1:
        //

        GetLog() << " Example: create a simple power train with ChShaft objects: \n";

        // We will model a very basic powertrain with two shafts A and B,
        // connected by a reducer [ t ] with transmision ratio 't'. Shafts are
        // free to rotate, shaft A has an applied torque Ta, so A and B will
        // constantly accelerate. Each shaft must have some inertia, it's like a
        // flywheel, marked as || in the following scheme:
        //
        //       A           B
        //  Ta  ||---[ t ]---||
        //

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create a 1-degree-of-freedom '1D' mechanical object, that
        // is a ChShaft (an item that can oly rotate, with one inertia value
        // and maybe one applied torque). The ChShaft objects do not have
        // any meaning in 3d: they are just 'building blocks' for making
        // power trains as in imput-output black box schemes.
        auto my_shaftA = std::make_shared<ChShaft>();
        my_shaftA->SetInertia(10);
        my_shaftA->SetAppliedTorque(6);
        my_system.Add(my_shaftA);

        // Create another shaft. Note that we use shared pointers for ChShaft
        // objects, as we did for ChBody objects. Also, note that we must add them
        // to the ChSystem.
        auto my_shaftB = std::make_shared<ChShaft>();
        my_shaftB->SetInertia(100);
        my_shaftB->SetShaftFixed(false);
        my_system.Add(my_shaftB);

        // Create a ChShaftsGear, that represents a simplified model
        // of a reducer, with transmission ratio t, between two ChShaft objects.
        // (Note that you could also build a 3D powertrain by creating full rigid bodies
        // of ChBody type and join them using ChLinkLockRevolute, ChLinkGear 3D constraints,
        // but this would introduce many unnecessary degrees of freedom/constraints
        // whereas the 1D items of ChShaft type, in this example, make things much simplier).
        auto my_shaft_gearAB = std::make_shared<ChShaftsGear>();
        my_shaft_gearAB->Initialize(my_shaftA, my_shaftB);
        my_shaft_gearAB->SetTransmissionRatio(-0.1);  // ex., a couple of spur gears with 20 and 200 teeth
        my_system.Add(my_shaft_gearAB);

        GetLog() << "\n\n\nHere's the system hierarchy: \n\n ";
        my_system.ShowHierarchy(GetLog());

        // Perform a very simple simulation loop..
        double chronoTime = 0;
        while (chronoTime < 2.5) {
            chronoTime += 0.01;

            // PERFORM SIMULATION UP TO chronoTime
            my_system.DoFrameDynamics(chronoTime);

            // Print something on the console..

            GetLog() << "Time: " << chronoTime << "\n"
                     << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPos_dt()
                     << "  accel: " << my_shaftA->GetPos_dtdt() << "\n"
                     << "  shaft B  rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPos_dt()
                     << "  accel: " << my_shaftB->GetPos_dtdt() << "\n"
                     << "  AB gear, torque on A side: " << my_shaft_gearAB->GetTorqueReactionOn1()
                     << "  AB gear, torque on B side: " << my_shaft_gearAB->GetTorqueReactionOn2() << "\n";
        }
    }

    if (false) {
        //
        // EXAMPLE 2:
        //

        GetLog() << " Example: a clutch between two shafts \n";

        // We will model a very basic powertrain with two shafts A and B,
        // connected by a clutch [ c ]. Shafts (see flywheels || in scheme)
        // starts with nonzero speed, and are free to rotate independently
        // until the clutch is activated: since activation, they will decelerate
        // until they have the same speed.
        //
        //       A           B
        //  Ta  ||---[ c ]---||
        //

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create a ChShaft that starts with nonzero angular velocity
        auto my_shaftA = std::make_shared<ChShaft>();
        my_shaftA->SetInertia(0.5);
        my_shaftA->SetPos_dt(30);
        my_system.Add(my_shaftA);

        // Create another ChShaft, with opposite initial angular velocity
        auto my_shaftB = std::make_shared<ChShaft>();
        my_shaftB->SetInertia(0.6);
        my_shaftB->SetPos_dt(-10);
        my_system.Add(my_shaftB);

        // Create a ChShaftsClutch, that represents a simplified model
        // of a clutch between two ChShaft objects (something that limits
        // the max transmitted torque, up to slippage).
        auto my_shaft_clutchAB = std::make_shared<ChShaftsClutch>();
        my_shaft_clutchAB->Initialize(my_shaftA, my_shaftB);
        my_shaft_clutchAB->SetTorqueLimit(60);
        my_system.Add(my_shaft_clutchAB);

        // Let's begin the simulation with the clutch disengaged:
        my_shaft_clutchAB->SetModulation(0);

        GetLog() << "\n\n\nHere's the system hierarchy: \n\n ";
        my_system.ShowHierarchy(GetLog());

        // Perform a very simple simulation loop..
        double chronoTime = 0;
        while (chronoTime < 1.5) {
            chronoTime += 0.01;

            // PERFORM SIMULATION UP TO chronoTime
            my_system.DoFrameDynamics(chronoTime);

            // Activate the clutch only after 0.8 seconds of simulation:
            if (chronoTime > 0.8) {
                my_shaft_clutchAB->SetModulation(1);
            }

            // Print something on the console..
            GetLog() << "Time: " << chronoTime << "\n"
                     << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPos_dt()
                     << "  accel: " << my_shaftA->GetPos_dtdt() << "\n"
                     << "  shaft B  rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPos_dt()
                     << "  accel: " << my_shaftB->GetPos_dtdt() << "\n"
                     << "  AB clutch, torque on A side: " << my_shaft_clutchAB->GetTorqueReactionOn1()
                     << "  AB clutch, torque on B side: " << my_shaft_clutchAB->GetTorqueReactionOn2() << "\n";
        }
    }

    if (false) {
        //
        // EXAMPLE 3:
        //

        GetLog() << " Example: an epicycloidal reducer \n";

        // We will model an epicycloidal reducer using the ChShaftsPlanetary
        // constraint.
        // The ChShaftsPlanetary makes a kinematic constraint between three
        // shafts: so one of them will be 'fixed' and will represent the truss
        // of the reducer -in epicycloidaal reducer, this is the role of the
        // large gear with inner teeth- and the two remaining shafts are the
        // input and output shafts (in other cases, such as the differential
        // planetary gear of the cars, all three shafts are free).
        // Also, a brake is applied for the output shaft: the ChShaftsClutch
        // will be used to this end, it's enough that one of the two shafts is fixed.
        // In the following scheme, the brake is [ b ], the planetary (the
        // reducer) is [ p ], the shafts are A,B,C,D applied torque is Ta, inertias
        // of free shafts are shown as flywheels || , and fixed shafts are marked with * .
        //
        //       A           B            D
        //  Ta  ||---[ p ]---||---[ b ]---*
        //           [   ]---*
        //                   C

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create shaft A, with applied torque
        auto my_shaftA = std::make_shared<ChShaft>();
        my_shaftA->SetInertia(0.5);
        my_shaftA->SetAppliedTorque(10);
        my_system.Add(my_shaftA);

        // Create shaft B
        auto my_shaftB = std::make_shared<ChShaft>();
        my_shaftB->SetInertia(0.5);
        my_system.Add(my_shaftB);

        // Create shaft C, that will be fixed (to be used as truss of epicycloidal reducer)
        auto my_shaftC = std::make_shared<ChShaft>();
        my_shaftC->SetShaftFixed(true);
        my_system.Add(my_shaftC);

        // Create a ChShaftsPlanetary, that represents a simplified model
        // of a planetary gear between THREE ChShaft objects (ex.: a car differential)
        // An epicycloidal reducer is a special type of planetary gear.
        auto my_shaft_planetaryBAC = std::make_shared<ChShaftsPlanetary>();
        my_shaft_planetaryBAC->Initialize(my_shaftB, my_shaftA, my_shaftC);  // output, carrier, fixed
        // We can set the ratios of the planetary using a simplified formula, for the
        // so called 'Willis' case. Imagine we hold fixed the carrier (shaft B in epic. reducers),
        // and leave free the truss C (the outer gear with inner teeth in our reducer); which is
        // the transmission ratio t0 that we get? It is simply t0=-Za/Zc, with Z = num of teeth of gears.
        // So just use the following to set all the three ratios automatically:
        double t0 =
            -50.0 / 100.0;  // suppose, in the reducer, that pinion A has 50 teeth and truss has 100 inner teeth.
        my_shaft_planetaryBAC->SetTransmissionRatioOrdinary(t0);
        my_system.Add(my_shaft_planetaryBAC);

        // Now, let's make a shaft D, that is fixed, and used for the right side
        // of a clutch (so the clutch will act as a brake).
        auto my_shaftD = std::make_shared<ChShaft>();
        my_shaftD->SetShaftFixed(true);
        my_system.Add(my_shaftD);

        // Make the brake. It is, in fact a clutch between shafts B and D, where
        // D is fixed as a truss, so the clutch will operate as a brake.
        auto my_shaft_clutchBD = std::make_shared<ChShaftsClutch>();
        my_shaft_clutchBD->Initialize(my_shaftB, my_shaftD);
        my_shaft_clutchBD->SetTorqueLimit(60);
        my_system.Add(my_shaft_clutchBD);

        GetLog() << "\n\n\nHere's the system hierarchy: \n\n ";
        my_system.ShowHierarchy(GetLog());

        // Perform a very simple simulation loop..
        double chronoTime = 0;
        while (chronoTime < 1.5) {
            chronoTime += 0.01;

            // PERFORM SIMULATION UP TO chronoTime
            my_system.DoFrameDynamics(chronoTime);

            // Print something on the console..
            GetLog() << "Time: " << chronoTime << "\n"
                     << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPos_dt()
                     << "  accel: " << my_shaftA->GetPos_dtdt() << "\n"
                     << "  shaft B  rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPos_dt()
                     << "  accel: " << my_shaftB->GetPos_dtdt() << "\n"
                     << "  epicycloidal react torques on shafts - on A: "
                     << my_shaft_planetaryBAC->GetTorqueReactionOn2()
                     << " ,   on B: " << my_shaft_planetaryBAC->GetTorqueReactionOn1()
                     << " ,   on C: " << my_shaft_planetaryBAC->GetTorqueReactionOn3() << "\n";
        }
    }

    if (false) {
        //
        // EXAMPLE 4:
        //

        GetLog() << " Example: constraint between a ChBody and a ChShaft \n";

        // Suppose you want to create a 3D model, for instance a slider-crank,
        // built with multiple ChBody objects; moreover you want to create a
        // powertrain, for instance a motor, a clutch, etc, for the rotation of
        // the crank. How to connect the '1D items' of ChShaft class to the 3D
        // items of ChBody class? The solution is to use the ChShaftsBody constraint,
        // shown as [ bs ] in the following scheme, where the 3D body is shown as <>.
        // In this example we also add a 'torsional spring damper' C, shown as [ t ]
        // that connects shafts A and C (C is shown as * because fixed).
        //
        //        B             A           C
        //  Ta   <>---[ bs ]---||---[ t ]---*
        //

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create 'A', a 1D shaft
        auto my_shaftA = std::make_shared<ChShaft>();
        my_shaftA->SetInertia(9);
        my_system.Add(my_shaftA);

        // Create 'C', a 1D shaft, fixed
        auto my_shaftC = std::make_shared<ChShaft>();
        my_shaftC->SetShaftFixed(true);
        my_system.Add(my_shaftC);

        // Create 'B', a 3D rigid body
        auto my_bodyB = std::make_shared<ChBody>();
        my_bodyB->Accumulate_torque(ChVector<>(0, 0, 3), true);  // set some constant torque to body
        my_system.Add(my_bodyB);

        // Make the torsional spring-damper between shafts A and C.
        auto my_shaft_torsionAC = std::make_shared<ChShaftsTorsionSpring>();
        my_shaft_torsionAC->Initialize(my_shaftA, my_shaftC);
        my_shaft_torsionAC->SetTorsionalStiffness(40);
        my_shaft_torsionAC->SetTorsionalDamping(0);
        my_system.Add(my_shaft_torsionAC);

        // Make the shaft 'A' connected to the rotation of the 3D body 'B'.
        // We must specify the direction (in body coordinates) along which the
        // shaft will affect the body.
        auto my_shaftbody_connection = std::make_shared<ChShaftsBody>();
        ChVector<> mshaftdir(VECT_Z);
        my_shaftbody_connection->Initialize(my_shaftA, my_bodyB, mshaftdir);
        my_system.Add(my_shaftbody_connection);

        GetLog() << "\n\n\nHere's the system hierarchy: \n\n ";
        my_system.ShowHierarchy(GetLog());

        // Perform a very simple simulation loop..
        double chronoTime = 0;
        while (chronoTime < 0.5) {
            chronoTime += 0.01;

            // PERFORM SIMULATION UP TO chronoTime
            my_system.DoFrameDynamics(chronoTime);

            // Print something on the console..
            GetLog() << "Time: " << chronoTime << "\n"
                     << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPos_dt()
                     << "  accel: " << my_shaftA->GetPos_dtdt() << "\n"
                     << "  Body B angular speed on z: " << my_bodyB->GetWvel_loc().z()
                     << "  accel on z: " << my_bodyB->GetWacc_loc().z() << "\n"
                     << "  AC spring, torque on A side: " << my_shaft_torsionAC->GetTorqueReactionOn1()
                     << "  torque on C side: " << my_shaft_torsionAC->GetTorqueReactionOn2() << "\n"
                     << "  shafts/body reaction,  on shaft A: " << my_shaftbody_connection->GetTorqueReactionOnShaft()
                     << " ,   on body (x y z): " << my_shaftbody_connection->GetTorqueReactionOnBody().x() << " "
                     << my_shaftbody_connection->GetTorqueReactionOnBody().y() << " "
                     << my_shaftbody_connection->GetTorqueReactionOnBody().z() << " "
                     << "\n";
        }
    }

    if (true) {
        //
        // EXAMPLE 5:
        //

        GetLog() << " Example: torque converter and thermal engine \n";

        // In this example we use a torque converter.
        // The torque converter is represented by a ChShaftsTorqueConverter
        // object, that connects three '1D items' of ChShaft class:
        // - the input shaft A, ie. the impeller connected to the engine
        // - the output shaft B, i.e. the turbine connected to the gears and wheels
        // - the stator C, that does not rotate and transmits reaction to the truss.
        // In the following scheme, the torque converter is represented as [ tc ],
        // and we also add a thermal engine, shown with [ e ], and a breaking torque Tb
        // (C is shown as * because fixed).
        //
        //   D           A             B
        //   *---[ e ]---||---[ tc ]---||  Tb
        //                    [    ]---*
        //                             C
        //

        // The physical system: it contains all physical objects.
        ChSystem my_system;

        // Create 'A', a 1D shaft
        auto my_shaftA = std::make_shared<ChShaft>();
        my_shaftA->SetInertia(1.5);
        my_system.Add(my_shaftA);

        // Create 'B', a 1D shaft
        auto my_shaftB = std::make_shared<ChShaft>();
        my_shaftB->SetInertia(3.2);
        my_shaftB->SetAppliedTorque(-5);  // apply const braking torque
        my_system.Add(my_shaftB);

        // Create 'C', a 1D shaft, fixed
        auto my_shaftC = std::make_shared<ChShaft>();
        my_shaftC->SetShaftFixed(true);
        my_system.Add(my_shaftC);

        // Create 'D', a 1D shaft, fixed
        auto my_shaftD = std::make_shared<ChShaft>();
        my_shaftD->SetShaftFixed(true);
        my_system.Add(my_shaftD);

        // Make the torque converter and connect the shafts:
        // A (input),B (output), C(truss stator)
        auto my_torqueconverter = std::make_shared<ChShaftsTorqueConverter>();
        my_torqueconverter->Initialize(my_shaftA, my_shaftB, my_shaftC);
        my_system.Add(my_torqueconverter);

        auto mK = std::make_shared<ChFunction_Recorder>();
        mK->AddPoint(0.0, 15);
        mK->AddPoint(0.25, 15);
        mK->AddPoint(0.50, 15);
        mK->AddPoint(0.75, 16);
        mK->AddPoint(0.90, 18);
        mK->AddPoint(1.00, 35);
        my_torqueconverter->SetCurveCapacityFactor(mK);

        auto mT = std::make_shared<ChFunction_Recorder>();
        mT->AddPoint(0.0, 2.00);
        mT->AddPoint(0.25, 1.80);
        mT->AddPoint(0.50, 1.50);
        mT->AddPoint(0.75, 1.15);
        mT->AddPoint(1.00, 1.00);
        my_torqueconverter->SetCurveTorqueRatio(mT);

        // Make the thermal engine, acting on shaft A, the input to
        // the torque converter. Note that the thermal engine also
        // requires another shaft D, that is used to transmit the
        // reaction torque back to a truss (the motor block).

        // Option A: use a ChShaftsMotor, in the MOT_MODE_TORQUE mode.
        //  It works, but most often this is more useful when in MOT_MODE_SPEED.
        /*
        auto my_motor = std::make_shared<ChShaftsMotor>();
        my_motor->Initialize(my_shaftA, my_shaftD);
        my_motor->SetMotorMode(ChShaftsMotor::MOT_MODE_TORQUE);
        my_motor->SetMotorTorque(30);
        my_system.Add(my_motor);
        */

        // Option B: use a ChShaftsTorque, it just applies a torque
        // to my_shaftA (say, the crankshaft) and the negative torque
        // to my_shaftD (say, the motor block).
        // It is a quick approach. But you should take care of changing
        // the torque at each timestep if you want to simulate a torque curve...
        /*
        auto my_motor = std::make_shared<ChShaftsTorque>();
        my_motor->Initialize(my_shaftA, my_shaftD);
        my_motor->SetTorque(30);
        my_system.Add(my_motor);
        */

        // Option C: a more powerful approach where you can
        // define a torque curve and a throttle value, using the
        // ChShaftsThermalEngine.

        auto my_motor = std::make_shared<ChShaftsThermalEngine>();
        my_motor->Initialize(my_shaftA, my_shaftD);
        my_system.Add(my_motor);

        auto mTw = std::make_shared<ChFunction_Recorder>();
        mTw->AddPoint(-5, 30);  //   [rad/s],  [Nm]
        mTw->AddPoint(0, 30);
        mTw->AddPoint(200, 60);
        mTw->AddPoint(400, 40);
        mTw->AddPoint(450, 0);
        mTw->AddPoint(500, -60);  // torque curve must be defined beyond max speed too - engine might be 'pulled'
        my_motor->SetTorqueCurve(mTw);

        GetLog() << "\n\n\nHere's the system hierarchy: \n\n ";
        my_system.ShowHierarchy(GetLog());

        // Perform a very simple simulation loop..
        double chronoTime = 0;
        while (chronoTime < 4.5) {
            chronoTime += 0.01;

            // PERFORM SIMULATION UP TO chronoTime
            my_system.DoFrameDynamics(chronoTime);

            // Print something on the console..
            GetLog() << "Time: " << chronoTime << "\n"
                     << "  shaft A rot: " << my_shaftA->GetPos() << "  speed: " << my_shaftA->GetPos_dt()
                     << "  accel: " << my_shaftA->GetPos_dtdt() << "\n"
                     << "  shaft B rot: " << my_shaftB->GetPos() << "  speed: " << my_shaftB->GetPos_dt()
                     << "  accel: " << my_shaftB->GetPos_dtdt() << "\n"
                     << "  T.Convert.:"
                     << "  R=" << my_torqueconverter->GetSpeedRatio()
                     << "  Tin=" << my_torqueconverter->GetTorqueReactionOnInput()
                     << "  Tout=" << my_torqueconverter->GetTorqueReactionOnOutput()
                     << "  Tstator=" << my_torqueconverter->GetTorqueReactionOnStator() << "\n"
                     << "  T.Motor: "
                     << "  T(w)=" << my_motor->GetTorqueReactionOn1() << "[Nm]"
                     << "  w=" << my_motor->GetRelativeRotation_dt() << "[rad/s]"
                     << "\n";
        }
    }

    return 0;
}
